#include <Geode/Geode.hpp>
#include <Geode/ui/GeodeUI.hpp>
#include <Geode/utils/coro.hpp>
#include <Geode/modify/EditLevelLayer.hpp>
#include <Geode/modify/EditorPauseLayer.hpp>
#include <Geode/modify/GJBaseGameLayer.hpp>
#include <Geode/modify/LevelInfoLayer.hpp>
#include <Geode/modify/PauseLayer.hpp>
#include <Geode/modify/PlayLayer.hpp>
#include <UIBuilder.hpp>
#include "pathfinder.hpp"
#include "flappy.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <future>

using namespace geode::prelude;
using namespace geode::utils::file;

namespace {

constexpr double kPathfinderTPS = 240.0;

struct StoredSolution {
    std::string levelName;
    std::vector<PathfinderInput> inputs;
    bool armed = false;
};

struct PlaybackRuntime {
    PlayLayer* layer = nullptr;
    std::vector<PathfinderInput> inputs;
    size_t cursor = 0;
    std::array<std::array<bool, 4>, 2> held {};
    bool active = false;
};

StoredSolution g_solution;
PlaybackRuntime g_playback;

std::string levelNameOf(GJGameLevel* level) {
    return level ? std::string(level->m_levelName.c_str()) : std::string();
}

std::string decompressedLevelString(GJGameLevel* level) {
    if (!level)
        return {};

    std::string encoded = level->m_levelString.c_str();
    auto decoded = ZipUtils::decompressString(encoded, true, 0);
    if (!decoded.empty())
        return decoded;
    return encoded;
}

char const* vehicleName(int type) {
    switch (type) {
        case 0: return "Cube";
        case 1: return "Ship";
        case 2: return "Ball";
        case 3: return "UFO";
        case 4: return "Wave";
        case 5: return "Robot";
        case 6: return "Spider";
        case 7: return "Swing";
        default: return "Unknown";
    }
}

char const* searchPhaseName(int phase) {
    switch (phase) {
        case 0: return "Structured";
        case 1: return "Testing";
        case 2: return "Recovering";
        case 3: return "Advancing";
        default: return "Searching";
    }
}

void releasePlaybackButtons(PlayLayer* play) {
    if (!play)
        return;

    for (size_t player = 0; player < g_playback.held.size(); ++player) {
        for (int button = 1; button <= 3; ++button) {
            if (!g_playback.held[player][button])
                continue;
            play->GJBaseGameLayer::handleButton(false, button, player == 1);
            g_playback.held[player][button] = false;
        }
    }
}

void seekPlayback(PlayLayer* play, double seconds, bool syncButtons = true) {
    if (!g_playback.active)
        return;

    uint32_t frame = static_cast<uint32_t>(std::llround(
        std::max(0.0, seconds) * kPathfinderTPS
    ));

    std::array<std::array<bool, 4>, 2> desired {};
    size_t cursor = 0;

    while (cursor < g_playback.inputs.size() && g_playback.inputs[cursor].frame < frame) {
        auto const& input = g_playback.inputs[cursor++];
        if (input.button < 1 || input.button > 3)
            continue;
        desired[input.player2 ? 1 : 0][input.button] = input.down;
    }

    if (syncButtons && play) {
        releasePlaybackButtons(play);
        for (size_t player = 0; player < desired.size(); ++player) {
            for (int button = 1; button <= 3; ++button) {
                if (desired[player][button])
                    play->GJBaseGameLayer::handleButton(true, button, player == 1);
            }
        }
    }

    g_playback.held = desired;
    g_playback.cursor = cursor;
}

void preparePlayback(PlayLayer* layer) {
    if (g_playback.active && g_playback.layer)
        releasePlaybackButtons(g_playback.layer);

    g_playback = {};
    if (!layer || !layer->m_level || !g_solution.armed)
        return;
    if (levelNameOf(layer->m_level) != g_solution.levelName)
        return;

    g_playback.layer = layer;
    g_playback.inputs = g_solution.inputs;
    g_playback.active = !g_playback.inputs.empty();
    seekPlayback(layer, layer->m_attemptTime);

    if (g_playback.active)
        layer->m_level->m_isCompletionLegitimate = false;
}

void armSolution(std::string const& levelName, PathfinderResult const& result) {
    g_solution.levelName = levelName;
    g_solution.inputs = result.inputs;
    g_solution.armed = !g_solution.inputs.empty();

    if (auto* play = PlayLayer::get(); play && play->m_level && levelNameOf(play->m_level) == levelName) {
        preparePlayback(play);
        if (g_playback.active) {
            queueInMainThread([play] {
                if (PlayLayer::get() == play)
                    play->resetLevelFromStart();
            });
        }
    }
}

void feedPlayback(PlayLayer* play) {
    if (!g_playback.active || g_playback.layer != play)
        return;

    if (play->m_player1 && play->m_player1->m_isDead) {
        releasePlaybackButtons(play);
        return;
    }

    uint32_t frame = static_cast<uint32_t>(std::llround(
        std::max(0.0, play->m_attemptTime) * kPathfinderTPS
    ));

    while (g_playback.cursor < g_playback.inputs.size() &&
           g_playback.inputs[g_playback.cursor].frame <= frame) {
        auto const input = g_playback.inputs[g_playback.cursor++];
        if (input.button < 1 || input.button > 3)
            continue;

        play->GJBaseGameLayer::handleButton(
            input.down,
            static_cast<int>(input.button),
            input.player2
        );
        g_playback.held[input.player2 ? 1 : 0][input.button] = input.down;
    }
}

void openPathfinderSettings() {
    geode::openSettingsPopup(Mod::get());
}

CCMenu* resolvePauseMenu(CCNode* layer, char const* fallbackID, CCPoint fallbackPosition) {
    if (auto* menu = typeinfo_cast<CCMenu*>(layer->getChildByID("right-button-menu")))
        return menu;

    auto* menu = CCMenu::create();
    menu->setID(fallbackID);
    menu->setPosition(fallbackPosition);
    layer->addChild(menu, 20);
    return menu;
}

} // namespace

class PathfinderNode : public CCLayerColor {
    std::atomic_bool m_stop = false;
    std::atomic<double> m_progress = 0;
    std::atomic<float> m_currentX = 0.f;
    std::atomic<float> m_deathX = 0.f;
    std::atomic<float> m_clearance = 0.f;
    std::atomic<int> m_vehicleType = 0;
    std::atomic<int> m_searchLevel = 0;
    std::atomic<int> m_horizonFrames = 0;
    std::atomic<int> m_candidateCount = 0;
    std::atomic<int> m_workerCount = 1;
    std::atomic<int> m_phase = 0;
    std::atomic<uint64_t> m_totalTrials = 0;
    std::future<PathfinderResult> m_result;
    std::string m_levelName;

public:
    static PathfinderNode* create(
        std::string const& levelName,
        std::string const& lvlString,
        float trustedEndX = 0.f
    ) {
        auto* node = new PathfinderNode();
        if (node && node->init(levelName, lvlString, trustedEndX)) {
            node->autorelease();
            return node;
        }
        CC_SAFE_DELETE(node);
        return nullptr;
    }

    ~PathfinderNode() {
        m_stop = true;
        if (m_result.valid())
            (void)m_result.get();
    }

    void finalize(PathfinderResult result) {
        if (auto* stop = getChildByIDRecursive("stop"))
            stop->setVisible(false);
        if (auto* waitMenu = getChildByID("pathfinder-wait-menu"))
            waitMenu->setVisible(false);
        if (auto* game = getChildByID("pathfinder-flappy-game"))
            game->removeFromParentAndCleanup(true);
        if (auto* debug = getChildByIDRecursive("solver-debug"))
            debug->setVisible(false);

        auto* percent = typeinfo_cast<CCLabelBMFont*>(getChildByIDRecursive("percent"));
        if (percent) {
            auto progressText = result.complete
                ? fmt::format("Solved - {:.2f}%", result.progress)
                : fmt::format("Best path - {:.2f}%", result.progress);
            percent->setString(progressText.c_str());
        }

        if (!result.diagnostics.empty())
            log::info("Pathfinder diagnostics: {}", result.diagnostics);

        bool autoApply = Mod::get()->getSettingValue<bool>("auto-apply-inputs");
        bool hasInputs = !result.inputs.empty();
        if (autoApply && hasInputs)
            armSolution(m_levelName, result);

        auto* menu = typeinfo_cast<CCMenu*>(getChildByID("menu"));
        if (!menu)
            return;

        if (hasInputs && !autoApply) {
            Build<ButtonSprite>::create("Apply Now", "bigFont.fnt", "GJ_button_01.png")
                .intoMenuItem([this, result](CCMenuItemSpriteExtra*) {
                    armSolution(m_levelName, result);
                    if (auto* label = typeinfo_cast<CCLabelBMFont*>(getChildByIDRecursive("status")))
                        label->setString("Autoplay armed");
                })
                .scale(0.65f)
                .move(-55, -72)
                .parent(menu);
        }

        auto exportCallback = [this, macro = result.macro](this auto self) -> arc::Future<void> {
            if (macro.empty())
                co_return;

            auto saveDir = Mod::get()->getSaveDir();
            if (Loader::get()->isModLoaded("eclipse.eclipse-menu"))
                saveDir = Loader::get()->getLoadedMod("eclipse.eclipse-menu")->getSaveDir() / "replays";

            if (!exists(saveDir))
                create_directories(saveDir);

            FilePickOptions opts(
                saveDir / fmt::format("{}.gdr2", m_levelName), {{
                    std::string("Macro File"),
                    std::unordered_set {std::string("gdr2")}
                }}
            );

            if (auto path = co_await pick(PickMode::SaveFile, opts); path.isOk() && path.unwrap().has_value()) {
                (void)writeBinary(*path.unwrap(), macro);
                queueInMainThread([this] {
                    removeFromParentAndCleanup(true);
                });
            }
        };

        Build<ButtonSprite>::create("Export Macro", "bigFont.fnt", "GJ_button_01.png")
            .intoMenuItem(async::wrapSpawn(exportCallback))
            .scale(0.65f)
            .move(autoApply || !hasInputs ? 0 : 55, -72)
            .parent(menu);

        char const* statusText = !hasInputs
            ? "No usable input path was found"
            : autoApply
                ? "Autoplay armed automatically"
                : "Choose how to use this path";

        Build<CCLabelBMFont>::create(statusText, "chatFont.fnt")
            .id("status")
            .scale(0.55f)
            .move(0, -42)
            .parent(menu);
    }

    void keyBackClicked() override {
        m_stop = true;
        CCLayer::keyBackClicked();
        removeFromParentAndCleanup(true);
    }

    bool init(
        std::string const& levelName,
        std::string const& lvlString,
        float trustedEndX
    ) {
        if (!CCLayerColor::initWithColor({0, 0, 0, 100}))
            return false;
        setCascadeOpacityEnabled(true);
        m_levelName = levelName;

        m_result = std::async(std::launch::async, [lvlString, trustedEndX, this]() {
            try {
                return pathfind(lvlString, m_stop, [this](PathfinderTelemetry const& telemetry) {
                    if (m_progress < telemetry.progress)
                        m_progress = telemetry.progress;
                    m_currentX = telemetry.currentX;
                    m_deathX = telemetry.deathX;
                    m_clearance = telemetry.bestClearance;
                    m_vehicleType = telemetry.vehicleType;
                    m_searchLevel = telemetry.searchLevel;
                    m_horizonFrames = telemetry.horizonFrames;
                    m_candidateCount = telemetry.candidateCount;
                    m_workerCount = telemetry.workerCount;
                    m_phase = telemetry.phase;
                    m_totalTrials = telemetry.totalTrials;

                    log::debug(
                        "Pathfinder {} mode={} progress={:.2f}% currentX={:.2f} deathX={:.2f} level={} horizon={} candidates={} workers={} trials={} clearance={:.2f}",
                        telemetry.recoveryReason,
                        telemetry.vehicleType,
                        telemetry.progress,
                        telemetry.currentX,
                        telemetry.deathX,
                        telemetry.searchLevel,
                        telemetry.horizonFrames,
                        telemetry.candidateCount,
                        telemetry.workerCount,
                        telemetry.totalTrials,
                        telemetry.bestClearance
                    );
                }, trustedEndX);
            } catch (std::exception const& e) {
                log::error("Pathfinder failed: {}", e.what());
                return PathfinderResult {};
            }
        });

        setKeypadEnabled(true);
        Build(this).initTouch().schedule([this](float) {
            if (auto* label = typeinfo_cast<CCLabelBMFont*>(getChildByIDRecursive("percent"))) {
                auto text = fmt::format("{:.2f}%", m_progress.load());
                label->setString(text.c_str());
            }

            if (auto* debug = typeinfo_cast<CCLabelBMFont*>(getChildByIDRecursive("solver-debug"))) {
                auto text = fmt::format(
                    "{} | {} | L{} | {} workers\nX {:.0f} | death {:.0f} | clearance {:.1f}\n{} trials | horizon {} | {} candidates",
                    vehicleName(m_vehicleType.load()),
                    searchPhaseName(m_phase.load()),
                    m_searchLevel.load(),
                    m_workerCount.load(),
                    m_currentX.load(),
                    m_deathX.load(),
                    m_clearance.load(),
                    m_totalTrials.load(),
                    m_horizonFrames.load(),
                    m_candidateCount.load()
                );
                debug->setString(text.c_str());
            }

            if (m_result.valid() && m_result.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
                finalize(m_result.get());
        });

        auto handle = [this](CCMenuItemSpriteExtra* item) {
            m_stop = true;
            if (item->getID() == "stop") {
                if (m_result.valid())
                    finalize(m_result.get());
            } else {
                removeFromParentAndCleanup(true);
            }
        };

        Build<CCMenu>::create().parent(this).id("menu").children(
            Build<CCScale9Sprite>::create("GJ_square02.png")
                .contentSize(320, 215),
            Build<CCLabelBMFont>::create("Pathfinding...", "bigFont.fnt")
                .move(0, 82)
                .scale(0.8f),
            Build<CCLabelBMFont>::create("0.00%", "chatFont.fnt")
                .id("percent")
                .move(0, 48),
            Build<CCLabelBMFont>::create("Starting solver...", "chatFont.fnt")
                .id("solver-debug")
                .scale(0.42f)
                .move(0, 0),
            Build<ButtonSprite>::create("Stop", "bigFont.fnt", "GJ_button_04.png")
                .scale(0.72f)
                .intoMenuItem(handle)
                .id("stop")
                .move(0, -82),
            Build<CCSprite>::createSpriteName("GJ_closeBtn_001.png")
                .intoMenuItem(handle)
                .id("close")
                .move(-160, 107)
                .scale(0.8f)
        );

        auto win = CCDirector::sharedDirector()->getWinSize();
        auto* waitMenu = CCMenu::create();
        waitMenu->setID("pathfinder-wait-menu");
        waitMenu->setPosition({
            std::min(win.width - 55.f, win.width / 2.f + 202.f),
            win.height / 2.f + 105.f
        });
        addChild(waitMenu, 220);

        Build<ButtonSprite>::create("Flappy", "bigFont.fnt", "GJ_button_01.png")
            .scale(0.48f)
            .intoMenuItem([this](CCMenuItemSpriteExtra*) {
                togglePathfinderFlappy(this);
            })
            .parent(waitMenu);

        return true;
    }
};

struct PathfinderGameLayer : geode::Modify<PathfinderGameLayer, GJBaseGameLayer> {
    void processCommands(float dt, bool isHalfTick, bool isLastTick) {
        auto* play = typeinfo_cast<PlayLayer*>(static_cast<GJBaseGameLayer*>(this));
        if (play)
            feedPlayback(play);
        GJBaseGameLayer::processCommands(dt, isHalfTick, isLastTick);
    }
};

struct PathfinderPlayLayer : geode::Modify<PathfinderPlayLayer, PlayLayer> {
    bool init(GJGameLevel* level, bool useReplay, bool dontCreateObjects) {
        if (!PlayLayer::init(level, useReplay, dontCreateObjects))
            return false;
        preparePlayback(this);
        return true;
    }

    void resetLevel() {
        if (g_playback.layer == this && g_playback.active)
            releasePlaybackButtons(this);
        PlayLayer::resetLevel();
        if (g_playback.layer == this && g_playback.active)
            seekPlayback(this, m_attemptTime);
    }

    void onQuit() {
        if (g_playback.layer == this) {
            releasePlaybackButtons(this);
            g_playback = {};
        }
        PlayLayer::onQuit();
    }
};

struct PathfinderEditLevelLayer : geode::Modify<PathfinderEditLevelLayer, EditLevelLayer> {
    bool init(GJGameLevel* level) {
        if (!EditLevelLayer::init(level))
            return false;

        auto btn = Build<BasedButtonSprite>::create(
            CCSprite::create("pathfinder.png"_spr), BaseType::Circle, 4, 3
        ).scale(0.8f);
        btn->setTopRelativeScale(1.4f);

        btn.intoMenuItem([this]() {
                Build<PathfinderNode>::create(levelNameOf(m_level), decompressedLevelString(m_level))
                    .parent(this).zOrder(100);
            })
            .id("pathfinder-button")
            .intoNewParent(CCMenu::create())
            .parent(this)
            .id("pathfinder-menu")
            .matchPos(getChildByIDRecursive("delete-button"))
            .move(-45, 0);
        return true;
    }
};

struct PathfinderLevelInfoLayer : geode::Modify<PathfinderLevelInfoLayer, LevelInfoLayer> {
    bool init(GJGameLevel* level, bool challenge) {
        if (!LevelInfoLayer::init(level, challenge))
            return false;

        auto btn = Build<BasedButtonSprite>::create(
            CCSprite::create("pathfinder.png"_spr), BaseType::Circle, 4, 3
        ).scale(0.8f);
        btn->setTopRelativeScale(1.4f);

        btn.intoMenuItem([this]() {
                Build<PathfinderNode>::create(levelNameOf(m_level), decompressedLevelString(m_level))
                    .parent(this).zOrder(100);
            })
            .id("pathfinder-button")
            .parent(getChildByID("other-menu"))
            .matchPos(getChildByIDRecursive("list-button"))
            .move(0, 45);
        return true;
    }
};

struct PathfinderPauseLayer : geode::Modify<PathfinderPauseLayer, PauseLayer> {
    void customSetup() {
        PauseLayer::customSetup();

        auto win = CCDirector::sharedDirector()->getWinSize();
        auto* menu = resolvePauseMenu(this, "pathfinder-pause-menu", ccp(win.width - 50.f, win.height - 55.f));
        if (!menu)
            return;

        if (!getChildByIDRecursive("pathfinder-run-button")) {
            auto runBtn = Build<BasedButtonSprite>::create(
                CCSprite::create("pathfinder.png"_spr), BaseType::Circle, 4, 3
            ).scale(0.62f);
            runBtn->setTopRelativeScale(1.35f);

            runBtn.intoMenuItem([this]() {
                    if (auto* play = PlayLayer::get(); play && play->m_level) {
                        float endX = play->getEndPosition().x;
                        Build<PathfinderNode>::create(
                            levelNameOf(play->m_level), decompressedLevelString(play->m_level), endX
                        ).parent(this).zOrder(200);
                    }
                })
                .id("pathfinder-run-button")
                .parent(menu);

            Build<CCSprite>::createSpriteName("GJ_optionsBtn_001.png")
                .scale(0.62f)
                .intoMenuItem([](CCMenuItemSpriteExtra*) { openPathfinderSettings(); })
                .id("pathfinder-settings-button")
                .parent(menu);
            menu->updateLayout();
        }
    }
};

struct PathfinderEditorPauseLayer : geode::Modify<PathfinderEditorPauseLayer, EditorPauseLayer> {
    void customSetup() {
        EditorPauseLayer::customSetup();

        auto win = CCDirector::sharedDirector()->getWinSize();
        auto* menu = CCMenu::create();
        menu->setID("pathfinder-editor-pause-menu");
        menu->setPosition(ccp(win.width - 48.f, win.height - 48.f));
        addChild(menu, 30);

        auto runBtn = Build<BasedButtonSprite>::create(
            CCSprite::create("pathfinder.png"_spr), BaseType::Circle, 4, 3
        ).scale(0.6f);
        runBtn->setTopRelativeScale(1.35f);

        runBtn.intoMenuItem([this]() {
                if (!m_editorLayer)
                    return;
                std::string liveLevel = m_editorLayer->getLevelString().c_str();
                auto* level = m_editorLayer->m_level;
                Build<PathfinderNode>::create(levelNameOf(level), liveLevel)
                    .parent(this).zOrder(200);
            })
            .id("pathfinder-editor-run-button")
            .parent(menu)
            .move(-28, 0);

        Build<CCSprite>::createSpriteName("GJ_optionsBtn_001.png")
            .scale(0.6f)
            .intoMenuItem([](CCMenuItemSpriteExtra*) { openPathfinderSettings(); })
            .id("pathfinder-editor-settings-button")
            .parent(menu)
            .move(28, 0);
    }
};
