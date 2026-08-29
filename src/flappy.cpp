#include <Geode/Geode.hpp>
#include <UIBuilder.hpp>

#include <algorithm>
#include <random>
#include <vector>

#include "flappy.hpp"

using namespace geode::prelude;

namespace {

class PathfinderFlappy : public CCLayerColor {
    struct PipePair {
        CCScale9Sprite* top = nullptr;
        CCScale9Sprite* bottom = nullptr;
        float x = 0.f;
        float gapY = 0.f;
        bool scored = false;
    };

    static constexpr float kWidth = 126.f;
    static constexpr float kHeight = 142.f;
    static constexpr float kGap = 46.f;
    static constexpr float kPipeWidth = 15.f;

    float m_left = 0.f;
    float m_bottom = 0.f;
    float m_birdX = 0.f;
    float m_velocity = 0.f;
    float m_spawnTimer = 0.f;
    bool m_running = false;
    int m_score = 0;

    CCScale9Sprite* m_bird = nullptr;
    CCLabelBMFont* m_scoreLabel = nullptr;
    CCLabelBMFont* m_hintLabel = nullptr;
    std::vector<PipePair> m_pipes;
    std::mt19937 m_rng {std::random_device{}()};

public:
    static PathfinderFlappy* create() {
        auto* layer = new PathfinderFlappy();
        if (layer && layer->init()) {
            layer->autorelease();
            return layer;
        }
        CC_SAFE_DELETE(layer);
        return nullptr;
    }

    bool init() override {
        if (!CCLayerColor::initWithColor({0, 0, 0, 0}))
            return false;

        setID("pathfinder-flappy-game");

        auto win = CCDirector::sharedDirector()->getWinSize();
        float centerX = std::min(win.width - 68.f, win.width / 2.f + 205.f);
        float centerY = win.height / 2.f;
        m_left = centerX - kWidth / 2.f;
        m_bottom = centerY - kHeight / 2.f;
        m_birdX = m_left + 28.f;

        auto* panel = CCScale9Sprite::create("GJ_square02.png");
        panel->setContentSize({kWidth, kHeight});
        panel->setPosition({centerX, centerY});
        addChild(panel, 0);

        m_bird = CCScale9Sprite::create("GJ_square01.png");
        m_bird->setContentSize({11.f, 11.f});
        m_bird->setPosition({m_birdX, m_bottom + kHeight * 0.5f});
        addChild(m_bird, 4);

        m_scoreLabel = CCLabelBMFont::create("0", "bigFont.fnt");
        m_scoreLabel->setScale(0.45f);
        m_scoreLabel->setPosition({centerX, m_bottom + kHeight - 13.f});
        addChild(m_scoreLabel, 6);

        m_hintLabel = CCLabelBMFont::create("FLAP to start", "chatFont.fnt");
        m_hintLabel->setScale(0.38f);
        m_hintLabel->setPosition({centerX, m_bottom + 17.f});
        addChild(m_hintLabel, 6);

        auto* menu = CCMenu::create();
        menu->setPosition({centerX, m_bottom - 18.f});
        addChild(menu, 8);

        Build<ButtonSprite>::create("FLAP", "bigFont.fnt", "GJ_button_01.png")
            .scale(0.46f)
            .intoMenuItem([this](CCMenuItemSpriteExtra*) {
                flap();
            })
            .move(-22.f, 0.f)
            .parent(menu);

        Build<CCSprite>::createSpriteName("GJ_closeBtn_001.png")
            .scale(0.55f)
            .intoMenuItem([this](CCMenuItemSpriteExtra*) {
                removeFromParentAndCleanup(true);
            })
            .move(42.f, 0.f)
            .parent(menu);

        schedule(schedule_selector(PathfinderFlappy::tick));
        return true;
    }

private:
    void clearPipes() {
        for (auto const& pipe : m_pipes) {
            if (pipe.top)
                pipe.top->removeFromParentAndCleanup(true);
            if (pipe.bottom)
                pipe.bottom->removeFromParentAndCleanup(true);
        }
        m_pipes.clear();
    }

    void resetGame() {
        clearPipes();
        m_velocity = 0.f;
        m_spawnTimer = 0.f;
        m_score = 0;
        m_running = false;
        m_bird->setPosition({m_birdX, m_bottom + kHeight * 0.5f});
        m_scoreLabel->setString("0");
        m_hintLabel->setString("FLAP to start");
    }

    void flap() {
        if (!m_running) {
            clearPipes();
            m_score = 0;
            m_scoreLabel->setString("0");
            m_bird->setPosition({m_birdX, m_bottom + kHeight * 0.5f});
            m_spawnTimer = 0.65f;
            m_running = true;
            m_hintLabel->setString("");
        }
        m_velocity = 112.f;
    }

    void spawnPipe() {
        std::uniform_real_distribution<float> gapDist(
            m_bottom + 39.f,
            m_bottom + kHeight - 39.f
        );

        PipePair pipe;
        pipe.x = m_left + kWidth + 8.f;
        pipe.gapY = gapDist(m_rng);

        float bottomTop = pipe.gapY - kGap * 0.5f;
        float bottomHeight = std::max(1.f, bottomTop - m_bottom);
        float topBottom = pipe.gapY + kGap * 0.5f;
        float topHeight = std::max(1.f, m_bottom + kHeight - topBottom);

        pipe.bottom = CCScale9Sprite::create("GJ_square01.png");
        pipe.bottom->setContentSize({kPipeWidth, bottomHeight});
        pipe.bottom->setPosition({pipe.x, m_bottom + bottomHeight * 0.5f});
        addChild(pipe.bottom, 2);

        pipe.top = CCScale9Sprite::create("GJ_square01.png");
        pipe.top->setContentSize({kPipeWidth, topHeight});
        pipe.top->setPosition({pipe.x, topBottom + topHeight * 0.5f});
        addChild(pipe.top, 2);

        m_pipes.push_back(pipe);
    }

    bool collided(PipePair const& pipe) const {
        float birdY = m_bird->getPositionY();
        float birdHalf = 5.5f;
        float pipeHalf = kPipeWidth * 0.5f;
        bool xOverlap =
            m_birdX + birdHalf >= pipe.x - pipeHalf &&
            m_birdX - birdHalf <= pipe.x + pipeHalf;
        if (!xOverlap)
            return false;

        float gapBottom = pipe.gapY - kGap * 0.5f;
        float gapTop = pipe.gapY + kGap * 0.5f;
        return birdY - birdHalf < gapBottom || birdY + birdHalf > gapTop;
    }

    void lose() {
        m_running = false;
        m_velocity = 0.f;
        m_hintLabel->setString("FLAP to retry");
    }

    void tick(float dt) {
        if (!m_running)
            return;

        dt = std::min(dt, 1.f / 20.f);
        m_velocity -= 255.f * dt;
        m_bird->setPositionY(m_bird->getPositionY() + m_velocity * dt);

        if (m_bird->getPositionY() < m_bottom + 7.f ||
            m_bird->getPositionY() > m_bottom + kHeight - 7.f) {
            lose();
            return;
        }

        m_spawnTimer -= dt;
        if (m_spawnTimer <= 0.f) {
            spawnPipe();
            m_spawnTimer = 1.45f;
        }

        for (auto& pipe : m_pipes) {
            pipe.x -= 52.f * dt;
            if (pipe.top)
                pipe.top->setPositionX(pipe.x);
            if (pipe.bottom)
                pipe.bottom->setPositionX(pipe.x);

            if (collided(pipe)) {
                lose();
                return;
            }

            if (!pipe.scored && pipe.x < m_birdX) {
                pipe.scored = true;
                ++m_score;
                m_scoreLabel->setString(std::to_string(m_score).c_str());
            }
        }

        while (!m_pipes.empty() && m_pipes.front().x < m_left - 18.f) {
            if (m_pipes.front().top)
                m_pipes.front().top->removeFromParentAndCleanup(true);
            if (m_pipes.front().bottom)
                m_pipes.front().bottom->removeFromParentAndCleanup(true);
            m_pipes.erase(m_pipes.begin());
        }
    }
};

} // namespace

void togglePathfinderFlappy(CCNode* parent) {
    if (!parent)
        return;

    if (auto* existing = parent->getChildByID("pathfinder-flappy-game")) {
        existing->removeFromParentAndCleanup(true);
        return;
    }

    if (auto* game = PathfinderFlappy::create())
        parent->addChild(game, 250);
}
