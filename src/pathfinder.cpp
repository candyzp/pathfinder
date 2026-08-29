#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <Level.hpp>
#include <gdr/gdr.hpp>

#include "pathfinder.hpp"

class Replay2 : public gdr::Replay<Replay2, gdr::Input<"">> {
public:
    Replay2() : Replay("Pathfinder", 1) {}
};

struct Level2 : public Level {
    bool press1 = false;
    bool press2 = false;
    float highestY = 0.f;

    using Level::Level;

    explicit Level2(std::string const& lvlString) : Level(lvlString) {
        for (auto const& [_, section] : sections) {
            for (auto const& object : section)
                highestY = std::max(highestY, object->pos.y);
        }
    }

    void syncPresses() {
        press1 = latestState().button;
        press2 = latestState2().button;
    }

    void fixStatePointers() {
        for (auto& state : gameStates)
            state.level = this;
        for (auto& state : gameStates2)
            state.level = this;
    }
};

using SearchInput = uint32_t;

static SearchInput inputKey(uint32_t frame, bool player2) {
    return (frame << 1) | static_cast<uint32_t>(player2);
}

static bool reachedGoal(Level2 const& lvl) {
    return !lvl.gameStates.empty() && lvl.gameStates.back().completed;
}

struct TrialResult {
    int frame = 0;
    float x = 0.f;
    bool dead = false;
    bool complete = false;
};

struct Timeline {
    std::vector<Player> p1;
    std::vector<Player> p2;
    bool press1 = false;
    bool press2 = false;

    explicit Timeline(Level2 const& lvl) {
        capture(lvl);
    }

    void capture(Level2 const& lvl) {
        p1 = lvl.gameStates;
        p2 = lvl.gameStates2;
        press1 = lvl.press1;
        press2 = lvl.press2;
    }

    void restore(Level2& lvl) const {
        lvl.gameStates = p1;
        lvl.gameStates2 = p2;
        lvl.press1 = press1;
        lvl.press2 = press2;
        lvl.fixStatePointers();
    }

    int frame() const {
        return p1.empty() ? 0 : p1.back().frame;
    }

    float x() const {
        return p1.empty() ? 0.f : p1.back().pos.x;
    }

    bool complete() const {
        return !p1.empty() && !p1.back().dead && p1.back().completed;
    }
};

static int maxToggleBudget(VehicleType type) {
    switch (type) {
        case VehicleType::Cube:   return 10;
        case VehicleType::Ship:   return 18;
        case VehicleType::Ball:   return 10;
        case VehicleType::Ufo:    return 14;
        case VehicleType::Wave:   return 24;
        case VehicleType::Robot:  return 12;
        case VehicleType::Spider: return 10;
        case VehicleType::Swing:  return 20;
    }
    return 12;
}

static TrialResult tryInputs(
    Level2& lvl,
    std::set<SearchInput> const& inputs,
    int horizonFrames
) {
    int startFrame = lvl.currentFrame();
    bool press1Before = lvl.press1;
    bool press2Before = lvl.press2;
    int endFrame = startFrame + horizonFrames;

    while (lvl.currentFrame() < endFrame &&
           !lvl.latestState().dead &&
           !reachedGoal(lvl)) {
        uint32_t current = static_cast<uint32_t>(lvl.currentFrame());

        if (inputs.contains(inputKey(current, false)))
            lvl.press1 = !lvl.press1;
        if (inputs.contains(inputKey(current, true)))
            lvl.press2 = !lvl.press2;

        lvl.runFrame(lvl.press1, lvl.press2, 1.f / 240.f);
    }

    TrialResult result {
        lvl.currentFrame(),
        lvl.latestState().pos.x,
        lvl.latestState().dead,
        !lvl.latestState().dead && reachedGoal(lvl)
    };

    float lastY = lvl.latestState().pos.y;
    if (!result.complete &&
        (lastY > std::max(1300.f, lvl.highestY + 600.f) || lastY < -600.f)) {
        result.frame = startFrame;
        result.dead = true;
    }

    lvl.rollback(startFrame);
    lvl.press1 = press1Before;
    lvl.press2 = press2Before;
    return result;
}

static double progressFor(float x, float startX, float endX, bool complete) {
    if (complete)
        return 100.0;

    double span = static_cast<double>(endX - startX);
    if (span <= 1.0)
        return 0.0;

    return std::clamp(
        ((static_cast<double>(x) - startX) / span) * 100.0,
        0.0,
        99.99
    );
}

PathfinderResult pathfind(
    std::string const& lvlString,
    std::atomic_bool& stop,
    std::function<void(PathfinderTelemetry const&)> callback,
    float trustedEndX
) {
    Level2 lvl(lvlString);

    float solveStartX = lvl.latestState().pos.x;
    float simulatorLength = lvl.length;
    bool hasTrustedEnd = std::isfinite(trustedEndX) &&
                         trustedEndX > solveStartX + 30.f;

    if (hasTrustedEnd) {
        lvl.length = trustedEndX;
        lvl.lengthSource = "trusted-gd";
    }

    std::random_device rd;
    uint32_t baseSeed = rd() ^
        static_cast<uint32_t>(std::hash<std::string>{}(lvlString));
    std::mt19937 rng(baseSeed);

    int trueBestFrame = lvl.currentFrame();
    int fail = 1;
    int numAway = 1000;
    int stagnantRounds = 0;
    int recoveryCount = 0;

    float furthestX = lvl.latestState().pos.x;
    Timeline routeBest(lvl);
    Timeline progressBest(lvl);

    double lastReportedProgress = -1.0;

    auto emitProgress = [&](char const* reason) {
        if (!callback)
            return;

        double progress = progressFor(
            furthestX,
            solveStartX,
            lvl.length,
            reachedGoal(lvl)
        );

        if (!reachedGoal(lvl) &&
            progress < lastReportedProgress + 0.10) {
            return;
        }

        lastReportedProgress = progress;

        PathfinderTelemetry telemetry;
        telemetry.progress = progress;
        telemetry.startX = solveStartX;
        telemetry.currentX = lvl.latestState().pos.x;
        telemetry.furthestX = furthestX;
        telemetry.trustedEndX = hasTrustedEnd ? trustedEndX : 0.f;
        telemetry.inferredLength = simulatorLength;
        telemetry.frame = lvl.currentFrame();
        telemetry.checkpointFrame = routeBest.frame();
        telemetry.checkpointX = routeBest.x();
        telemetry.deathX = lvl.latestState().dead ? lvl.latestState().pos.x : 0.f;
        telemetry.mode = "classic-search";
        telemetry.recoveryReason = reason;
        callback(telemetry);
    };

    while (!reachedGoal(lvl) && !stop.load()) {
        if (lvl.latestState().dead) {
            progressBest.restore(lvl);
            int retreat = std::min(
                std::max(0, lvl.currentFrame() - 1),
                240 + recoveryCount * 240
            );
            lvl.rollback(std::max(1, lvl.currentFrame() - retreat));
            lvl.syncPresses();
            ++recoveryCount;
            rng.seed(baseSeed ^
                     static_cast<uint32_t>(recoveryCount * 7919) ^
                     static_cast<uint32_t>(lvl.currentFrame()));
            continue;
        }

        int frame = lvl.currentFrame();
        VehicleType mode = lvl.latestState().vehicle.type;

        int horizonFrames = 1000;
        if (mode == VehicleType::Ship ||
            mode == VehicleType::Wave ||
            mode == VehicleType::Swing) {
            horizonFrames = 1200;
        }

        std::uniform_int_distribution<int> frameDist(0, horizonFrames - 1);

        std::set<SearchInput> bestInputs;
        int bestFrame = frame;
        float bestX = lvl.latestState().pos.x;
        bool bestComplete = false;
        size_t bestToggleCount = std::numeric_limits<size_t>::max();

        constexpr int iterations = 300;
        for (int attempt = 0; attempt < iterations && !stop.load(); ++attempt) {
            std::set<SearchInput> inputs;
            bool dual = lvl.latestState().dualActive;

            int maxP1 = maxToggleBudget(mode);
            int maxP2 = dual
                ? maxToggleBudget(lvl.latestState2().vehicle.type)
                : 0;

            std::uniform_int_distribution<int> p1Budget(0, maxP1);
            std::uniform_int_distribution<int> p2Budget(0, maxP2);

            int p1Candidates = p1Budget(rng);
            int p2Candidates = p2Budget(rng);

            for (int i = 0; i < p1Candidates; ++i) {
                inputs.insert(inputKey(
                    static_cast<uint32_t>(frame + frameDist(rng)),
                    false
                ));
            }

            for (int i = 0; i < p2Candidates; ++i) {
                inputs.insert(inputKey(
                    static_cast<uint32_t>(frame + frameDist(rng)),
                    true
                ));
            }

            auto trial = tryInputs(lvl, inputs, horizonFrames);

            bool better = false;
            if (trial.complete != bestComplete) {
                better = trial.complete;
            } else if (trial.frame != bestFrame) {
                better = trial.frame > bestFrame;
            } else if (trial.x != bestX) {
                better = trial.x > bestX;
            } else {
                better = inputs.size() < bestToggleCount;
            }

            if (!better)
                continue;

            bestFrame = trial.frame;
            bestX = trial.x;
            bestComplete = trial.complete;
            bestToggleCount = inputs.size();
            bestInputs = std::move(inputs);

            if (bestComplete && bestToggleCount <= 2)
                break;

            if (bestFrame - frame >= horizonFrames &&
                bestToggleCount <= 2 &&
                attempt > 40) {
                break;
            }
        }

        if (stop.load())
            break;

        if (bestFrame == frame) {
            int preferred = std::max(
                frame - fail,
                trueBestFrame - numAway
            );
            int target = std::clamp(
                preferred,
                1,
                std::max(1, frame - 1)
            );

            lvl.rollback(target);
            lvl.syncPresses();

            fail += 5;
            if (fail > numAway + 1000) {
                numAway += 1000;
                fail = 1;

                if (numAway > 10000) {
                    numAway = 1000;
                    trueBestFrame = 1;
                    lvl.rollback(1);
                    lvl.syncPresses();
                    ++recoveryCount;
                    rng.seed(baseSeed ^
                             static_cast<uint32_t>(recoveryCount * 104729));
                }
            } else if (fail > 100) {
                fail += 50;
            }

            continue;
        }

        int applyUntil = bestComplete
            ? bestFrame
            : bestFrame - static_cast<int>((bestFrame - frame) / 1.5);

        while (lvl.currentFrame() < applyUntil &&
               !lvl.latestState().dead &&
               !reachedGoal(lvl)) {
            uint32_t current = static_cast<uint32_t>(lvl.currentFrame());

            if (bestInputs.contains(inputKey(current, false)))
                lvl.press1 = !lvl.press1;
            if (bestInputs.contains(inputKey(current, true)))
                lvl.press2 = !lvl.press2;

            lvl.runFrame(lvl.press1, lvl.press2, 1.f / 240.f);
        }

        if (lvl.latestState().dead)
            continue;

        if (lvl.currentFrame() > trueBestFrame) {
            trueBestFrame = lvl.currentFrame();
            fail = 0;
            numAway = 1000;
        }

        if (lvl.currentFrame() > routeBest.frame() || reachedGoal(lvl))
            routeBest.capture(lvl);

        bool advancedX = lvl.latestState().pos.x > furthestX + 1.f;
        if (advancedX) {
            furthestX = lvl.latestState().pos.x;
            progressBest.capture(lvl);
            stagnantRounds = 0;
            recoveryCount = std::max(0, recoveryCount - 1);
            emitProgress("advance");
        } else if (lvl.latestState().direction < 0) {
            stagnantRounds = 0;
        } else {
            ++stagnantRounds;
        }

        if (stagnantRounds >= 12 &&
            progressBest.frame() > 2 &&
            lvl.latestState().direction >= 0) {
            progressBest.restore(lvl);

            int retreat = std::min(
                progressBest.frame() - 1,
                480 * (1 + std::min(recoveryCount, 10))
            );

            lvl.rollback(std::max(
                1,
                progressBest.frame() - retreat
            ));
            lvl.syncPresses();

            ++recoveryCount;
            stagnantRounds = 0;
            fail = 1;
            numAway = std::min(
                6000,
                1000 + recoveryCount * 500
            );

            rng.seed(baseSeed ^
                     static_cast<uint32_t>(recoveryCount * 7919) ^
                     static_cast<uint32_t>(lvl.currentFrame()));
        }
    }

    if (!lvl.latestState().dead) {
        if (reachedGoal(lvl)) {
            routeBest.capture(lvl);
            progressBest.capture(lvl);
            furthestX = std::max(furthestX, lvl.latestState().pos.x);
            emitProgress("complete");
        } else if (lvl.currentFrame() > routeBest.frame()) {
            routeBest.capture(lvl);
        }
    }

    PathfinderResult result;
    Replay2 output;

    for (size_t i = 1; i < routeBest.p1.size(); ++i) {
        auto const& p1 = routeBest.p1[i];
        auto const& previousP1 = routeBest.p1[i - 1];

        if (p1.frame > 1 && p1.button != previousP1.button) {
            output.inputs.push_back(
                gdr::Input(p1.frame, 1, false, p1.button)
            );
            result.inputs.push_back({
                static_cast<uint32_t>(p1.frame),
                false,
                p1.button,
                1
            });
        }

        if (i < routeBest.p2.size()) {
            auto const& p2 = routeBest.p2[i];
            auto const& previousP2 = routeBest.p2[i - 1];

            if (p2.dualActive &&
                p2.frame > 1 &&
                p2.button != previousP2.button) {
                output.inputs.push_back(
                    gdr::Input(p2.frame, 1, true, p2.button)
                );
                result.inputs.push_back({
                    static_cast<uint32_t>(p2.frame),
                    true,
                    p2.button,
                    1
                });
            }
        }
    }

    std::sort(
        result.inputs.begin(),
        result.inputs.end(),
        [](auto const& a, auto const& b) {
            if (a.frame != b.frame)
                return a.frame < b.frame;
            return a.player2 < b.player2;
        }
    );

    result.macro = output.exportData().unwrapOr({});
    result.complete = routeBest.complete();
    result.progress = progressFor(
        furthestX,
        solveStartX,
        lvl.length,
        result.complete
    );

    std::ostringstream diagnostics;
    diagnostics
        << "solver=classic"
        << " progress=" << result.progress
        << " frame=" << routeBest.frame()
        << " furthestX=" << furthestX
        << " endX=" << lvl.length
        << " recoveryCount=" << recoveryCount
        << " inputs=" << result.inputs.size()
        << " complete=" << (result.complete ? 1 : 0);
    result.diagnostics = diagnostics.str();

    return result;
}
