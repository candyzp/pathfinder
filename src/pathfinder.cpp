#include <set>
#include <cstdint>
#include <algorithm>
#include <Level.hpp>
#include <random>
#include <limits>
#include <gdr/gdr.hpp>
#include "pathfinder.hpp"

class Replay2 : public gdr::Replay<Replay2, gdr::Input<"">> {
public:
    Replay2() : Replay("Pathfinder", 1) {}
};

struct Level2 : public Level {
    bool press1 = false;
    bool press2 = false;
    float highestY = 0;
    using Level::Level;

    Level2(std::string const& lvlString) : Level(lvlString) {
        for (auto& section : sections) {
            for (auto& object : section)
                highestY = std::max(highestY, object->pos.y);
        }
    }

    void syncPresses() {
        press1 = latestState().button;
        press2 = latestState2().button;
    }
};

using SearchInput = uint32_t;

static SearchInput inputKey(uint32_t frame, bool player2) {
    return (frame << 1) | static_cast<uint32_t>(player2);
}

struct TrialResult {
    int frame = 0;
    float x = 0.f;
    bool dead = false;
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

static TrialResult tryInputs(Level2& lvl, std::set<SearchInput> const& inputs) {
    auto startFrame = lvl.currentFrame();
    auto press1Before = lvl.press1;
    auto press2Before = lvl.press2;

    // Every candidate gets the exact same simulation horizon. The old solver
    // stopped as soon as a candidate ran out of generated toggles, which
    // accidentally rewarded spammy candidates simply because they contained
    // more future inputs.
    constexpr int horizonFrames = 1000;
    int endFrame = startFrame + horizonFrames;

    while (lvl.currentFrame() < endFrame &&
           !lvl.latestState().dead &&
           lvl.latestState().pos.x < lvl.length) {
        uint32_t current = static_cast<uint32_t>(lvl.currentFrame());
        auto p1 = inputKey(current, false);
        auto p2 = inputKey(current, true);

        if (inputs.contains(p1))
            lvl.press1 = !lvl.press1;
        if (inputs.contains(p2))
            lvl.press2 = !lvl.press2;

        lvl.runFrame(lvl.press1, lvl.press2, 1.f / 240.f);
    }

    TrialResult result {
        lvl.currentFrame(),
        lvl.latestState().pos.x,
        lvl.latestState().dead
    };

    float lastY = lvl.latestState().pos.y;
    if (result.x < lvl.length &&
        (lastY > std::max(1300.f, lvl.highestY + 600.f) || lastY < -600.f)) {
        result.frame = startFrame;
        result.dead = true;
    }

    lvl.rollback(startFrame);
    lvl.press1 = press1Before;
    lvl.press2 = press2Before;
    return result;
}

PathfinderResult pathfind(
    std::string const& lvlString,
    std::atomic_bool& stop,
    std::function<void(double)> callback
) {
    Level2 lvl(lvlString);

    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_int_distribution<int> frameDist(0, 999);

    int trueBest = 0;
    int fail = 1;
    int numAway = 1000;
    int stagnantRounds = 0;
    int recoveryCount = 0;

    Level2 lvlBest = lvl;

    while (lvl.latestState().pos.x < lvl.length && !stop) {
        auto frame = lvl.currentFrame();

        std::set<SearchInput> bestInputs;
        int bestFrame = frame;
        size_t bestToggleCount = std::numeric_limits<size_t>::max();

        constexpr int iterations = 360;
        for (int i = 0; i < iterations; i++) {
            std::set<SearchInput> inputs;
            bool dual = lvl.latestState().dualActive;

            int maxP1 = maxToggleBudget(lvl.latestState().vehicle.type);
            int maxP2 = dual ? maxP1 : 4;
            std::uniform_int_distribution<int> p1Budget(0, maxP1);
            std::uniform_int_distribution<int> p2Budget(0, maxP2);

            int p1Candidates = p1Budget(rng);
            int p2Candidates = p2Budget(rng);

            for (int j = 0; j < p1Candidates; ++j) {
                uint32_t candidateFrame = static_cast<uint32_t>(frame + frameDist(rng));
                inputs.insert(inputKey(candidateFrame, false));
            }
            for (int j = 0; j < p2Candidates; ++j) {
                uint32_t candidateFrame = static_cast<uint32_t>(frame + frameDist(rng));
                inputs.insert(inputKey(candidateFrame, true));
            }

            auto trial = tryInputs(lvl, inputs);
            if (trial.frame > bestFrame ||
                (trial.frame == bestFrame && inputs.size() < bestToggleCount)) {
                bestFrame = trial.frame;
                bestToggleCount = inputs.size();
                bestInputs = std::move(inputs);

                // A full clean horizon is already excellent. Keep searching a
                // little for a simpler input set, but don't waste all 360 tries.
                if (bestFrame - frame >= 1000 && bestToggleCount <= 2 && i > 40)
                    break;
            }
        }

        if (bestFrame == frame) {
            lvl.rollback(std::max(std::max(frame - fail, trueBest - numAway), 1));
            lvl.syncPresses();

            fail += 5;
            if (fail > numAway + 1000) {
                numAway += 1000;
                fail = 1;

                if (numAway > 10000) {
                    numAway = 1000;
                    trueBest = 0;
                    lvl.rollback(1);
                    lvl.syncPresses();
                }
            } else if (fail > 100) {
                fail += 50;
            }
        } else {
            // Only commit the safer first part of the winning trial. The rest
            // remains look-ahead and will be re-evaluated from the new state.
            int applyUntil = bestFrame - static_cast<int>((bestFrame - frame) / 1.5);
            for (int i = frame; i < applyUntil && !lvl.latestState().dead; ++i) {
                auto p1 = inputKey(static_cast<uint32_t>(i), false);
                auto p2 = inputKey(static_cast<uint32_t>(i), true);
                if (bestInputs.contains(p1))
                    lvl.press1 = !lvl.press1;
                if (bestInputs.contains(p2))
                    lvl.press2 = !lvl.press2;

                lvl.runFrame(lvl.press1, lvl.press2, 1.f / 240.f);
                if (lvl.latestState().pos.x >= lvl.length)
                    break;
            }
        }

        if (lvl.currentFrame() > trueBest) {
            trueBest = lvl.currentFrame();
            fail = 0;
            numAway = 1000;
            stagnantRounds = 0;
        } else {
            ++stagnantRounds;
        }

        if (lvl.currentFrame() > lvlBest.currentFrame() && !lvl.latestState().dead)
            lvlBest = lvl;

        // Hard stall recovery. Instead of spending minutes oscillating around
        // the same 50-70% state, jump back to the best known path, retreat a
        // progressively larger amount, and reseed the search.
        if (stagnantRounds >= 12 && lvlBest.currentFrame() > 2) {
            lvl = lvlBest;
            int retreat = std::min(
                lvlBest.currentFrame() - 1,
                480 * (1 + std::min(recoveryCount, 10))
            );
            lvl.rollback(std::max(1, lvlBest.currentFrame() - retreat));
            lvl.syncPresses();

            ++recoveryCount;
            stagnantRounds = 0;
            fail = 1;
            numAway = std::min(6000, 1000 + recoveryCount * 500);
            rng.seed(rd() ^ static_cast<unsigned int>(lvl.currentFrame() + recoveryCount * 7919));
        }

        if (callback && lvl.length > 0.f) {
            callback(std::clamp(
                (static_cast<double>(lvlBest.latestState().pos.x) / lvl.length) * 100.0,
                0.0,
                100.0
            ));
        }
    }

    PathfinderResult result;
    if (lvl.currentFrame() > lvlBest.currentFrame() && !lvl.latestState().dead)
        lvlBest = lvl;

    Replay2 output;
    for (size_t i = 1; i < lvlBest.gameStates.size(); ++i) {
        auto const& p1 = lvlBest.gameStates[i];
        auto const& p1Prev = lvlBest.gameStates[i - 1];
        if (p1.frame > 1 && p1.button != p1Prev.button) {
            output.inputs.push_back(gdr::Input(p1.frame, 1, false, p1.button));
            result.inputs.push_back(PathfinderInput {
                static_cast<uint32_t>(p1.frame), false, p1.button
            });
        }

        if (i < lvlBest.gameStates2.size()) {
            auto const& p2 = lvlBest.gameStates2[i];
            auto const& p2Prev = lvlBest.gameStates2[i - 1];
            if (p2.dualActive && p2.frame > 1 && p2.button != p2Prev.button) {
                output.inputs.push_back(gdr::Input(p2.frame, 1, true, p2.button));
                result.inputs.push_back(PathfinderInput {
                    static_cast<uint32_t>(p2.frame), true, p2.button
                });
            }
        }
    }

    std::sort(result.inputs.begin(), result.inputs.end(), [](auto const& a, auto const& b) {
        if (a.frame != b.frame)
            return a.frame < b.frame;
        return a.player2 < b.player2;
    });

    result.macro = output.exportData().unwrapOr({});
    if (lvl.length > 0.f) {
        result.progress = std::clamp(
            (static_cast<double>(lvlBest.latestState().pos.x) / lvl.length) * 100.0,
            0.0,
            100.0
        );
    }
    result.complete = !lvlBest.latestState().dead && lvlBest.latestState().pos.x >= lvl.length;
    return result;
}
