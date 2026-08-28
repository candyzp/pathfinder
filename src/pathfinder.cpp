#include <set>
#include <cstdint>
#include <algorithm>
#include <Level.hpp>
#include <random>
#include <limits>
#include <vector>
#include <iterator>
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
using InputSet = std::set<SearchInput>;

static SearchInput inputKey(uint32_t frame, bool player2) {
    return (frame << 1) | static_cast<uint32_t>(player2);
}

static uint32_t inputFrame(SearchInput input) {
    return input >> 1;
}

static bool inputPlayer2(SearchInput input) {
    return (input & 1u) != 0;
}

struct TrialResult {
    int frame = 0;
    float x = 0.f;
    bool dead = false;
    bool complete = false;
};

struct CandidateResult {
    InputSet inputs;
    TrialResult trial;
};

static bool betterCandidate(CandidateResult const& a, CandidateResult const& b) {
    if (a.trial.complete != b.trial.complete)
        return a.trial.complete;
    if (a.trial.x != b.trial.x)
        return a.trial.x > b.trial.x;
    if (a.trial.dead != b.trial.dead)
        return !a.trial.dead;
    if (a.inputs.size() != b.inputs.size())
        return a.inputs.size() < b.inputs.size();
    return a.trial.frame > b.trial.frame;
}

static int maxToggleBudget(VehicleType type) {
    switch (type) {
        case VehicleType::Cube:   return 8;
        case VehicleType::Ship:   return 16;
        case VehicleType::Ball:   return 8;
        case VehicleType::Ufo:    return 12;
        case VehicleType::Wave:   return 20;
        case VehicleType::Robot:  return 10;
        case VehicleType::Spider: return 8;
        case VehicleType::Swing:  return 16;
    }
    return 10;
}

static std::vector<int> usefulHoldDurations(VehicleType type) {
    switch (type) {
        case VehicleType::Cube:   return {2, 6, 12, 24};
        case VehicleType::Ship:   return {12, 24, 48, 96, 180, 300};
        case VehicleType::Ball:   return {2, 6, 12};
        case VehicleType::Ufo:    return {2, 6, 12, 24};
        case VehicleType::Wave:   return {8, 16, 32, 64, 120, 220};
        case VehicleType::Robot:  return {8, 20, 40, 70, 110};
        case VehicleType::Spider: return {2, 6, 12};
        case VehicleType::Swing:  return {8, 16, 32, 64, 120};
    }
    return {4, 12, 24};
}

static void addToggle(InputSet& inputs, int startFrame, int horizon, int offset, bool player2) {
    if (offset < 0 || offset >= horizon)
        return;
    inputs.insert(inputKey(static_cast<uint32_t>(startFrame + offset), player2));
}

static void addPulse(InputSet& inputs, int startFrame, int horizon, int offset, int duration, bool player2) {
    addToggle(inputs, startFrame, horizon, offset, player2);
    addToggle(inputs, startFrame, horizon, offset + std::max(1, duration), player2);
}

static TrialResult tryInputs(Level2& lvl, InputSet const& inputs, int horizonFrames) {
    auto startFrame = lvl.currentFrame();
    auto press1Before = lvl.press1;
    auto press2Before = lvl.press2;
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
        lvl.latestState().dead,
        !lvl.latestState().dead && lvl.latestState().pos.x >= lvl.length
    };

    float lastY = lvl.latestState().pos.y;
    if (!result.complete && result.x < lvl.length &&
        (lastY > std::max(1300.f, lvl.highestY + 600.f) || lastY < -600.f)) {
        result.frame = startFrame;
        result.dead = true;
    }

    lvl.rollback(startFrame);
    lvl.press1 = press1Before;
    lvl.press2 = press2Before;
    return result;
}

static std::vector<InputSet> makeStructuredSeeds(
    Level2 const& lvl,
    int startFrame,
    int horizonFrames,
    bool player2
) {
    std::vector<InputSet> seeds;
    auto const type = player2 ? lvl.latestState2().vehicle.type : lvl.latestState().vehicle.type;
    auto const durations = usefulHoldDurations(type);

    // Always test doing absolutely nothing. On safe stretches this should beat
    // needless clicking and helps Pathfinder naturally ignore fake/decorative orbs.
    seeds.emplace_back();

    int denseWindow = std::min(horizonFrames, 960);
    int coarseWindow = std::min(horizonFrames, 1500);

    // Dense timing sweep near the player. Hard jump windows usually live here.
    for (int offset = 0; offset < denseWindow; offset += 12) {
        for (int duration : durations) {
            InputSet input;
            addPulse(input, startFrame, horizonFrames, offset, duration, player2);
            seeds.push_back(std::move(input));
        }
    }

    // Farther look-ahead gets a coarser sweep so we can discover setups for an
    // obstacle before it is already on top of the player.
    for (int offset = 960; offset < coarseWindow; offset += 36) {
        for (size_t i = 0; i < durations.size(); i += 2) {
            InputSet input;
            addPulse(input, startFrame, horizonFrames, offset, durations[i], player2);
            seeds.push_back(std::move(input));
        }
    }

    return seeds;
}

static SearchInput nthInput(InputSet const& inputs, size_t index) {
    auto it = inputs.begin();
    std::advance(it, static_cast<long>(index));
    return *it;
}

static InputSet mutateCandidate(
    InputSet base,
    Level2 const& lvl,
    int startFrame,
    int horizonFrames,
    std::mt19937& rng
) {
    std::uniform_int_distribution<int> mutationDist(0, 4);
    std::uniform_int_distribution<int> offsetDist(0, std::max(0, horizonFrames - 1));
    std::uniform_int_distribution<int> shiftDist(-48, 48);
    bool dual = lvl.latestState().dualActive;

    int mutations = 1 + static_cast<int>(rng() % 3);
    for (int m = 0; m < mutations; ++m) {
        int kind = mutationDist(rng);

        if (kind == 0 || base.empty()) {
            bool player2 = dual && ((rng() & 3u) == 0u);
            auto type = player2 ? lvl.latestState2().vehicle.type : lvl.latestState().vehicle.type;
            auto durations = usefulHoldDurations(type);
            int offset = offsetDist(rng);
            int duration = durations[rng() % durations.size()];
            addPulse(base, startFrame, horizonFrames, offset, duration, player2);
        } else if (kind == 1 && !base.empty()) {
            size_t index = static_cast<size_t>(rng() % base.size());
            auto value = nthInput(base, index);
            base.erase(value);
        } else if (kind == 2 && !base.empty()) {
            size_t index = static_cast<size_t>(rng() % base.size());
            auto value = nthInput(base, index);
            bool player2 = inputPlayer2(value);
            int oldOffset = static_cast<int>(inputFrame(value)) - startFrame;
            int newOffset = std::clamp(oldOffset + shiftDist(rng), 0, horizonFrames - 1);
            base.erase(value);
            addToggle(base, startFrame, horizonFrames, newOffset, player2);
        } else if (kind == 3) {
            bool player2 = dual && ((rng() & 1u) != 0u);
            addToggle(base, startFrame, horizonFrames, offsetDist(rng), player2);
        } else if (kind == 4 && base.size() >= 2) {
            // Remove a nearby toggle pair. This is a direct simplification move
            // and lets refinement discover cleaner routes from noisy parents.
            size_t index = static_cast<size_t>(rng() % base.size());
            auto first = nthInput(base, index);
            auto next = base.upper_bound(first);
            base.erase(first);
            if (next != base.end())
                base.erase(next);
        }
    }

    return base;
}

static CandidateResult searchBestInputs(
    Level2& lvl,
    std::atomic_bool& stop,
    std::mt19937& rng,
    int horizonFrames,
    int refinementStrength
) {
    int frame = lvl.currentFrame();
    bool dual = lvl.latestState().dualActive;

    std::vector<InputSet> seeds = makeStructuredSeeds(lvl, frame, horizonFrames, false);
    if (dual) {
        auto p2Seeds = makeStructuredSeeds(lvl, frame, horizonFrames, true);
        size_t keep = std::min<size_t>(p2Seeds.size(), 80);
        for (size_t i = 1; i < keep; ++i)
            seeds.push_back(std::move(p2Seeds[i]));
    }

    // Add broad random candidates after the structured sweep. Randomness is
    // still useful for unusual setups, but it is no longer the entire brain.
    int randomCandidates = 180 + refinementStrength * 80;
    std::uniform_int_distribution<int> frameDist(0, std::max(0, horizonFrames - 1));
    int maxP1 = maxToggleBudget(lvl.latestState().vehicle.type);
    int maxP2 = dual ? maxToggleBudget(lvl.latestState2().vehicle.type) : 2;

    for (int i = 0; i < randomCandidates; ++i) {
        InputSet inputs;
        int p1Count = static_cast<int>(rng() % static_cast<unsigned>(maxP1 + 1));
        int p2Count = dual ? static_cast<int>(rng() % static_cast<unsigned>(maxP2 + 1)) : 0;

        for (int j = 0; j < p1Count; ++j)
            addToggle(inputs, frame, horizonFrames, frameDist(rng), false);
        for (int j = 0; j < p2Count; ++j)
            addToggle(inputs, frame, horizonFrames, frameDist(rng), true);

        seeds.push_back(std::move(inputs));
    }

    std::vector<CandidateResult> elites;
    constexpr size_t eliteCount = 18;

    auto consider = [&](InputSet inputs) {
        if (stop)
            return;

        CandidateResult candidate;
        candidate.trial = tryInputs(lvl, inputs, horizonFrames);
        candidate.inputs = std::move(inputs);

        elites.push_back(std::move(candidate));
        std::sort(elites.begin(), elites.end(), betterCandidate);
        if (elites.size() > eliteCount)
            elites.resize(eliteCount);
    };

    for (auto& seed : seeds) {
        if (stop)
            break;
        consider(std::move(seed));
        if (!elites.empty() && elites.front().trial.complete && elites.front().inputs.size() <= 2)
            break;
    }

    // Evolutionary refinement: repeatedly mutate the best routes instead of
    // throwing them away and starting random from scratch every round.
    int rounds = 3 + refinementStrength;
    int childrenPerRound = 150 + refinementStrength * 60;
    for (int round = 0; round < rounds && !stop && !elites.empty(); ++round) {
        auto parents = elites;
        for (int child = 0; child < childrenPerRound && !stop; ++child) {
            size_t parentIndex = static_cast<size_t>(rng() % std::min<size_t>(parents.size(), 10));
            auto mutated = mutateCandidate(
                parents[parentIndex].inputs,
                lvl,
                frame,
                horizonFrames,
                rng
            );
            consider(std::move(mutated));
        }

        if (!elites.empty() && elites.front().trial.complete && elites.front().inputs.size() <= 2)
            break;
    }

    if (elites.empty()) {
        CandidateResult empty;
        empty.trial = {frame, lvl.latestState().pos.x, lvl.latestState().dead, false};
        return empty;
    }

    return elites.front();
}

PathfinderResult pathfind(
    std::string const& lvlString,
    std::atomic_bool& stop,
    std::function<void(double)> callback
) {
    Level2 lvl(lvlString);

    std::random_device rd;
    std::mt19937 rng(rd());

    int trueBest = 0;
    int fail = 1;
    int numAway = 1000;
    int stagnantRounds = 0;
    int recoveryCount = 0;
    float furthestX = lvl.latestState().pos.x;

    Level2 lvlBest = lvl;

    while (lvl.latestState().pos.x < lvl.length && !stop) {
        auto frame = lvl.currentFrame();

        // Spend more time thinking when progress is difficult. Normal sections
        // use a 6.25 second look-ahead; repeated stalls ramp that toward 10 sec
        // and also add extra refinement rounds/candidates.
        int difficulty = std::min(4, recoveryCount + stagnantRounds / 4);
        int horizonFrames = 1500 + difficulty * 225;
        auto best = searchBestInputs(lvl, stop, rng, horizonFrames, difficulty);

        if (stop)
            break;

        int bestFrame = best.trial.frame;
        bool bestComplete = best.trial.complete;

        if (bestFrame == frame || best.trial.x <= lvl.latestState().pos.x) {
            lvl.rollback(std::max(std::max(frame - fail, trueBest - numAway), 1));
            lvl.syncPresses();

            fail += 5;
            if (fail > numAway + 1000) {
                numAway += 1000;
                fail = 1;

                if (numAway > 12000) {
                    numAway = 1500;
                    trueBest = 0;
                    lvl.rollback(1);
                    lvl.syncPresses();
                }
            } else if (fail > 100) {
                fail += 50;
            }
        } else {
            // Keep more of a route only when it actually reaches the end.
            // Otherwise commit roughly the first half so the solver repeatedly
            // re-checks the future instead of blindly trusting a long forecast.
            int applyUntil = bestComplete
                ? bestFrame
                : frame + std::max(1, (bestFrame - frame) / 2);

            for (int i = frame; i < applyUntil && !lvl.latestState().dead; ++i) {
                auto p1 = inputKey(static_cast<uint32_t>(i), false);
                auto p2 = inputKey(static_cast<uint32_t>(i), true);
                if (best.inputs.contains(p1))
                    lvl.press1 = !lvl.press1;
                if (best.inputs.contains(p2))
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
        }

        if (!lvl.latestState().dead && lvl.latestState().pos.x > furthestX + 1.f) {
            furthestX = lvl.latestState().pos.x;
            lvlBest = lvl;
            stagnantRounds = 0;
            // Keep a little memory of recent difficulty rather than dropping all
            // the way back to shallow search after moving forward by one pixel.
            recoveryCount = std::max(0, recoveryCount - 1);
        } else {
            ++stagnantRounds;
        }

        if (stagnantRounds >= 8 && lvlBest.currentFrame() > 2) {
            lvl = lvlBest;
            int retreat = std::min(
                lvlBest.currentFrame() - 1,
                720 * (1 + std::min(recoveryCount, 10))
            );
            lvl.rollback(std::max(1, lvlBest.currentFrame() - retreat));
            lvl.syncPresses();

            ++recoveryCount;
            stagnantRounds = 0;
            fail = 1;
            numAway = std::min(9000, 1500 + recoveryCount * 750);
            rng.seed(rd() ^ static_cast<unsigned int>(lvl.currentFrame() + recoveryCount * 7919));
        }

        if (callback && lvl.length > 0.f) {
            callback(std::clamp(
                (static_cast<double>(furthestX) / lvl.length) * 100.0,
                0.0,
                100.0
            ));
        }
    }

    PathfinderResult result;
    if (!lvl.latestState().dead && lvl.latestState().pos.x > furthestX) {
        furthestX = lvl.latestState().pos.x;
        lvlBest = lvl;
    }

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
            (static_cast<double>(furthestX) / lvl.length) * 100.0,
            0.0,
            100.0
        );
    }
    result.complete = !lvlBest.latestState().dead && lvlBest.latestState().pos.x >= lvl.length;
    return result;
}
