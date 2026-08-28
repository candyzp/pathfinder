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

static bool continuousMode(VehicleType type) {
    return type == VehicleType::Ship ||
           type == VehicleType::Wave ||
           type == VehicleType::Swing;
}

static bool betterCandidate(CandidateResult const& a, CandidateResult const& b) {
    if (a.trial.complete != b.trial.complete)
        return a.trial.complete;

    // Tiny X differences are usually noise. If two paths are effectively tied,
    // prefer the one that stays alive and uses fewer toggles instead of buying
    // a couple pixels of progress with messy inputs.
    constexpr float progressTie = 6.f;
    if (a.trial.x > b.trial.x + progressTie)
        return true;
    if (b.trial.x > a.trial.x + progressTie)
        return false;

    if (a.trial.dead != b.trial.dead)
        return !a.trial.dead;
    if (a.inputs.size() != b.inputs.size())
        return a.inputs.size() < b.inputs.size();
    if (a.trial.x != b.trial.x)
        return a.trial.x > b.trial.x;
    return a.trial.frame > b.trial.frame;
}

static int maxToggleBudget(VehicleType type) {
    switch (type) {
        case VehicleType::Cube:   return 8;
        case VehicleType::Ship:   return 18;
        case VehicleType::Ball:   return 8;
        case VehicleType::Ufo:    return 10;
        case VehicleType::Wave:   return 22;
        case VehicleType::Robot:  return 10;
        case VehicleType::Spider: return 8;
        case VehicleType::Swing:  return 18;
    }
    return 10;
}

static std::vector<int> usefulHoldDurations(VehicleType type) {
    switch (type) {
        // Tap modes deliberately use short holds. Long cube/UFO holds can create
        // accidental repeated jumps after landing, which was one source of the
        // newer solver looking more spammy than the locked build.
        case VehicleType::Cube:   return {1, 2, 4, 6};
        case VehicleType::Ball:   return {1, 2, 4};
        case VehicleType::Ufo:    return {1, 2, 4, 6};
        case VehicleType::Spider: return {1, 2, 4};
        case VehicleType::Robot:  return {4, 8, 14, 24, 40, 64, 96};
        case VehicleType::Ship:   return {6, 12, 24, 40, 64, 96, 144, 220};
        case VehicleType::Wave:   return {4, 8, 16, 28, 44, 64, 96, 144};
        case VehicleType::Swing:  return {4, 8, 16, 28, 44, 64, 96, 144};
    }
    return {2, 4, 8};
}

static int denseTimingStep(VehicleType type, int strength) {
    if (continuousMode(type))
        return strength >= 2 ? 4 : 6;
    return strength >= 2 ? 6 : 8;
}

static int coarseTimingStep(VehicleType type, int strength) {
    if (continuousMode(type))
        return strength >= 2 ? 16 : 22;
    return strength >= 2 ? 22 : 30;
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

static SearchInput nthInput(InputSet const& inputs, size_t index) {
    auto it = inputs.begin();
    std::advance(it, static_cast<long>(index));
    return *it;
}

static std::vector<InputSet> makeStructuredSeeds(
    Level2& lvl,
    int startFrame,
    int horizonFrames,
    bool player2,
    int strength
) {
    std::vector<InputSet> seeds;
    auto const type = player2 ? lvl.latestState2().vehicle.type : lvl.latestState().vehicle.type;
    auto const durations = usefulHoldDurations(type);

    seeds.emplace_back();

    // If a tap-mode player is currently held, explicitly test releasing now.
    // This prevents a previous jump hold from silently turning into auto-jump spam.
    bool currentlyHeld = player2 ? lvl.press2 : lvl.press1;
    if (currentlyHeld && !continuousMode(type)) {
        InputSet release;
        addToggle(release, startFrame, horizonFrames, 0, player2);
        seeds.push_back(std::move(release));
    }

    int denseWindow = std::min(horizonFrames, strength >= 2 ? 1080 : 780);
    int coarseWindow = std::min(horizonFrames, strength >= 2 ? 2100 : 1500);
    int denseStep = denseTimingStep(type, strength);
    int coarseStep = coarseTimingStep(type, strength);

    for (int offset = 0; offset < denseWindow; offset += denseStep) {
        for (int duration : durations) {
            InputSet input;
            addPulse(input, startFrame, horizonFrames, offset, duration, player2);
            seeds.push_back(std::move(input));
        }
    }

    for (int offset = denseWindow; offset < coarseWindow; offset += coarseStep) {
        for (size_t i = 0; i < durations.size(); i += 2) {
            InputSet input;
            addPulse(input, startFrame, horizonFrames, offset, durations[i], player2);
            seeds.push_back(std::move(input));
        }
    }

    // A modest two-pulse bank catches chained cube/UFO/robot hazards without
    // exploding into the giant deterministic grid from the previous build.
    if (!continuousMode(type)) {
        int pairWindow = std::min(horizonFrames, strength >= 2 ? 720 : 480);
        int duration = durations[durations.size() / 2];
        int offsetStep = strength >= 2 ? 24 : 36;
        for (int offset = 0; offset < pairWindow; offset += offsetStep) {
            for (int gap : {72, 120, 180}) {
                InputSet input;
                addPulse(input, startFrame, horizonFrames, offset, duration, player2);
                addPulse(input, startFrame, horizonFrames, offset + gap, duration, player2);
                seeds.push_back(std::move(input));
            }
        }
    }

    // Keep the straight-fly/wave strength that tested well, but with a smaller
    // pattern bank on normal sections. Hard sections unlock the denser cadences.
    if (continuousMode(type)) {
        int patternWindow = std::min(horizonFrames, strength >= 2 ? 1500 : 1080);
        std::vector<int> periods = strength >= 2
            ? std::vector<int>{24, 32, 40, 48, 64, 80, 96}
            : std::vector<int>{32, 48, 64, 80, 96};

        for (int period : periods) {
            for (int dutyNumerator : {1, 2, 3}) {
                int hold = std::max(2, period * dutyNumerator / 4);
                InputSet pattern;
                for (int offset = 0; offset < patternWindow; offset += period)
                    addPulse(pattern, startFrame, horizonFrames, offset, hold, player2);
                seeds.push_back(std::move(pattern));
            }
        }
    }

    return seeds;
}

static InputSet mutateCandidate(
    InputSet base,
    Level2& lvl,
    int startFrame,
    int horizonFrames,
    std::mt19937& rng,
    int strength
) {
    std::uniform_int_distribution<int> mutationDist(0, 4);
    std::uniform_int_distribution<int> offsetDist(0, std::max(0, horizonFrames - 1));
    std::uniform_int_distribution<int> shiftDist(strength >= 2 ? -48 : -32, strength >= 2 ? 48 : 32);
    bool dual = lvl.latestState().dualActive;

    int mutations = 1 + static_cast<int>(rng() % static_cast<unsigned>(strength >= 2 ? 3 : 2));
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
            base.erase(nthInput(base, index));
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
    float startX = lvl.latestState().pos.x;
    bool dual = lvl.latestState().dualActive;
    auto p1Type = lvl.latestState().vehicle.type;

    // Stage 1: cheap confidence probe. Most easy sections do not need thousands
    // of candidate simulations. If doing nothing safely survives a useful chunk,
    // return immediately and re-plan later from farther ahead.
    int probeHorizon = std::min(horizonFrames, 720 + refinementStrength * 90);
    InputSet noInput;
    CandidateResult quick {noInput, tryInputs(lvl, noInput, probeHorizon)};

    if (quick.trial.complete)
        return quick;

    // For tap modes, prefer releasing a carried hold if it is just as safe.
    if (lvl.press1 && !continuousMode(p1Type)) {
        InputSet release;
        addToggle(release, frame, probeHorizon, 0, false);
        CandidateResult released {release, tryInputs(lvl, release, probeHorizon)};
        if (!released.trial.dead && released.trial.x + 6.f >= quick.trial.x)
            quick = std::move(released);
    }

    if (!quick.trial.dead && quick.trial.frame >= frame + probeHorizon)
        return quick;

    // Stage 2: structured search only when the simple probe actually fails.
    std::vector<InputSet> seeds = makeStructuredSeeds(
        lvl, frame, horizonFrames, false, refinementStrength
    );
    if (dual) {
        auto p2Seeds = makeStructuredSeeds(
            lvl, frame, horizonFrames, true, refinementStrength
        );
        size_t keep = std::min<size_t>(p2Seeds.size(), 80 + refinementStrength * 25);
        for (size_t i = 1; i < keep; ++i)
            seeds.push_back(std::move(p2Seeds[i]));
    }

    int randomCandidates = 120 + refinementStrength * 70;
    std::uniform_int_distribution<int> frameDist(0, std::max(0, horizonFrames - 1));
    int maxP1 = maxToggleBudget(lvl.latestState().vehicle.type);
    int maxP2 = dual ? maxToggleBudget(lvl.latestState2().vehicle.type) : 0;

    for (int i = 0; i < randomCandidates; ++i) {
        InputSet inputs;
        int p1Count = static_cast<int>(rng() % static_cast<unsigned>(maxP1 + 1));
        int p2Count = dual
            ? static_cast<int>(rng() % static_cast<unsigned>(maxP2 + 1))
            : 0;

        for (int j = 0; j < p1Count; ++j)
            addToggle(inputs, frame, horizonFrames, frameDist(rng), false);
        for (int j = 0; j < p2Count; ++j)
            addToggle(inputs, frame, horizonFrames, frameDist(rng), true);
        seeds.push_back(std::move(inputs));
    }

    std::vector<CandidateResult> elites;
    size_t eliteCount = static_cast<size_t>(14 + refinementStrength * 3);

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

    // Keep the best cheap probe in the population so heavier search must truly
    // beat it rather than forgetting a simple route that was already decent.
    elites.push_back(quick);

    for (auto& seed : seeds) {
        if (stop)
            break;
        consider(std::move(seed));
        if (!elites.empty() && elites.front().trial.complete && elites.front().inputs.size() <= 2)
            break;
    }

    int rounds = 3 + refinementStrength;
    int childrenPerRound = 130 + refinementStrength * 75;
    for (int round = 0; round < rounds && !stop && !elites.empty(); ++round) {
        auto parents = elites;
        size_t parentPool = std::min<size_t>(parents.size(), 10 + refinementStrength * 2);

        for (int child = 0; child < childrenPerRound && !stop; ++child) {
            size_t parentIndex = static_cast<size_t>(rng() % parentPool);
            auto mutated = mutateCandidate(
                parents[parentIndex].inputs,
                lvl,
                frame,
                horizonFrames,
                rng,
                refinementStrength
            );
            consider(std::move(mutated));
        }

        if (!elites.empty() && elites.front().trial.complete && elites.front().inputs.size() <= 2)
            break;
    }

    // Precision work is now conditional instead of running on every chunk.
    // Continuous modes get a small timing polish because it helps straight-fly;
    // difficult/stalled sections get the full one-frame refinement.
    if (!elites.empty()) {
        bool needsPrecision = refinementStrength >= 2 || continuousMode(p1Type);
        float gain = elites.front().trial.x - startX;
        if (gain < 350.f)
            needsPrecision = true;

        if (needsPrecision) {
            std::vector<int> shifts = refinementStrength >= 2
                ? std::vector<int>{12, 6, 3, 1}
                : std::vector<int>{4, 1};

            for (int shift : shifts) {
                if (stop || elites.empty())
                    break;

                auto parents = elites;
                size_t parentCount = std::min<size_t>(parents.size(), refinementStrength >= 2 ? 4 : 2);
                for (size_t p = 0; p < parentCount && !stop; ++p) {
                    size_t inputCount = std::min<size_t>(parents[p].inputs.size(), 12);
                    for (size_t i = 0; i < inputCount && !stop; ++i) {
                        auto original = nthInput(parents[p].inputs, i);
                        bool player2 = inputPlayer2(original);
                        int oldOffset = static_cast<int>(inputFrame(original)) - frame;

                        for (int direction : {-1, 1}) {
                            int newOffset = std::clamp(
                                oldOffset + direction * shift,
                                0,
                                horizonFrames - 1
                            );
                            if (newOffset == oldOffset)
                                continue;

                            auto polished = parents[p].inputs;
                            polished.erase(original);
                            addToggle(polished, frame, horizonFrames, newOffset, player2);
                            consider(std::move(polished));
                        }
                    }
                }
            }
        }
    }

    if (elites.empty()) {
        CandidateResult empty;
        empty.trial = {frame, startX, lvl.latestState().dead, false};
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

        // Balanced mode: about 6.25 seconds normally, scaling toward ~10.8
        // seconds only when the solver proves the section is difficult.
        int difficulty = std::min(5, recoveryCount + stagnantRounds / 5);
        int horizonFrames = 1500 + difficulty * 220;
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
            int span = std::max(1, bestFrame - frame);
            int commitNumerator;
            int commitDenominator = 100;

            if (bestComplete) {
                commitNumerator = 100;
            } else if (best.inputs.empty() && !best.trial.dead) {
                // Safe/no-input stretches are cheap and reliable, so move ahead
                // more aggressively before re-planning.
                commitNumerator = 65;
            } else if (difficulty <= 1) {
                commitNumerator = 55;
            } else {
                commitNumerator = 45;
            }

            int applyUntil = bestComplete
                ? bestFrame
                : frame + std::max(1, span * commitNumerator / commitDenominator);

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
            recoveryCount = std::max(0, recoveryCount - 1);
        } else {
            ++stagnantRounds;
        }

        // Keep the useful stall recovery, but escalate search depth instead of
        // making every easy section pay the hard-section cost up front.
        if (stagnantRounds >= 9 && lvlBest.currentFrame() > 2) {
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
            numAway = std::min(9000, 1500 + recoveryCount * 700);
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
