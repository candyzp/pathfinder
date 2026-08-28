#include <set>
#include <cstdint>
#include <algorithm>
#include <cmath>
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
    bool survivedHorizon = false;
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

static bool reachedGoal(Level2& lvl) {
    return lvl.latestState().completed;
}

static bool betterCandidate(CandidateResult const& a, CandidateResult const& b) {
    if (a.trial.complete != b.trial.complete)
        return a.trial.complete;

    // A route that survives the full forecast is categorically better than one
    // that merely gets a few pixels farther before dying. This prevents the
    // solver from repeatedly steering itself into the same spike wall.
    if (a.trial.survivedHorizon != b.trial.survivedHorizon)
        return a.trial.survivedHorizon;

    if (a.trial.dead != b.trial.dead)
        return !a.trial.dead;

    if (a.trial.survivedHorizon && b.trial.survivedHorizon) {
        if (a.inputs.size() != b.inputs.size())
            return a.inputs.size() < b.inputs.size();
        if (a.trial.x != b.trial.x)
            return a.trial.x > b.trial.x;
        return a.trial.frame > b.trial.frame;
    }

    // Among routes that all die, farther progress still provides useful search
    // information, but tiny differences are not worth extra noisy inputs.
    constexpr float progressTie = 12.f;
    if (a.trial.x > b.trial.x + progressTie)
        return true;
    if (b.trial.x > a.trial.x + progressTie)
        return false;
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
        return strength >= 2 ? 5 : 7;
    return strength >= 2 ? 7 : 10;
}

static int coarseTimingStep(VehicleType type, int strength) {
    if (continuousMode(type))
        return strength >= 2 ? 18 : 26;
    return strength >= 2 ? 26 : 36;
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
           !reachedGoal(lvl)) {
        uint32_t current = static_cast<uint32_t>(lvl.currentFrame());
        auto p1 = inputKey(current, false);
        auto p2 = inputKey(current, true);

        if (inputs.contains(p1))
            lvl.press1 = !lvl.press1;
        if (inputs.contains(p2))
            lvl.press2 = !lvl.press2;

        lvl.runFrame(lvl.press1, lvl.press2, 1.f / 240.f);
    }

    bool complete = !lvl.latestState().dead && reachedGoal(lvl);
    TrialResult result {
        lvl.currentFrame(),
        lvl.latestState().pos.x,
        lvl.latestState().dead,
        complete,
        !complete && !lvl.latestState().dead && lvl.currentFrame() >= endFrame
    };

    // Natural end crossing sets Player::completed in Level::runFrame. If a route
    // somehow appears well beyond the endpoint without completion, it got there
    // through a bad teleport/target resolution and must never be ranked as progress.
    if (!result.complete && result.x > lvl.length + 120.f) {
        result.dead = true;
        result.survivedHorizon = false;
    }

    float lastY = lvl.latestState().pos.y;
    if (!result.complete && result.x < lvl.length &&
        (lastY > std::max(1300.f, lvl.highestY + 600.f) || lastY < -600.f)) {
        result.frame = startFrame;
        result.dead = true;
        result.survivedHorizon = false;
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

    bool currentlyHeld = player2 ? lvl.press2 : lvl.press1;
    if (currentlyHeld && !continuousMode(type)) {
        InputSet release;
        addToggle(release, startFrame, horizonFrames, 0, player2);
        seeds.push_back(std::move(release));
    }

    // Keep broad coverage, but stop spending enormous time on timings that are
    // far beyond the next obstacle. A targeted danger bank below handles the
    // immediate collision with much denser timing.
    int denseWindow = std::min(horizonFrames, strength >= 2 ? 840 : 600);
    int coarseWindow = std::min(horizonFrames, strength >= 2 ? 1680 : 1200);
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

    if (!continuousMode(type)) {
        int pairWindow = std::min(horizonFrames, strength >= 2 ? 600 : 420);
        int duration = durations[durations.size() / 2];
        int offsetStep = strength >= 2 ? 30 : 42;
        for (int offset = 0; offset < pairWindow; offset += offsetStep) {
            for (int gap : {72, 120, 180}) {
                InputSet input;
                addPulse(input, startFrame, horizonFrames, offset, duration, player2);
                addPulse(input, startFrame, horizonFrames, offset + gap, duration, player2);
                seeds.push_back(std::move(input));
            }
        }
    }

    if (continuousMode(type)) {
        int patternWindow = std::min(horizonFrames, strength >= 2 ? 1320 : 900);
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
    std::uniform_int_distribution<int> shiftDist(strength >= 2 ? -40 : -28, strength >= 2 ? 40 : 28);
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

    // Short probe first. If there is no imminent danger, do not make an easy
    // section pay for a giant evolutionary search.
    int probeHorizon = std::min(horizonFrames, 480 + refinementStrength * 60);
    InputSet noInput;
    CandidateResult quick {noInput, tryInputs(lvl, noInput, probeHorizon)};

    if (quick.trial.complete)
        return quick;

    if (lvl.press1 && !continuousMode(p1Type)) {
        InputSet release;
        addToggle(release, frame, probeHorizon, 0, false);
        CandidateResult released {release, tryInputs(lvl, release, probeHorizon)};
        if (betterCandidate(released, quick))
            quick = std::move(released);
    }

    if (quick.trial.survivedHorizon)
        return quick;

    std::vector<InputSet> seeds;

    // If the simple route dies, its death frame is valuable information. Test
    // inputs specifically before that danger instead of wasting most of the
    // search budget on timings far away from the obstacle.
    if (quick.trial.dead && quick.trial.frame > frame) {
        int dangerOffset = std::clamp(quick.trial.frame - frame, 1, horizonFrames - 1);
        auto durations = usefulHoldDurations(p1Type);
        std::vector<int> leads = continuousMode(p1Type)
            ? std::vector<int>{12, 20, 28, 36, 48, 64, 80, 104, 136}
            : std::vector<int>{24, 36, 48, 60, 72, 96, 120, 156, 192};

        for (int lead : leads) {
            int offset = dangerOffset - lead;
            if (offset < 0)
                continue;
            for (int duration : durations) {
                InputSet focused;
                addPulse(focused, frame, horizonFrames, offset, duration, false);
                seeds.push_back(std::move(focused));
            }
        }
    }

    auto broadSeeds = makeStructuredSeeds(lvl, frame, horizonFrames, false, refinementStrength);
    size_t broadLimit = std::min<size_t>(
        broadSeeds.size(),
        static_cast<size_t>(180 + refinementStrength * 60)
    );
    for (size_t i = 0; i < broadLimit; ++i)
        seeds.push_back(std::move(broadSeeds[i]));

    if (dual) {
        auto p2Seeds = makeStructuredSeeds(lvl, frame, horizonFrames, true, refinementStrength);
        size_t keep = std::min<size_t>(p2Seeds.size(), 50 + refinementStrength * 15);
        for (size_t i = 1; i < keep; ++i)
            seeds.push_back(std::move(p2Seeds[i]));
    }

    int randomCandidates = 50 + refinementStrength * 30;
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
    size_t eliteCount = static_cast<size_t>(10 + refinementStrength * 2);

    auto consider = [&](InputSet inputs) -> bool {
        if (stop)
            return false;

        CandidateResult candidate;
        candidate.trial = tryInputs(lvl, inputs, horizonFrames);
        candidate.inputs = std::move(inputs);
        bool survived = candidate.trial.survivedHorizon;

        elites.push_back(std::move(candidate));
        std::sort(elites.begin(), elites.end(), betterCandidate);
        if (elites.size() > eliteCount)
            elites.resize(eliteCount);
        return survived;
    };

    elites.push_back(quick);
    std::sort(elites.begin(), elites.end(), betterCandidate);

    int survivorHits = 0;
    for (auto& seed : seeds) {
        if (stop)
            break;
        if (consider(std::move(seed)))
            ++survivorHits;

        if (!elites.empty() && elites.front().trial.complete)
            break;
        if (!elites.empty() && elites.front().trial.survivedHorizon &&
            elites.front().inputs.size() <= 2 && survivorHits >= 2)
            break;
    }

    if (!elites.empty() && elites.front().trial.survivedHorizon &&
        elites.front().inputs.size() <= 2)
        return elites.front();

    int rounds = 2 + std::min(refinementStrength, 2);
    int childrenPerRound = 60 + refinementStrength * 35;
    for (int round = 0; round < rounds && !stop && !elites.empty(); ++round) {
        auto parents = elites;
        size_t parentPool = std::min<size_t>(parents.size(), 6 + refinementStrength * 2);

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

            if (!elites.empty() && elites.front().trial.survivedHorizon &&
                elites.front().inputs.size() <= 2)
                break;
        }

        if (!elites.empty() && elites.front().trial.complete)
            break;
    }

    // Precision polishing is useful only when we still have not found a clean
    // full-horizon survivor. Do not burn one-frame sweeps on already-safe routes.
    if (!elites.empty() && !elites.front().trial.survivedHorizon) {
        bool needsPrecision = refinementStrength >= 2 || continuousMode(p1Type);
        float gain = elites.front().trial.x - startX;
        if (gain < 300.f)
            needsPrecision = true;

        if (needsPrecision) {
            std::vector<int> shifts = refinementStrength >= 2
                ? std::vector<int>{8, 4, 2, 1}
                : std::vector<int>{4, 2};

            for (int shift : shifts) {
                if (stop || elites.empty() || elites.front().trial.survivedHorizon)
                    break;

                auto parents = elites;
                size_t parentCount = std::min<size_t>(parents.size(), refinementStrength >= 2 ? 3 : 2);
                for (size_t p = 0; p < parentCount && !stop; ++p) {
                    size_t inputCount = std::min<size_t>(parents[p].inputs.size(), 10);
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
        empty.trial = {frame, startX, lvl.latestState().dead, false, false};
        return empty;
    }

    return elites.front();
}

PathfinderResult pathfind(
    std::string const& lvlString,
    std::atomic_bool& stop,
    std::function<void(double)> callback,
    float trustedEndX
) {
    Level2 lvl(lvlString);
    float solveStartX = lvl.latestState().pos.x;

    // When Pathfinder is launched from an active PlayLayer, Geometry Dash already
    // knows the exact end position. Prefer that over any object-cloud inference.
    if (std::isfinite(trustedEndX) && trustedEndX > solveStartX + 30.f)
        lvl.length = trustedEndX;

    auto progressFor = [&](float x, bool complete) -> double {
        if (complete)
            return 100.0;
        double span = static_cast<double>(lvl.length - solveStartX);
        if (span <= 1.0)
            return 0.0;
        return std::clamp(
            ((static_cast<double>(x) - solveStartX) / span) * 100.0,
            0.0,
            100.0
        );
    };

    std::random_device rd;
    std::mt19937 rng(rd());

    int trueBest = 0;
    int fail = 1;
    int numAway = 1000;
    int stagnantRounds = 0;
    int recoveryCount = 0;
    int repeatedDeathZone = 0;
    float lastDeathX = -100000.f;
    float furthestX = solveStartX;

    Level2 lvlBest = lvl;

    while (!reachedGoal(lvl) && !stop) {
        auto frame = lvl.currentFrame();

        // Faster normal forecast, with extra depth reserved for sections that
        // actually prove difficult or repeatedly kill the same route.
        int difficulty = std::min(
            4,
            recoveryCount + stagnantRounds / 3 + repeatedDeathZone / 2
        );
        int horizonFrames = 1200 + difficulty * 150;
        auto best = searchBestInputs(lvl, stop, rng, horizonFrames, difficulty);

        if (stop)
            break;

        int bestFrame = best.trial.frame;
        bool bestComplete = best.trial.complete;

        if (best.trial.dead && !bestComplete) {
            if (std::abs(best.trial.x - lastDeathX) <= 40.f)
                ++repeatedDeathZone;
            else
                repeatedDeathZone = 1;
            lastDeathX = best.trial.x;
        } else {
            repeatedDeathZone = 0;
            lastDeathX = -100000.f;
        }

        // Three searches dying at essentially the same X means we are trapped in
        // a local solution. Retreat immediately instead of spending nine more
        // expensive rounds rediscovering the same spike.
        if (repeatedDeathZone >= 3 && lvlBest.currentFrame() > 2) {
            lvl = lvlBest;
            int retreat = std::min(
                lvlBest.currentFrame() - 1,
                480 + std::min(recoveryCount, 6) * 240 + repeatedDeathZone * 120
            );
            lvl.rollback(std::max(1, lvlBest.currentFrame() - retreat));
            lvl.syncPresses();

            ++recoveryCount;
            stagnantRounds = 0;
            repeatedDeathZone = 0;
            fail = 1;
            numAway = std::min(7000, 1200 + recoveryCount * 600);
            rng.seed(rd() ^ static_cast<unsigned int>(lvl.currentFrame() + recoveryCount * 7919));
            continue;
        }

        if (!bestComplete && (bestFrame == frame || best.trial.x <= lvl.latestState().pos.x)) {
            lvl.rollback(std::max(std::max(frame - fail, trueBest - numAway), 1));
            lvl.syncPresses();

            fail += 5;
            if (fail > numAway + 1000) {
                numAway += 800;
                fail = 1;

                if (numAway > 9000) {
                    numAway = 1200;
                    trueBest = 0;
                    lvl.rollback(1);
                    lvl.syncPresses();
                }
            } else if (fail > 100) {
                fail += 40;
            }
        } else {
            int span = std::max(1, bestFrame - frame);
            int commitNumerator;
            int commitDenominator = 100;

            if (bestComplete) {
                commitNumerator = 100;
            } else if (best.trial.dead) {
                // A doomed route is only a hint about where to move next. Never
                // trust half of it and walk directly toward its known collision.
                commitNumerator = difficulty <= 1 ? 18 : 12;
            } else if (best.trial.survivedHorizon && best.inputs.empty()) {
                commitNumerator = 65;
            } else if (best.trial.survivedHorizon) {
                commitNumerator = difficulty <= 1 ? 52 : 44;
            } else {
                commitNumerator = 30;
            }

            int committedSpan = std::max(1, span * commitNumerator / commitDenominator);
            if (best.trial.dead)
                committedSpan = std::min(committedSpan, 240);

            int applyUntil = bestComplete
                ? bestFrame
                : frame + committedSpan;

            for (int i = frame; i < applyUntil && !lvl.latestState().dead; ++i) {
                auto p1 = inputKey(static_cast<uint32_t>(i), false);
                auto p2 = inputKey(static_cast<uint32_t>(i), true);
                if (best.inputs.contains(p1))
                    lvl.press1 = !lvl.press1;
                if (best.inputs.contains(p2))
                    lvl.press2 = !lvl.press2;

                lvl.runFrame(lvl.press1, lvl.press2, 1.f / 240.f);
                if (reachedGoal(lvl))
                    break;
            }
        }

        // If a malformed teleport/group target flings the player beyond the known
        // endpoint without actually completing, invalidate that route immediately.
        if (!lvl.latestState().completed && lvl.latestState().pos.x > lvl.length + 120.f)
            lvl.latestState().dead = true;

        if (lvl.currentFrame() > trueBest) {
            trueBest = lvl.currentFrame();
            fail = 0;
            numAway = 1000;
        }

        if (!lvl.latestState().dead && reachedGoal(lvl)) {
            furthestX = std::max(furthestX, lvl.latestState().pos.x);
            lvlBest = lvl;
            stagnantRounds = 0;
        } else if (!lvl.latestState().dead && lvl.latestState().pos.x > furthestX + 1.f) {
            furthestX = lvl.latestState().pos.x;
            lvlBest = lvl;
            stagnantRounds = 0;
            recoveryCount = std::max(0, recoveryCount - 1);
        } else {
            ++stagnantRounds;
        }

        // Recover much sooner. Each search is expensive, so five stagnant rounds
        // is already enough evidence that the current local path is bad.
        if (stagnantRounds >= 5 && lvlBest.currentFrame() > 2) {
            lvl = lvlBest;
            int retreat = std::min(
                lvlBest.currentFrame() - 1,
                600 * (1 + std::min(recoveryCount, 8))
            );
            lvl.rollback(std::max(1, lvlBest.currentFrame() - retreat));
            lvl.syncPresses();

            ++recoveryCount;
            stagnantRounds = 0;
            repeatedDeathZone = 0;
            fail = 1;
            numAway = std::min(7500, 1200 + recoveryCount * 600);
            rng.seed(rd() ^ static_cast<unsigned int>(lvl.currentFrame() + recoveryCount * 7919));
        }

        if (callback && lvl.length > solveStartX) {
            bool bestIsComplete = !lvlBest.latestState().dead && reachedGoal(lvlBest);
            callback(progressFor(furthestX, bestIsComplete));
        }
    }

    PathfinderResult result;
    if (!lvl.latestState().dead &&
        (reachedGoal(lvl) || lvl.latestState().pos.x > furthestX)) {
        furthestX = std::max(furthestX, lvl.latestState().pos.x);
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
    result.complete = !lvlBest.latestState().dead && reachedGoal(lvlBest);
    result.progress = progressFor(furthestX, result.complete);
    return result;
}
