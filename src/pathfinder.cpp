#include <set>
#include <cstdint>
#include <algorithm>
#include <cmath>
#include <functional>
#include <Level.hpp>
#include <random>
#include <limits>
#include <vector>
#include <iterator>
#include <unordered_set>
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
    float y = 0.f;
    double velocity = 0.0;
    float clearance = 0.f;
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

static float localClearance(Level2& lvl, Player const& player) {
    if (lvl.sections.empty())
        return 999.f;

    int section = std::clamp(
        static_cast<int>(player.pos.x / Level::sectionSize),
        0,
        static_cast<int>(lvl.sections.size()) - 1
    );

    float best = 999.f;
    int first = std::max(0, section - 2);
    int last = std::min(static_cast<int>(lvl.sections.size()) - 1, section + 2);

    for (int s = first; s <= last; ++s) {
        for (auto const& object : lvl.sections[s]) {
            if (object->prio != 1 && object->prio != 2)
                continue;

            float dx = 0.f;
            if (player.getRight() < object->getLeft())
                dx = object->getLeft() - player.getRight();
            else if (object->getRight() < player.getLeft())
                dx = player.getLeft() - object->getRight();

            float dy = 0.f;
            if (player.getTop() < object->getBottom())
                dy = object->getBottom() - player.getTop();
            else if (object->getTop() < player.getBottom())
                dy = player.getBottom() - object->getTop();

            float distance = std::sqrt(dx * dx + dy * dy);
            best = std::min(best, distance);
        }
    }

    return best;
}

static bool betterCandidate(CandidateResult const& a, CandidateResult const& b) {
    if (a.trial.complete != b.trial.complete)
        return a.trial.complete;

    if (a.trial.survivedHorizon != b.trial.survivedHorizon)
        return a.trial.survivedHorizon;

    if (a.trial.dead != b.trial.dead)
        return !a.trial.dead;

    // If every forecast dies, surviving longer is the most useful signal. A
    // candidate that clears the current obstacle and dies at the next one is much
    // more valuable than a route that gains a few extra pixels then dies now.
    if (a.trial.dead && b.trial.dead && a.trial.frame != b.trial.frame)
        return a.trial.frame > b.trial.frame;

    // Progress matters before click count. For flying modes most surviving
    // candidates reach nearly the same X, so clearance is the useful tie-breaker.
    constexpr float progressTie = 8.f;
    if (a.trial.x > b.trial.x + progressTie)
        return true;
    if (b.trial.x > a.trial.x + progressTie)
        return false;

    if (a.trial.clearance > b.trial.clearance + 1.5f)
        return true;
    if (b.trial.clearance > a.trial.clearance + 1.5f)
        return false;

    if (a.trial.frame != b.trial.frame)
        return a.trial.frame > b.trial.frame;

    // Never reward "fewer inputs" as a strategy. If two routes are truly
    // equivalent, leaving them tied keeps high-frequency winning routes alive.
    return false;
}

static int maxPulseBudget(VehicleType type) {
    switch (type) {
        case VehicleType::Cube:   return 6;
        case VehicleType::Ship:   return 22;
        case VehicleType::Ball:   return 8;
        case VehicleType::Ufo:    return 10;
        case VehicleType::Wave:   return 40;
        case VehicleType::Robot:  return 10;
        case VehicleType::Spider: return 8;
        case VehicleType::Swing:  return 24;
    }
    return 10;
}

static std::vector<int> usefulHoldDurations(VehicleType type) {
    switch (type) {
        case VehicleType::Cube:   return {1, 2, 3, 4, 6};
        case VehicleType::Ball:   return {1, 2, 3, 4};
        case VehicleType::Ufo:    return {1, 2, 3, 4, 6};
        case VehicleType::Spider: return {1, 2, 3, 4};
        case VehicleType::Robot:  return {3, 6, 10, 16, 24, 40, 64, 96};
        case VehicleType::Ship:   return {2, 4, 6, 10, 16, 24, 40, 64, 96, 144};
        case VehicleType::Wave:   return {1, 2, 3, 4, 6, 8, 12, 16, 24, 32, 48, 64};
        case VehicleType::Swing:  return {2, 4, 6, 10, 16, 24, 40, 64, 96};
    }
    return {2, 4, 8};
}

static int denseTimingStep(VehicleType type, int strength) {
    if (type == VehicleType::Wave)
        return strength >= 2 ? 4 : 6;
    if (continuousMode(type))
        return strength >= 2 ? 6 : 8;
    return strength >= 2 ? 8 : 12;
}

static int coarseTimingStep(VehicleType type, int strength) {
    if (type == VehicleType::Wave)
        return strength >= 2 ? 14 : 20;
    if (continuousMode(type))
        return strength >= 2 ? 20 : 28;
    return strength >= 2 ? 28 : 40;
}

static void addToggle(InputSet& inputs, int startFrame, int horizon, int offset, bool player2) {
    if (offset < 0 || offset >= horizon)
        return;
    inputs.insert(inputKey(static_cast<uint32_t>(startFrame + offset), player2));
}

static void addPulse(
    InputSet& inputs,
    int startFrame,
    int horizon,
    int offset,
    int duration,
    bool player2
) {
    if (offset < 0 || offset >= horizon - 1)
        return;

    duration = std::clamp(duration, 1, horizon - offset - 1);
    addToggle(inputs, startFrame, horizon, offset, player2);
    addToggle(inputs, startFrame, horizon, offset + duration, player2);
}

static uint64_t hashInputs(InputSet const& inputs) {
    uint64_t h = 1469598103934665603ull;
    for (auto value : inputs) {
        h ^= static_cast<uint64_t>(value) + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
        h *= 1099511628211ull;
    }
    h ^= static_cast<uint64_t>(inputs.size()) * 0x517cc1b727220a95ull;
    return h;
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

    auto const& p1 = lvl.latestState();
    bool complete = !p1.dead && reachedGoal(lvl);
    float clearance = p1.dead ? 0.f : localClearance(lvl, p1);

    if (p1.dualActive && !p1.dead)
        clearance = std::min(clearance, localClearance(lvl, lvl.latestState2()));

    TrialResult result {
        lvl.currentFrame(),
        p1.pos.x,
        p1.pos.y,
        p1.velocity,
        clearance,
        p1.dead,
        complete,
        !complete && !p1.dead && lvl.currentFrame() >= endFrame
    };

    // A malformed teleport can fling X far beyond the real endpoint. Never let
    // that become "progress" or a recovery checkpoint.
    if (!result.complete && result.x > lvl.length + 120.f) {
        result.dead = true;
        result.survivedHorizon = false;
        result.clearance = 0.f;
    }

    if (!result.complete && result.x < lvl.length &&
        (result.y > std::max(1300.f, lvl.highestY + 600.f) || result.y < -600.f)) {
        result.frame = startFrame;
        result.dead = true;
        result.survivedHorizon = false;
        result.clearance = 0.f;
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

static void addRapidTrain(
    InputSet& inputs,
    int startFrame,
    int horizon,
    int window,
    int interval,
    int phase,
    bool player2
) {
    window = std::min(window, horizon);
    for (int offset = phase; offset < window; offset += interval)
        addToggle(inputs, startFrame, horizon, offset, player2);
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
    if (currentlyHeld) {
        InputSet release;
        addToggle(release, startFrame, horizonFrames, 0, player2);
        seeds.push_back(std::move(release));
    }

    // Wave gets high-frequency corridor-tracing seeds first, before broad pulses.
    // This means the solver can discover spammy zig-zag routes without waiting
    // for random mutation to accidentally invent them.
    if (type == VehicleType::Wave) {
        int patternWindow = std::min(horizonFrames, strength >= 2 ? 900 : 720);

        for (int interval : {2, 3, 4, 5, 6, 8, 10, 12}) {
            for (int phase : {0, 1}) {
                InputSet train;
                addRapidTrain(train, startFrame, horizonFrames, patternWindow, interval, phase, player2);
                seeds.push_back(std::move(train));
            }
        }

        for (int period : {4, 6, 8, 10, 12, 16, 20, 24, 32, 40}) {
            for (int dutyNumerator : {1, 2, 3}) {
                for (int phase : {0, period / 2}) {
                    int hold = std::max(1, period * dutyNumerator / 4);
                    InputSet pattern;
                    for (int offset = phase; offset < patternWindow; offset += period)
                        addPulse(pattern, startFrame, horizonFrames, offset, hold, player2);
                    seeds.push_back(std::move(pattern));
                }
            }
        }
    } else if (continuousMode(type)) {
        int patternWindow = std::min(horizonFrames, strength >= 2 ? 960 : 720);
        for (int period : {12, 16, 20, 24, 32, 40, 48, 64, 80}) {
            for (int dutyNumerator : {1, 2, 3}) {
                int hold = std::max(2, period * dutyNumerator / 4);
                InputSet pattern;
                for (int offset = 0; offset < patternWindow; offset += period)
                    addPulse(pattern, startFrame, horizonFrames, offset, hold, player2);
                seeds.push_back(std::move(pattern));
            }
        }
    }

    int denseWindow = std::min(horizonFrames, strength >= 2 ? 720 : 540);
    int coarseWindow = std::min(horizonFrames, strength >= 2 ? 1200 : 900);
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
        int pairWindow = std::min(horizonFrames, strength >= 2 ? 540 : 360);
        int duration = durations[durations.size() / 2];
        for (int offset = 0; offset < pairWindow; offset += (strength >= 2 ? 28 : 40)) {
            for (int gap : {60, 90, 120, 180}) {
                InputSet input;
                addPulse(input, startFrame, horizonFrames, offset, duration, player2);
                addPulse(input, startFrame, horizonFrames, offset + gap, duration, player2);
                seeds.push_back(std::move(input));
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
    bool dual = lvl.latestState().dualActive;
    std::uniform_int_distribution<int> mutationDist(0, 5);
    std::uniform_int_distribution<int> offsetDist(0, std::max(0, horizonFrames - 2));
    std::uniform_int_distribution<int> shiftDist(
        strength >= 2 ? -32 : -20,
        strength >= 2 ? 32 : 20
    );

    int mutations = 1 + static_cast<int>(rng() % static_cast<unsigned>(strength >= 2 ? 3 : 2));
    for (int m = 0; m < mutations; ++m) {
        int kind = mutationDist(rng);

        bool player2 = dual && ((rng() & 3u) == 0u);
        auto type = player2 ? lvl.latestState2().vehicle.type : lvl.latestState().vehicle.type;

        if (kind == 0 || base.empty()) {
            auto durations = usefulHoldDurations(type);
            int offset = offsetDist(rng);
            int duration = durations[rng() % durations.size()];
            addPulse(base, startFrame, horizonFrames, offset, duration, player2);
        } else if (kind == 1 && !base.empty()) {
            size_t index = static_cast<size_t>(rng() % base.size());
            auto value = nthInput(base, index);
            bool p2 = inputPlayer2(value);
            int oldOffset = static_cast<int>(inputFrame(value)) - startFrame;
            int newOffset = std::clamp(oldOffset + shiftDist(rng), 0, horizonFrames - 1);
            base.erase(value);
            addToggle(base, startFrame, horizonFrames, newOffset, p2);
        } else if (kind == 2 && !base.empty()) {
            size_t index = static_cast<size_t>(rng() % base.size());
            base.erase(nthInput(base, index));
        } else if (kind == 3) {
            addToggle(base, startFrame, horizonFrames, offsetDist(rng), player2);
        } else if (kind == 4 && type == VehicleType::Wave) {
            int interval = 2 + static_cast<int>(rng() % 9u);
            int window = std::min(horizonFrames, 240 + static_cast<int>(rng() % 480u));
            int phase = static_cast<int>(rng() % static_cast<unsigned>(interval));
            addRapidTrain(base, startFrame, horizonFrames, window, interval, phase, player2);
        } else if (kind == 5 && continuousMode(type)) {
            auto durations = usefulHoldDurations(type);
            int first = offsetDist(rng);
            int period = 8 + static_cast<int>(rng() % 48u);
            int duration = durations[rng() % durations.size()];
            for (int offset = first; offset < std::min(horizonFrames, first + 360); offset += period)
                addPulse(base, startFrame, horizonFrames, offset, duration, player2);
        }
    }

    return base;
}

static bool sameStateBucket(CandidateResult const& a, CandidateResult const& b) {
    return a.trial.complete == b.trial.complete &&
           a.trial.dead == b.trial.dead &&
           a.trial.survivedHorizon == b.trial.survivedHorizon &&
           std::abs(a.trial.x - b.trial.x) < 24.f &&
           std::abs(a.trial.y - b.trial.y) < 22.f &&
           std::abs(a.trial.velocity - b.trial.velocity) < 90.0;
}

static void insertDiverseElite(
    std::vector<CandidateResult>& elites,
    CandidateResult candidate,
    size_t eliteCount
) {
    for (auto& existing : elites) {
        if (!sameStateBucket(existing, candidate))
            continue;

        if (betterCandidate(candidate, existing))
            existing = std::move(candidate);

        std::sort(elites.begin(), elites.end(), betterCandidate);
        return;
    }

    elites.push_back(std::move(candidate));
    std::sort(elites.begin(), elites.end(), betterCandidate);
    if (elites.size() > eliteCount)
        elites.resize(eliteCount);
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

    // Cheap probe only diagnoses immediate danger. It is never mixed into the
    // full-horizon elite pool as if it survived the longer search.
    int probeHorizon = std::min(horizonFrames, 300 + refinementStrength * 45);
    InputSet noInput;
    CandidateResult probe {noInput, tryInputs(lvl, noInput, probeHorizon)};

    if (probe.trial.complete)
        return probe;

    // Easy ground sections can take the fast path. Flying modes always inspect
    // alternative trajectories so "hold forever" cannot win by default.
    if (!continuousMode(p1Type) && probe.trial.survivedHorizon)
        return probe;

    std::vector<InputSet> seeds;

    // Focus around the first known collision. For Wave, use shorter lead times
    // because small timing changes rapidly alter the diagonal trajectory.
    if (probe.trial.dead && probe.trial.frame > frame) {
        int dangerOffset = std::clamp(probe.trial.frame - frame, 1, horizonFrames - 1);
        auto durations = usefulHoldDurations(p1Type);
        std::vector<int> leads = p1Type == VehicleType::Wave
            ? std::vector<int>{4, 8, 12, 16, 24, 32, 48, 64, 88, 120}
            : continuousMode(p1Type)
                ? std::vector<int>{8, 16, 24, 36, 48, 64, 88, 120}
                : std::vector<int>{20, 32, 44, 60, 76, 96, 128, 160};

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
        static_cast<size_t>(
            p1Type == VehicleType::Wave
                ? 300 + refinementStrength * 40
                : 170 + refinementStrength * 35
        )
    );
    for (size_t i = 0; i < broadLimit; ++i)
        seeds.push_back(std::move(broadSeeds[i]));

    if (dual) {
        auto p2Seeds = makeStructuredSeeds(lvl, frame, horizonFrames, true, refinementStrength);
        size_t keep = std::min<size_t>(
            p2Seeds.size(),
            static_cast<size_t>(60 + refinementStrength * 15)
        );
        for (size_t i = 1; i < keep; ++i)
            seeds.push_back(std::move(p2Seeds[i]));
    }

    int randomCandidates = (p1Type == VehicleType::Wave ? 55 : 30) + refinementStrength * 15;
    std::uniform_int_distribution<int> frameDist(0, std::max(0, horizonFrames - 2));

    for (int i = 0; i < randomCandidates; ++i) {
        InputSet inputs;
        int p1Pulses = static_cast<int>(
            rng() % static_cast<unsigned>(maxPulseBudget(lvl.latestState().vehicle.type) + 1)
        );
        for (int j = 0; j < p1Pulses; ++j) {
            auto durations = usefulHoldDurations(lvl.latestState().vehicle.type);
            addPulse(
                inputs,
                frame,
                horizonFrames,
                frameDist(rng),
                durations[rng() % durations.size()],
                false
            );
        }

        if (dual) {
            int p2Pulses = static_cast<int>(
                rng() % static_cast<unsigned>(maxPulseBudget(lvl.latestState2().vehicle.type) + 1)
            );
            for (int j = 0; j < p2Pulses; ++j) {
                auto durations = usefulHoldDurations(lvl.latestState2().vehicle.type);
                addPulse(
                    inputs,
                    frame,
                    horizonFrames,
                    frameDist(rng),
                    durations[rng() % durations.size()],
                    true
                );
            }
        }

        seeds.push_back(std::move(inputs));
    }

    std::vector<CandidateResult> elites;
    std::unordered_set<uint64_t> seen;
    size_t eliteCount = static_cast<size_t>(
        (continuousMode(p1Type) ? 18 : 12) + refinementStrength * 2
    );

    auto consider = [&](InputSet inputs) -> bool {
        if (stop)
            return false;

        uint64_t key = hashInputs(inputs);
        if (!seen.insert(key).second)
            return false;

        CandidateResult candidate;
        candidate.inputs = std::move(inputs);
        candidate.trial = tryInputs(lvl, candidate.inputs, horizonFrames);
        bool survived = candidate.trial.survivedHorizon;

        insertDiverseElite(elites, std::move(candidate), eliteCount);
        return survived;
    };

    // Full-horizon no-input candidate. This is intentionally different from
    // the short probe above.
    consider(InputSet {});

    int survivorHits = 0;
    for (auto& seed : seeds) {
        if (stop)
            break;

        if (consider(std::move(seed)))
            ++survivorHits;

        if (!elites.empty() && elites.front().trial.complete)
            break;

        if (!elites.empty() && elites.front().trial.survivedHorizon) {
            int needed = continuousMode(p1Type) ? 6 : 2;
            float clearanceNeeded = p1Type == VehicleType::Wave ? 14.f : 10.f;
            if (survivorHits >= needed && elites.front().trial.clearance >= clearanceNeeded)
                break;
        }
    }

    if (!elites.empty() && elites.front().trial.complete)
        return elites.front();

    bool strongSurvivor =
        !elites.empty() &&
        elites.front().trial.survivedHorizon &&
        elites.front().trial.clearance >= (p1Type == VehicleType::Wave ? 20.f : 14.f);

    if (strongSurvivor)
        return elites.front();

    int rounds = 1 + std::min(refinementStrength, 2);
    int childrenPerRound = 40 + refinementStrength * 25 + (continuousMode(p1Type) ? 20 : 0);

    for (int round = 0; round < rounds && !stop && !elites.empty(); ++round) {
        auto parents = elites;
        size_t parentPool = std::min<size_t>(
            parents.size(),
            static_cast<size_t>(continuousMode(p1Type) ? 10 : 7)
        );

        for (int child = 0; child < childrenPerRound && !stop; ++child) {
            auto mutated = mutateCandidate(
                parents[static_cast<size_t>(rng() % parentPool)].inputs,
                lvl,
                frame,
                horizonFrames,
                rng,
                refinementStrength
            );
            consider(std::move(mutated));

            if (!elites.empty() && elites.front().trial.complete)
                break;

            if (!elites.empty() &&
                elites.front().trial.survivedHorizon &&
                elites.front().trial.clearance >= (p1Type == VehicleType::Wave ? 24.f : 18.f)) {
                break;
            }
        }

        if (!elites.empty() && elites.front().trial.complete)
            break;
    }

    // Small precision pass only when the best route is still unsafe or flying
    // extremely close to geometry. This is much cheaper than polishing every
    // candidate on every easy section.
    if (!elites.empty()) {
        bool needsPrecision =
            !elites.front().trial.survivedHorizon ||
            (continuousMode(p1Type) && elites.front().trial.clearance < 12.f);

        if (needsPrecision) {
            std::vector<int> shifts = refinementStrength >= 2
                ? std::vector<int>{4, 2, 1}
                : std::vector<int>{2, 1};

            for (int shift : shifts) {
                if (stop || elites.empty() || elites.front().trial.complete)
                    break;

                auto parents = elites;
                size_t parentCount = std::min<size_t>(parents.size(), 2);

                for (size_t p = 0; p < parentCount && !stop; ++p) {
                    size_t inputCount = std::min<size_t>(parents[p].inputs.size(), 12);
                    for (size_t i = 0; i < inputCount && !stop; ++i) {
                        auto original = nthInput(parents[p].inputs, i);
                        bool p2 = inputPlayer2(original);
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
                            addToggle(polished, frame, horizonFrames, newOffset, p2);
                            consider(std::move(polished));
                        }
                    }
                }

                if (!elites.empty() &&
                    elites.front().trial.survivedHorizon &&
                    elites.front().trial.clearance >= 16.f) {
                    break;
                }
            }
        }
    }

    if (elites.empty()) {
        CandidateResult empty;
        empty.trial = {
            frame,
            startX,
            lvl.latestState().pos.y,
            lvl.latestState().velocity,
            0.f,
            lvl.latestState().dead,
            false,
            false
        };
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

    uint32_t baseSeed = static_cast<uint32_t>(std::hash<std::string>{}(lvlString));
    baseSeed ^= static_cast<uint32_t>(std::llround(std::max(0.f, trustedEndX)));
    baseSeed ^= 0x9e3779b9u;

    // Keep the structured search deterministic inside one run, but give a fresh
    // launch a different branch ordering. Re-running a difficult level should be
    // able to escape a local minimum instead of replaying the exact same failure.
    std::random_device rd;
    baseSeed ^= rd();
    std::mt19937 rng(baseSeed);

    int stagnantRounds = 0;
    int recoveryLevel = 0;
    int repeatedDeathZone = 0;
    float lastDeathX = -100000.f;
    float furthestX = solveStartX;

    Level2 lvlBest = lvl;

    auto recover = [&](int extraRetreat) {
        lvl = lvlBest;

        int available = std::max(0, lvlBest.currentFrame() - 1);
        int retreat = std::min(
            available,
            180 + recoveryLevel * 180 + stagnantRounds * 90 + extraRetreat
        );

        lvl.rollback(std::max(1, lvlBest.currentFrame() - retreat));
        lvl.syncPresses();

        recoveryLevel = std::min(10, recoveryLevel + 1);
        stagnantRounds = 0;
        repeatedDeathZone = 0;
        lastDeathX = -100000.f;

        rng.seed(
            baseSeed ^
            static_cast<uint32_t>(recoveryLevel * 0x45d9f3bu) ^
            static_cast<uint32_t>(lvl.currentFrame() * 7919)
        );
    };

    while (!reachedGoal(lvl) && !stop) {
        if (lvl.latestState().dead) {
            recover(240);
            continue;
        }

        auto frame = lvl.currentFrame();
        auto mode = lvl.latestState().vehicle.type;

        int difficulty = std::min(
            5,
            recoveryLevel / 2 + stagnantRounds + repeatedDeathZone
        );

        // Smarter candidate structure lets us look less far ahead and replan more
        // often. This saves a large amount of simulation work on complex levels.
        int horizonFrames =
            (continuousMode(mode) ? 840 : 720) +
            difficulty * (continuousMode(mode) ? 90 : 75);

        auto best = searchBestInputs(lvl, stop, rng, horizonFrames, difficulty);

        if (stop)
            break;

        if (best.trial.complete) {
            int applyUntil = best.trial.frame;
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
        } else {
            if (best.trial.dead) {
                if (std::abs(best.trial.x - lastDeathX) <= 45.f)
                    ++repeatedDeathZone;
                else
                    repeatedDeathZone = 1;
                lastDeathX = best.trial.x;

                int deathSpan = std::max(0, best.trial.frame - frame);
                float deathGain = best.trial.x - lvl.latestState().pos.x;

                // Complex levels commonly have no route that survives the whole
                // forecast: a candidate may clear the current obstacle and die at
                // the next one. Throwing that route away entirely causes the solver
                // to sit at one percentage forever. Commit only an early, verified
                // prefix and replan long before the known death.
                int margin = mode == VehicleType::Wave
                    ? 120
                    : continuousMode(mode) ? 100 : 80;
                int fractionPercent = mode == VehicleType::Wave
                    ? 32
                    : continuousMode(mode) ? 38 : 46;
                int safeSpan = std::min(
                    std::max(0, deathSpan - margin),
                    deathSpan * fractionPercent / 100
                );

                bool usefulPrefix =
                    safeSpan >= 24 &&
                    deathGain > 30.f &&
                    best.trial.frame > frame + margin;

                if (!usefulPrefix) {
                    recover(repeatedDeathZone >= 2 ? 420 : 180);
                    continue;
                }

                // Keep dead-route prefixes small enough that a new search gets a
                // chance to react before the later obstacle that killed the forecast.
                int prefixCap = mode == VehicleType::Wave
                    ? 220
                    : continuousMode(mode) ? 280 : 360;
                safeSpan = std::min(safeSpan, prefixCap);
                int applyUntil = frame + safeSpan;

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

                if (lvl.latestState().dead) {
                    recover(300);
                    continue;
                }
            } else {
                if (best.trial.frame <= frame ||
                    best.trial.x <= lvl.latestState().pos.x + 0.5f) {
                    ++stagnantRounds;
                    if (stagnantRounds >= 2)
                        recover(300);
                    continue;
                }

                int span = std::max(1, best.trial.frame - frame);
                int commitNumerator;

                if (mode == VehicleType::Wave)
                    commitNumerator = best.trial.survivedHorizon ? 28 : 20;
                else if (continuousMode(mode))
                    commitNumerator = best.trial.survivedHorizon ? 38 : 26;
                else if (best.trial.survivedHorizon && best.inputs.empty())
                    commitNumerator = 68;
                else if (best.trial.survivedHorizon)
                    commitNumerator = 56;
                else
                    commitNumerator = 34;

                int committedSpan = std::max(1, span * commitNumerator / 100);
                int applyUntil = frame + committedSpan;

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
        }

        if (!lvl.latestState().completed && lvl.latestState().pos.x > lvl.length + 120.f) {
            recover(420);
            continue;
        }

        if (!lvl.latestState().dead && reachedGoal(lvl)) {
            furthestX = std::max(furthestX, lvl.latestState().pos.x);
            lvlBest = lvl;
            stagnantRounds = 0;
            recoveryLevel = std::max(0, recoveryLevel - 1);
        } else if (!lvl.latestState().dead && lvl.latestState().pos.x > furthestX + 1.f) {
            furthestX = lvl.latestState().pos.x;
            lvlBest = lvl;
            stagnantRounds = 0;
            repeatedDeathZone = 0;
            recoveryLevel = std::max(0, recoveryLevel - 1);
        } else {
            ++stagnantRounds;
        }

        if (stagnantRounds >= 3 && lvlBest.currentFrame() > 1) {
            recover(360);
            continue;
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
