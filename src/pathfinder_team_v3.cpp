#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <random>
#include <set>
#include <sstream>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Orb.hpp>

// Reuse the simulator, candidate generators, scoring helpers and replay export
// code, but replace the old controller with an adaptive cooperative search.
#define pathfind pathfind_parallel_core
#include "pathfinder.cpp"
#undef pathfind

namespace {

enum class TeamRole : int {
    Specialist = 0,
    BruteForce = 1,
    Refinement = 2,
};

char const* roleReason(TeamRole role) {
    switch (role) {
        case TeamRole::Specialist: return "specialist-winner";
        case TeamRole::BruteForce: return "bruteforce-winner";
        case TeamRole::Refinement: return "refinement-winner";
    }
    return "team-winner";
}

struct TeamBatchResult {
    std::set<SearchInput> bestInputs;
    TrialResult bestTrial;
    bool haveBest = false;
    TeamRole bestRole = TeamRole::Specialist;
    float lastDeathX = 0.f;
    uint64_t trials = 0;
    int physicalWorkers = 30;
    int logicalWorkers = 50;
    int candidateCount = 0;
};

struct WorkerBaseline {
    Player p1;
    Player p2;
    bool press1 = false;
    bool press2 = false;
    std::vector<int> moveActivationFrames;
};

struct LocalBest {
    std::set<SearchInput> inputs;
    TrialResult trial;
    bool have = false;
    TeamRole role = TeamRole::Specialist;
    size_t toggles = std::numeric_limits<size_t>::max();
    float lastDeathX = 0.f;
};

struct LiveLeader {
    std::set<SearchInput> inputs;
    TrialResult trial;
    size_t toggles = std::numeric_limits<size_t>::max();
};

struct UpcomingOrb {
    bool found = false;
    OrbType type = OrbType::Yellow;
    float x = 0.f;
    int frameOffset = -1;
};

using LiveBatchCallback = std::function<void(uint64_t, float)>;

std::set<SearchInput> windowedRoute(
    std::set<SearchInput> const& source,
    int frame,
    int horizonFrames
) {
    std::set<SearchInput> result;
    int endFrame = frame + horizonFrames;
    for (SearchInput key : source) {
        int inputFrame = static_cast<int>(key >> 1);
        if (inputFrame >= frame && inputFrame < endFrame)
            result.insert(key);
    }
    return result;
}

// Geometry Dash 2.2 key 121 is NoTouch. The simulator historically treated a
// NoTouch spike/block as real collision geometry, which turns decorated levels
// into imaginary mazes. Keep group target metadata, but remove those objects from
// the physics sections and moving-geometry maps before search begins.
int pruneNoTouchPhysics(Level2& lvl, std::string const& lvlString) {
    std::unordered_set<int> noTouchIDs;
    std::stringstream stream(lvlString);
    std::string objectString;
    bool first = true;
    int supportedIndex = 0;

    while (std::getline(stream, objectString, ';')) {
        if (first) {
            first = false;
            continue;
        }

        ObjectFields fields = parseFields(objectString);
        int objectID = intField(fields, 1);
        if (objectID <= 0 || objectID == 31)
            continue;

        ObjectFields probe = fields;
        if (Object::create(std::move(probe))) {
            if (boolField(fields, 121))
                noTouchIDs.insert(supportedIndex);
            ++supportedIndex;
        }
    }

    if (!noTouchIDs.empty()) {
        for (auto& [_, section] : lvl.sections) {
            section.erase(
                std::remove_if(
                    section.begin(),
                    section.end(),
                    [&](ObjectContainer const& object) {
                        return noTouchIDs.contains(object->id);
                    }
                ),
                section.end()
            );
        }

        for (int id : noTouchIDs) {
            lvl.baseObjectPositions.erase(id);
            lvl.objectSections.erase(id);
            lvl.movingObjectIDs.erase(id);
        }

        for (auto& [_, members] : lvl.groupObjects) {
            members.erase(
                std::remove_if(
                    members.begin(),
                    members.end(),
                    [&](int id) { return noTouchIDs.contains(id); }
                ),
                members.end()
            );
        }

        lvl.highestY = 0.f;
        for (auto const& [_, section] : lvl.sections) {
            for (auto const& object : section)
                lvl.highestY = std::max(lvl.highestY, object->pos.y);
        }
    }

    // Unsupported decoration metadata is useful while parsing, but the hot solver
    // never reads it. Dropping it here prevents deco-heavy levels from carrying a
    // huge dead-weight vector into every compact worker copy.
    lvl.unsupportedObjects.clear();
    lvl.unsupportedObjects.shrink_to_fit();

    return static_cast<int>(noTouchIDs.size());
}

Level2 compactWorkerBase(Level2 const& base) {
    Level2 compact = base;
    Player p1 = base.latestState();
    Player p2 = base.latestState2();
    compact.gameStates.assign(1, p1);
    compact.gameStates2.assign(1, p2);
    compact.gameStates.front().level = &compact;
    compact.gameStates2.front().level = &compact;
    compact.press1 = base.press1;
    compact.press2 = base.press2;
    compact.unsupportedObjects.clear();
    return compact;
}

void prepareWorker(Level2& worker) {
    if (worker.gameStates.empty() || worker.gameStates2.empty())
        return;
    worker.gameStates.front().level = &worker;
    worker.gameStates2.front().level = &worker;
}

WorkerBaseline captureWorkerBaseline(Level2 const& worker) {
    WorkerBaseline baseline;
    baseline.p1 = worker.latestState();
    baseline.p2 = worker.latestState2();
    baseline.press1 = worker.press1;
    baseline.press2 = worker.press2;
    baseline.moveActivationFrames.reserve(worker.moveTriggers.size());
    for (auto const& trigger : worker.moveTriggers)
        baseline.moveActivationFrames.push_back(trigger.activationFrame);
    return baseline;
}

void restoreWorker(Level2& worker, WorkerBaseline const& baseline) {
    worker.gameStates.resize(1);
    worker.gameStates2.resize(1);
    worker.gameStates.front() = baseline.p1;
    worker.gameStates2.front() = baseline.p2;
    worker.gameStates.front().level = &worker;
    worker.gameStates2.front().level = &worker;
    worker.press1 = baseline.press1;
    worker.press2 = baseline.press2;

    size_t count = std::min(worker.moveTriggers.size(), baseline.moveActivationFrames.size());
    for (size_t i = 0; i < count; ++i)
        worker.moveTriggers[i].activationFrame = baseline.moveActivationFrames[i];

    worker.applyMoveGeometry(baseline.p1.frame);
}

TrialResult tryInputsTeam(
    Level2& lvl,
    WorkerBaseline const& baseline,
    std::set<SearchInput> const& inputs,
    int horizonFrames
) {
    int startFrame = baseline.p1.frame;
    float startX = baseline.p1.pos.x;
    int endFrame = startFrame + horizonFrames;
    VehicleType startMode = baseline.p1.vehicle.type;
    int clearanceStride = flightMode(startMode)
        ? 7
        : startMode == VehicleType::Robot || startMode == VehicleType::Spider
            ? 10
            : 14;
    float minClearance = hazardClearance(lvl, lvl.latestState());

    while (lvl.currentFrame() < endFrame &&
           !lvl.latestState().dead &&
           !reachedGoal(lvl)) {
        uint32_t current = static_cast<uint32_t>(lvl.currentFrame());
        if (inputs.contains(inputKey(current, false)))
            lvl.press1 = !lvl.press1;
        if (inputs.contains(inputKey(current, true)))
            lvl.press2 = !lvl.press2;

        lvl.runFrame(lvl.press1, lvl.press2, 1.f / 240.f);

        if (!lvl.latestState().dead &&
            (lvl.currentFrame() - startFrame) % clearanceStride == 0) {
            minClearance = std::min(minClearance, hazardClearance(lvl, lvl.latestState()));
            if (lvl.latestState().dualActive)
                minClearance = std::min(minClearance, hazardClearance(lvl, lvl.latestState2()));
        }
    }

    TrialResult result {
        lvl.currentFrame(),
        lvl.latestState().pos.x,
        lvl.latestState().dead ? lvl.latestState().pos.x : 0.f,
        minClearance,
        lvl.latestState().dead,
        !lvl.latestState().dead && reachedGoal(lvl)
    };

    float y = lvl.latestState().pos.y;
    if (!result.complete &&
        (y > std::max(1300.f, lvl.highestY + 600.f) || y < -600.f)) {
        result.frame = startFrame;
        result.x = startX;
        result.deathX = startX;
        result.dead = true;
    }

    restoreWorker(lvl, baseline);
    return result;
}

bool teamBetterTrial(
    TrialResult const& trial,
    size_t toggleCount,
    bool haveBest,
    TrialResult const& best,
    size_t bestToggleCount,
    VehicleType mode
) {
    if (!haveBest)
        return true;

    if (trial.complete != best.complete)
        return trial.complete;
    if (trial.dead != best.dead)
        return !trial.dead;

    if (trial.dead && best.dead) {
        if (std::abs(trial.x - best.x) > 0.01f)
            return trial.x > best.x;
        if (trial.frame != best.frame)
            return trial.frame > best.frame;
        return toggleCount < bestToggleCount;
    }

    float xDelta = trial.x - best.x;
    if (!flightMode(mode) && std::abs(xDelta) <= 8.f) {
        // Avoid the old click-soup winner problem. On ground modes, if two routes
        // reach effectively the same place, require a real advantage before paying
        // for a pile of extra toggles.
        if (toggleCount + 2 < bestToggleCount)
            return true;
        if (bestToggleCount + 5 < toggleCount)
            return false;
    }

    return betterTrial(
        trial,
        toggleCount,
        true,
        best,
        bestToggleCount,
        mode
    );
}

void considerLocal(
    LocalBest& local,
    std::set<SearchInput> const& candidate,
    TrialResult const& trial,
    TeamRole role,
    VehicleType mode
) {
    if (trial.dead)
        local.lastDeathX = std::max(local.lastDeathX, trial.deathX);

    if (teamBetterTrial(
            trial,
            candidate.size(),
            local.have,
            local.trial,
            local.toggles,
            mode
        )) {
        local.inputs = candidate;
        local.trial = trial;
        local.have = true;
        local.role = role;
        local.toggles = candidate.size();
    }
}

size_t routeDifference(
    std::set<SearchInput> const& a,
    std::set<SearchInput> const& b,
    size_t stopAfter = 12
) {
    size_t difference = 0;
    auto ia = a.begin();
    auto ib = b.begin();
    while (ia != a.end() || ib != b.end()) {
        if (ia == a.end()) {
            difference += static_cast<size_t>(std::distance(ib, b.end()));
            break;
        }
        if (ib == b.end()) {
            difference += static_cast<size_t>(std::distance(ia, a.end()));
            break;
        }
        if (*ia == *ib) {
            ++ia;
            ++ib;
        } else if (*ia < *ib) {
            ++difference;
            ++ia;
        } else {
            ++difference;
            ++ib;
        }
        if (difference > stopAfter)
            break;
    }
    return difference;
}

bool leaderBetter(LiveLeader const& a, LiveLeader const& b, VehicleType mode) {
    if (a.trial.complete != b.trial.complete)
        return a.trial.complete;
    if (a.trial.dead != b.trial.dead)
        return !a.trial.dead;
    if (std::abs(a.trial.x - b.trial.x) > 0.01f)
        return a.trial.x > b.trial.x;
    float clearanceBand = flightMode(mode) ? 1.5f : 0.75f;
    if (std::abs(a.trial.minClearance - b.trial.minClearance) > clearanceBand)
        return a.trial.minClearance > b.trial.minClearance;
    if (a.trial.frame != b.trial.frame)
        return a.trial.frame > b.trial.frame;
    return a.toggles < b.toggles;
}

bool normalJumpOrb(OrbType type) {
    return type == OrbType::Yellow ||
           type == OrbType::Blue ||
           type == OrbType::Pink ||
           type == OrbType::Red ||
           type == OrbType::Green ||
           type == OrbType::Black;
}

int estimateFrameOffsetForX(Level2 const& base, float targetX, int horizonFrames) {
    Player const& player = base.latestState();
    if (player.direction <= 0)
        return -1;

    int speedIndex = std::clamp(player.speed, 0, 4);
    double timeScale = std::clamp(static_cast<double>(player.timeWarp), 0.05, 4.0);
    double xPerSecond = player_speeds[speedIndex] * timeScale;
    if (xPerSecond < 1.0)
        return -1;

    float dx = targetX - player.pos.x;
    if (dx < -20.f)
        return -1;

    int offset = static_cast<int>(std::lround(
        static_cast<double>(dx) * 240.0 / xPerSecond
    ));
    return offset >= 0 && offset < horizonFrames ? offset : -1;
}

UpcomingOrb findUpcomingOrb(Level2 const& base, int horizonFrames) {
    UpcomingOrb result;
    Player const& player = base.latestState();
    if (player.direction <= 0 || player.dashing || flightMode(player.vehicle.type))
        return result;

    int speedIndex = std::clamp(player.speed, 0, 4);
    double timeScale = std::clamp(static_cast<double>(player.timeWarp), 0.05, 4.0);
    double xPerSecond = player_speeds[speedIndex] * timeScale;
    if (xPerSecond < 1.0)
        return result;

    float travelX = static_cast<float>(xPerSecond * horizonFrames / 240.0);
    float lowX = player.pos.x - 35.f;
    float highX = player.pos.x + travelX + 50.f;
    int firstSection = static_cast<int>(std::floor(lowX / Level::sectionSize));
    int lastSection = static_cast<int>(std::floor(highX / Level::sectionSize));
    float nearestDx = std::numeric_limits<float>::max();

    for (int section = firstSection; section <= lastSection; ++section) {
        auto it = base.sections.find(section);
        if (it == base.sections.end())
            continue;

        for (auto const& object : it->second) {
            auto const* orb = dynamic_cast<Orb const*>(object.operator->());
            if (orb == nullptr || !normalJumpOrb(orb->type))
                continue;

            float dx = orb->pos.x - player.pos.x;
            if (dx < -35.f || dx > travelX + 50.f)
                continue;
            if (dx < nearestDx) {
                nearestDx = dx;
                result.found = true;
                result.type = orb->type;
                result.x = orb->pos.x;
            }
        }
    }

    if (result.found)
        result.frameOffset = estimateFrameOffsetForX(base, result.x, horizonFrames);
    return result;
}

void eraseP1Window(
    std::set<SearchInput>& route,
    int frame,
    int startOffset,
    int endOffset
) {
    int start = frame + std::max(0, startOffset);
    int end = frame + std::max(startOffset, endOffset);
    for (auto it = route.begin(); it != route.end();) {
        int f = static_cast<int>(*it >> 1);
        bool p2 = (*it & 1u) != 0;
        if (!p2 && f >= start && f <= end)
            it = route.erase(it);
        else
            ++it;
    }
}

bool heldBeforeOffset(
    std::set<SearchInput> const& route,
    int frame,
    int offset,
    bool initialHeld
) {
    bool held = initialHeld;
    int limit = frame + offset;
    for (SearchInput key : route) {
        if ((key & 1u) != 0)
            continue;
        int f = static_cast<int>(key >> 1);
        if (f >= limit)
            break;
        if (f >= frame)
            held = !held;
    }
    return held;
}

void forceFreshPulse(
    std::set<SearchInput>& route,
    Level2 const& base,
    int horizonFrames,
    int pressAt,
    int width
) {
    if (pressAt < 0 || pressAt >= horizonFrames)
        return;

    int frame = base.currentFrame();
    int clearStart = std::max(0, pressAt - 6);
    int clearEnd = std::min(horizonFrames - 1, pressAt + std::max(width, 8) + 3);
    eraseP1Window(route, frame, clearStart, clearEnd);

    bool held = heldBeforeOffset(route, frame, clearStart, base.press1);
    if (held)
        addToggle(route, frame, horizonFrames, clearStart, false);

    addToggle(route, frame, horizonFrames, pressAt, false);
    if (width > 0 && pressAt + width < horizonFrames)
        addToggle(route, frame, horizonFrames, pressAt + width, false);
}

void appendUpcomingOrbCandidates(
    Level2 const& base,
    int horizonFrames,
    std::set<SearchInput> const& seedLeader,
    UpcomingOrb const& orb,
    std::vector<std::set<SearchInput>>& candidates
) {
    if (!orb.found || orb.frameOffset < 0)
        return;

    static constexpr std::array<int, 17> coarseDeltas = {
        -20, -14, -10, -8, -6, -4, -2, -1, 0,
        1, 2, 4, 6, 8, 10, 14, 20
    };
    static constexpr std::array<int, 25> blueDeltas = {
        -24, -20, -16, -14, -12, -10, -8, -6, -4, -3, -2, -1, 0,
        1, 2, 3, 4, 6, 8, 10, 12, 14, 16, 20, 24
    };
    static constexpr std::array<int, 7> blueWidths = {1, 2, 4, 8, 16, 28, 48};

    auto addFamily = [&](int delta, int width, bool useSeed) {
        std::set<SearchInput> route = useSeed
            ? windowedRoute(seedLeader, base.currentFrame(), horizonFrames)
            : std::set<SearchInput>{};
        forceFreshPulse(route, base, horizonFrames, orb.frameOffset + delta, width);
        candidates.push_back(std::move(route));
    };

    if (orb.type == OrbType::Blue) {
        int index = 0;
        for (int delta : blueDeltas) {
            for (int width : blueWidths)
                addFamily(delta, width, true);
            if ((index++ % 4) == 0) {
                for (int width : {1, 4, 16})
                    addFamily(delta, width, false);
            }
        }
    } else {
        int index = 0;
        for (int delta : coarseDeltas) {
            for (int width : {1, 2, 4, 8, 20})
                addFamily(delta, width, true);
            if ((index++ % 5) == 0)
                addFamily(delta, 2, false);
        }
    }
}

void appendFailureRecoveryCandidates(
    Level2 const& base,
    int horizonFrames,
    std::set<SearchInput> const& seedLeader,
    float failureX,
    int repeatCount,
    std::vector<std::set<SearchInput>>& candidates
) {
    if (repeatCount < 2 || failureX <= base.latestState().pos.x + 3.f)
        return;

    int center = estimateFrameOffsetForX(base, failureX, horizonFrames);
    if (center < 0)
        return;

    VehicleType mode = base.latestState().vehicle.type;
    static constexpr std::array<int, 19> deltas = {
        -36, -24, -18, -14, -10, -8, -6, -4, -2,
        0, 2, 4, 6, 8, 10, 14, 18, 24, 36
    };

    if (flightMode(mode)) {
        for (int delta : deltas) {
            for (int width : {4, 8, 12, 18, 24, 36, 48}) {
                int start = center + delta;
                if (start < 0 || start + width >= horizonFrames)
                    continue;
                auto route = windowedRoute(seedLeader, base.currentFrame(), horizonFrames);
                addToggle(route, base.currentFrame(), horizonFrames, start, false);
                addToggle(route, base.currentFrame(), horizonFrames, start + width, false);
                candidates.push_back(std::move(route));
            }
        }
        return;
    }

    int index = 0;
    for (int delta : deltas) {
        for (int width : {1, 2, 4, 8, 16, 28, 48}) {
            auto route = windowedRoute(seedLeader, base.currentFrame(), horizonFrames);
            forceFreshPulse(route, base, horizonFrames, center + delta, width);
            candidates.push_back(std::move(route));
        }
        if ((index++ % 4) == 0) {
            std::set<SearchInput> clean;
            forceFreshPulse(clean, base, horizonFrames, center + delta, 2);
            candidates.push_back(std::move(clean));
        }
    }
}

void dedupeCandidates(std::vector<std::set<SearchInput>>& candidates) {
    std::unordered_set<uint64_t> seen;
    std::vector<std::set<SearchInput>> unique;
    unique.reserve(candidates.size());

    for (auto& candidate : candidates) {
        uint64_t hash = 1469598103934665603ull;
        for (SearchInput key : candidate) {
            hash ^= static_cast<uint64_t>(key) + 0x9e3779b97f4a7c15ull;
            hash *= 1099511628211ull;
        }
        hash ^= static_cast<uint64_t>(candidate.size()) * 0x517cc1b727220a95ull;
        if (seen.insert(hash).second)
            unique.push_back(std::move(candidate));
    }
    candidates = std::move(unique);
}

std::set<SearchInput> makeBruteCandidate(
    Level2 const& base,
    int horizonFrames,
    int logicalLane,
    int attempt,
    int focusOffset,
    std::mt19937& rng
) {
    std::set<SearchInput> route;
    int frame = base.currentFrame();
    VehicleType mode = base.latestState().vehicle.type;
    bool dual = base.latestState().dualActive;

    int focusRadius = focusOffset >= 0 ? 110 + (logicalLane % 5) * 25 : horizonFrames;
    auto randomOffset = [&](int salt) {
        if (focusOffset >= 0 && ((salt + attempt + logicalLane) % 5) != 0) {
            int low = std::max(0, focusOffset - focusRadius);
            int high = std::min(horizonFrames - 1, focusOffset + focusRadius);
            std::uniform_int_distribution<int> dist(low, std::max(low, high));
            return dist(rng);
        }
        std::uniform_int_distribution<int> dist(0, std::max(0, horizonFrames - 1));
        return dist(rng);
    };

    if (!flightMode(mode)) {
        int pulseCount = 1 + ((logicalLane + attempt) % 3);
        for (int p = 0; p < pulseCount; ++p) {
            int start = randomOffset(p);
            int width = mode == VehicleType::Robot
                ? 1 + ((logicalLane * 5 + attempt * 3 + p) % 32)
                : mode == VehicleType::Spider
                    ? 1 + ((logicalLane + attempt + p) % 2 == 0
                        ? (logicalLane % 5) + 1
                        : 24 + ((logicalLane + p) % 6) * 18)
                    : 1 + ((logicalLane * 7 + attempt * 5 + p) % 28);
            forceFreshPulse(route, base, horizonFrames, start, width);
        }
    } else {
        int toggles = 3 + ((logicalLane * 3 + attempt) % 9);
        for (int i = 0; i < toggles; ++i)
            addToggle(route, frame, horizonFrames, randomOffset(i), false);
    }

    if (dual && ((logicalLane + attempt) % 3 == 0)) {
        int p2Toggles = 1 + ((logicalLane + attempt) % 4);
        for (int i = 0; i < p2Toggles; ++i)
            addToggle(route, frame, horizonFrames, randomOffset(i + 31), true);
    }

    return route;
}

std::set<SearchInput> mutateLeaderCandidate(
    Level2 const& base,
    std::set<SearchInput> const& rawLeader,
    int horizonFrames,
    int logicalLane,
    int attempt,
    int focusOffset,
    std::mt19937& rng
) {
    int frame = base.currentFrame();
    VehicleType mode = base.latestState().vehicle.type;
    bool dual = base.latestState().dualActive;
    std::set<SearchInput> route = windowedRoute(rawLeader, frame, horizonFrames);

    if (route.empty()) {
        int start = focusOffset >= 0
            ? std::clamp(focusOffset + ((logicalLane % 9) - 4) * 3, 0, horizonFrames - 1)
            : (logicalLane * 37 + attempt * 17) % std::max(1, std::min(horizonFrames, 480));
        int width = mode == VehicleType::Robot
            ? 1 + ((logicalLane * 5 + attempt * 3) % 32)
            : mode == VehicleType::Spider
                ? 1 + ((logicalLane + attempt) % 2 == 0
                    ? (logicalLane % 6) + 1
                    : 30 + (logicalLane % 5) * 24)
                : 2 + ((logicalLane * 11 + attempt * 7) % 48);
        forceFreshPulse(route, base, horizonFrames, start, width);
        return route;
    }

    std::vector<SearchInput> keys(route.begin(), route.end());
    std::uniform_int_distribution<size_t> pick(0, keys.size() - 1);
    SearchInput chosen = keys[pick(rng)];
    int chosenFrame = static_cast<int>(chosen >> 1);
    bool chosenP2 = (chosen & 1u) != 0;
    int strategy = (logicalLane + attempt) % 7;

    if (strategy == 0) {
        static constexpr int deltas[] = {-16, -8, -4, -2, -1, 1, 2, 4, 8, 16};
        int delta = deltas[(logicalLane * 3 + attempt) % 10];
        int shifted = std::clamp(chosenFrame + delta, frame, frame + horizonFrames - 1);
        route.erase(chosen);
        route.insert(inputKey(static_cast<uint32_t>(shifted), chosenP2));
    } else if (strategy == 1) {
        route.erase(chosen);
    } else if (strategy == 2) {
        int start = focusOffset >= 0
            ? std::clamp(focusOffset + ((logicalLane % 11) - 5) * 2, 0, horizonFrames - 1)
            : (logicalLane * 29 + attempt * 13) % std::max(1, std::min(horizonFrames, 420));
        int width = 1 + ((logicalLane * 7 + attempt) % (mode == VehicleType::Robot ? 32 : 48));
        forceFreshPulse(route, base, horizonFrames, start, width);
    } else if (strategy == 3) {
        int delta = ((logicalLane + attempt) % 11) - 5;
        if (delta == 0)
            delta = logicalLane % 2 == 0 ? -1 : 1;
        std::set<SearchInput> shiftedRoute;
        for (SearchInput key : route) {
            int f = static_cast<int>(key >> 1) + delta;
            bool p2 = (key & 1u) != 0;
            if (f >= frame && f < frame + horizonFrames)
                shiftedRoute.insert(inputKey(static_cast<uint32_t>(f), p2));
        }
        route = std::move(shiftedRoute);
    } else if (strategy == 4) {
        int center = focusOffset >= 0 ? focusOffset : horizonFrames / 3;
        eraseP1Window(route, frame, center - 18, center + 18);
    } else if (strategy == 5 && dual) {
        route.insert(inputKey(static_cast<uint32_t>(chosenFrame), !chosenP2));
    } else {
        int low = focusOffset >= 0 ? std::max(0, focusOffset - 80) : 0;
        int high = focusOffset >= 0
            ? std::min(horizonFrames - 1, focusOffset + 80)
            : horizonFrames - 1;
        std::uniform_int_distribution<int> dist(low, std::max(low, high));
        addToggle(route, frame, horizonFrames, dist(rng), false);
    }

    return route;
}

TeamBatchResult evaluateTeamBatch(
    Level2 const& base,
    std::vector<std::set<SearchInput>> const& specialists,
    std::set<SearchInput> const& seedLeader,
    int horizonFrames,
    int searchLevel,
    int focusOffset,
    bool focused,
    uint32_t baseSeed,
    std::atomic_bool& stop,
    LiveBatchCallback liveCallback
) {
    TeamBatchResult out;
    int frame = base.currentFrame();
    VehicleType mode = base.latestState().vehicle.type;
    out.bestTrial = {
        frame,
        base.latestState().pos.x,
        0.f,
        0.f,
        false,
        false
    };

    constexpr int kRoleThreads = 10;
    constexpr int kTotalThreads = 30;
    constexpr int kLogicalWorkers = 50;
    int generatedAttempts = focused
        ? std::clamp(16 + searchLevel * 2, 16, 34)
        : std::clamp(12 + searchLevel * 2, 12, 30);
    out.candidateCount = static_cast<int>(
        specialists.size() + generatedAttempts * kRoleThreads * 2
    );

    Level2 compactBase = compactWorkerBase(base);
    std::atomic_bool batchSolved = false;
    std::atomic_bool batchSatisfied = false;
    std::atomic<uint64_t> trials = 0;
    std::atomic<uint64_t> nextLiveReport = 24;
    std::atomic<float> liveBestX = base.latestState().pos.x;

    std::mutex bestMutex;
    size_t bestToggleCount = std::numeric_limits<size_t>::max();
    std::mutex reportMutex;

    std::mutex leaderMutex;
    std::vector<LiveLeader> liveLeaders;
    std::atomic<uint64_t> leaderVersion = 0;

    auto publishLeader = [&](std::set<SearchInput> const& candidate, TrialResult const& trial) {
        if (trial.dead)
            return;

        std::lock_guard<std::mutex> guard(leaderMutex);
        LiveLeader incoming {candidate, trial, candidate.size()};
        bool changed = false;

        for (auto& leader : liveLeaders) {
            if (routeDifference(candidate, leader.inputs, 6) <= 3) {
                if (leaderBetter(incoming, leader, mode)) {
                    leader = std::move(incoming);
                    changed = true;
                }
                if (changed)
                    leaderVersion.fetch_add(1, std::memory_order_release);
                return;
            }
        }

        liveLeaders.push_back(std::move(incoming));
        std::sort(
            liveLeaders.begin(),
            liveLeaders.end(),
            [&](LiveLeader const& a, LiveLeader const& b) {
                return leaderBetter(a, b, mode);
            }
        );
        if (liveLeaders.size() > 4)
            liveLeaders.resize(4);
        changed = true;

        float bestX = base.latestState().pos.x;
        for (auto const& leader : liveLeaders)
            bestX = std::max(bestX, leader.trial.x);
        liveBestX.store(bestX, std::memory_order_relaxed);
        if (changed)
            leaderVersion.fetch_add(1, std::memory_order_release);
    };

    auto recordTrial = [&](TrialResult const& trial) {
        uint64_t n = trials.fetch_add(1, std::memory_order_relaxed) + 1;
        uint64_t target = nextLiveReport.load(std::memory_order_relaxed);
        if (liveCallback && n >= target &&
            nextLiveReport.compare_exchange_strong(
                target,
                n + 24,
                std::memory_order_relaxed
            )) {
            std::lock_guard<std::mutex> guard(reportMutex);
            liveCallback(n, liveBestX.load(std::memory_order_relaxed));
        }

        int minimum = focused
            ? 96
            : flightMode(mode) ? 220 : 160;
        if (!trial.dead &&
            trial.frame >= frame + horizonFrames &&
            n >= static_cast<uint64_t>(minimum)) {
            batchSatisfied.store(true, std::memory_order_release);
        }
        if (trial.complete) {
            batchSolved.store(true, std::memory_order_release);
            batchSatisfied.store(true, std::memory_order_release);
        }
        return n;
    };

    auto mergeLocal = [&](LocalBest&& local) {
        std::lock_guard<std::mutex> guard(bestMutex);
        out.lastDeathX = std::max(out.lastDeathX, local.lastDeathX);
        if (!local.have)
            return;
        if (teamBetterTrial(
                local.trial,
                local.toggles,
                out.haveBest,
                out.bestTrial,
                bestToggleCount,
                mode
            )) {
            out.bestInputs = std::move(local.inputs);
            out.bestTrial = local.trial;
            out.haveBest = true;
            out.bestRole = local.role;
            bestToggleCount = local.toggles;
        }
    };

    std::vector<std::thread> threads;
    threads.reserve(kTotalThreads);

    // 1-10: deterministic/mode-aware specialists, including orb and failure rescue routes.
    for (int lane = 0; lane < kRoleThreads; ++lane) {
        threads.emplace_back([&, lane] {
            Level2 worker = compactBase;
            prepareWorker(worker);
            WorkerBaseline baseline = captureWorkerBaseline(worker);
            LocalBest local;

            for (size_t index = static_cast<size_t>(lane);
                 index < specialists.size() &&
                 !stop.load() &&
                 !batchSolved.load(std::memory_order_acquire) &&
                 !batchSatisfied.load(std::memory_order_acquire);
                 index += kRoleThreads) {
                TrialResult trial = tryInputsTeam(worker, baseline, specialists[index], horizonFrames);
                considerLocal(local, specialists[index], trial, TeamRole::Specialist, mode);
                publishLeader(specialists[index], trial);
                recordTrial(trial);
            }
            mergeLocal(std::move(local));
        });
    }

    // 11-20: exploration. Twenty logical lanes are multiplexed over ten physical threads.
    for (int lane = 0; lane < kRoleThreads; ++lane) {
        threads.emplace_back([&, lane] {
            Level2 worker = compactBase;
            prepareWorker(worker);
            WorkerBaseline baseline = captureWorkerBaseline(worker);
            LocalBest local;
            std::mt19937 localRng(
                baseSeed ^
                static_cast<uint32_t>((frame + 1) * 2654435761u) ^
                static_cast<uint32_t>((lane + 11) * 104729u)
            );

            for (int attempt = 0;
                 attempt < generatedAttempts &&
                 !stop.load() &&
                 !batchSolved.load(std::memory_order_acquire) &&
                 !batchSatisfied.load(std::memory_order_acquire);
                 ++attempt) {
                int logicalLane = lane + ((attempt & 1) ? kRoleThreads : 0);
                auto candidate = makeBruteCandidate(
                    base,
                    horizonFrames,
                    logicalLane,
                    attempt,
                    focusOffset,
                    localRng
                );
                TrialResult trial = tryInputsTeam(worker, baseline, candidate, horizonFrames);
                considerLocal(local, candidate, trial, TeamRole::BruteForce, mode);
                publishLeader(candidate, trial);
                recordTrial(trial);
            }
            mergeLocal(std::move(local));
        });
    }

    // 21-30: refinement. These workers rotate through up to four live leaders instead
    // of dogpiling one champion, so a locally-good route cannot erase all diversity.
    for (int lane = 0; lane < kRoleThreads; ++lane) {
        threads.emplace_back([&, lane] {
            Level2 worker = compactBase;
            prepareWorker(worker);
            WorkerBaseline baseline = captureWorkerBaseline(worker);
            LocalBest local;
            std::mt19937 localRng(
                baseSeed ^
                static_cast<uint32_t>((frame + 17) * 2246822519u) ^
                static_cast<uint32_t>((lane + 21) * 3266489917u)
            );

            uint64_t cachedVersion = std::numeric_limits<uint64_t>::max();
            std::vector<std::set<SearchInput>> cachedLeaders;

            for (int attempt = 0;
                 attempt < generatedAttempts &&
                 !stop.load() &&
                 !batchSolved.load(std::memory_order_acquire) &&
                 !batchSatisfied.load(std::memory_order_acquire);
                 ++attempt) {
                uint64_t version = leaderVersion.load(std::memory_order_acquire);
                if (version != cachedVersion) {
                    std::lock_guard<std::mutex> guard(leaderMutex);
                    cachedLeaders.clear();
                    cachedLeaders.reserve(liveLeaders.size());
                    for (auto const& leader : liveLeaders)
                        cachedLeaders.push_back(leader.inputs);
                    cachedVersion = leaderVersion.load(std::memory_order_relaxed);
                }

                std::set<SearchInput> leader;
                if (!cachedLeaders.empty()) {
                    size_t index = static_cast<size_t>(lane + attempt) % cachedLeaders.size();
                    leader = cachedLeaders[index];
                } else if (!seedLeader.empty()) {
                    leader = seedLeader;
                } else if (!specialists.empty()) {
                    size_t index = static_cast<size_t>(
                        lane + attempt * kRoleThreads
                    ) % specialists.size();
                    leader = specialists[index];
                }

                int logicalLane = lane + ((attempt & 1) ? kRoleThreads : 0);
                auto candidate = mutateLeaderCandidate(
                    base,
                    leader,
                    horizonFrames,
                    logicalLane,
                    attempt,
                    focusOffset,
                    localRng
                );
                TrialResult trial = tryInputsTeam(worker, baseline, candidate, horizonFrames);
                considerLocal(local, candidate, trial, TeamRole::Refinement, mode);
                publishLeader(candidate, trial);
                recordTrial(trial);
            }
            mergeLocal(std::move(local));
        });
    }

    for (auto& thread : threads)
        thread.join();

    out.trials = trials.load(std::memory_order_relaxed);
    if (liveCallback)
        liveCallback(out.trials, liveBestX.load(std::memory_order_relaxed));
    return out;
}

} // namespace

PathfinderResult pathfind(
    std::string const& lvlString,
    std::atomic_bool& stop,
    std::function<void(PathfinderTelemetry const&)> callback,
    float trustedEndX
) {
    Level2 lvl(lvlString);
    int prunedNoTouch = pruneNoTouchPhysics(lvl, lvlString);

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

    int trueBestFrame = lvl.currentFrame();
    int fail = 1;
    int numAway = 1000;
    int stagnantRounds = 0;
    int recoveryCount = 0;
    int fullRestarts = 0;
    int hardestSearchLevel = 0;
    int recoveredExceptions = 0;
    int deadCandidatesRejected = 0;
    int specialistWins = 0;
    int bruteWins = 0;
    int refinementWins = 0;
    int deathRepeat = 0;
    int focusedRounds = 0;
    int debugSearchLevel = 0;
    int debugHorizon = 0;
    int debugCandidateCount = 0;
    int debugHelpers = 50;
    int debugPhysicalWorkers = 30;
    int debugVehicleType = static_cast<int>(lvl.latestState().vehicle.type);
    uint64_t totalTrials = 0;

    float furthestX = lvl.latestState().pos.x;
    float lastDeathX = 0.f;
    float deathClusterX = 0.f;
    float debugClearance = 0.f;
    Timeline bestPlayable(lvl);
    std::set<SearchInput> persistentLeader;

    auto makeTelemetry = [&](int phase, char const* reason) {
        PathfinderTelemetry telemetry;
        telemetry.progress = progressFor(furthestX, solveStartX, lvl.length, reachedGoal(lvl));
        telemetry.startX = solveStartX;
        telemetry.currentX = lvl.latestState().pos.x;
        telemetry.furthestX = furthestX;
        telemetry.trustedEndX = hasTrustedEnd ? trustedEndX : 0.f;
        telemetry.inferredLength = simulatorLength;
        telemetry.checkpointX = bestPlayable.x();
        telemetry.deathX = lastDeathX;
        telemetry.deathProgress = static_cast<float>(
            progressFor(lastDeathX, solveStartX, lvl.length, false)
        );
        telemetry.bestClearance = debugClearance;
        telemetry.frame = lvl.currentFrame();
        telemetry.checkpointFrame = bestPlayable.frame();
        telemetry.vehicleType = debugVehicleType;
        telemetry.searchLevel = debugSearchLevel;
        telemetry.horizonFrames = debugHorizon;
        telemetry.candidateCount = debugCandidateCount;
        // The UI label says workers. In v3 this intentionally reports logical helpers;
        // diagnostics still expose the 30 physical search threads separately.
        telemetry.workerCount = debugHelpers;
        telemetry.phase = phase;
        telemetry.totalTrials = totalTrials;
        telemetry.mode = "adaptive-team50-30thread-v3";
        telemetry.recoveryReason = reason;
        return telemetry;
    };

    auto emitTelemetry = [&](int phase, char const* reason) {
        if (callback)
            callback(makeTelemetry(phase, reason));
    };

    auto recoverFromBest = [&](int retreatFrames) {
        bestPlayable.restore(lvl);
        int target = std::max(1, bestPlayable.frame() - retreatFrames);
        lvl.rollback(target);
        lvl.syncPresses();
        persistentLeader.clear();
        ++recoveryCount;
    };

    while (!reachedGoal(lvl) && !stop.load()) {
        try {
            if (lvl.latestState().dead) {
                recoverFromBest(std::min(3000, 240 + recoveryCount * 220));
                emitTelemetry(2, "recover-dead-state");
                continue;
            }

            int frame = lvl.currentFrame();
            VehicleType mode = lvl.latestState().vehicle.type;
            int searchLevel = std::min(
                14,
                recoveryCount + stagnantRounds / 3 + deathRepeat / 2
            );
            hardestSearchLevel = std::max(hardestSearchLevel, searchLevel);

            int horizonFrames = 680 + searchLevel * 70;
            if (flightMode(mode))
                horizonFrames += 260;
            else if (mode == VehicleType::Robot)
                horizonFrames += 120;
            else if (mode == VehicleType::Spider)
                horizonFrames = std::min(horizonFrames, 1200);
            horizonFrames = std::min(horizonFrames, 1900);

            UpcomingOrb orb = findUpcomingOrb(lvl, horizonFrames);
            int failureOffset = estimateFrameOffsetForX(lvl, lastDeathX, horizonFrames);
            int focusOffset = -1;

            if (orb.found && orb.frameOffset >= 0 && orb.frameOffset <= 300)
                focusOffset = orb.frameOffset;
            if (deathRepeat >= 2 && failureOffset >= 0 && failureOffset <= 420) {
                if (focusOffset < 0 || failureOffset < focusOffset)
                    focusOffset = failureOffset;
            }

            bool focused = focusOffset >= 0;
            if (focused) {
                ++focusedRounds;
                int focusedTail = flightMode(mode) ? 300 : 180;
                int minimum = flightMode(mode) ? 360 : 220;
                int maximum = flightMode(mode) ? 760 : 600;
                horizonFrames = std::clamp(
                    focusOffset + focusedTail,
                    minimum,
                    std::min(maximum, horizonFrames)
                );
                orb = findUpcomingOrb(lvl, horizonFrames);
            }

            bool dual = lvl.latestState().dualActive;
            auto specialists = structuredCandidates(
                frame,
                horizonFrames,
                mode,
                dual,
                lvl.press1,
                lvl.latestState().dashing
            );
            appendUpcomingOrbCandidates(
                lvl,
                horizonFrames,
                persistentLeader,
                orb,
                specialists
            );
            appendFailureRecoveryCandidates(
                lvl,
                horizonFrames,
                persistentLeader,
                lastDeathX,
                deathRepeat,
                specialists
            );
            dedupeCandidates(specialists);

            int generatedAttempts = focused
                ? std::clamp(16 + searchLevel * 2, 16, 34)
                : std::clamp(12 + searchLevel * 2, 12, 30);
            debugVehicleType = static_cast<int>(mode);
            debugSearchLevel = searchLevel;
            debugHorizon = horizonFrames;
            debugHelpers = 50;
            debugPhysicalWorkers = 30;
            debugCandidateCount = static_cast<int>(
                specialists.size() + generatedAttempts * 20
            );
            debugClearance = 0.f;
            emitTelemetry(0, focused ? "focused-search" : "adaptive-search");

            auto liveReport = [&](uint64_t batchTrials, float liveX) {
                if (!callback)
                    return;
                PathfinderTelemetry telemetry = makeTelemetry(1, focused ? "live-focused" : "live-search");
                telemetry.currentX = std::max(telemetry.currentX, liveX);
                telemetry.totalTrials = totalTrials + batchTrials;
                callback(telemetry);
            };

            TeamBatchResult team = evaluateTeamBatch(
                lvl,
                specialists,
                persistentLeader,
                horizonFrames,
                searchLevel,
                focusOffset,
                focused,
                baseSeed ^
                    static_cast<uint32_t>(recoveryCount * 7919u) ^
                    static_cast<uint32_t>(frame * 104729u),
                stop,
                liveReport
            );

            totalTrials += team.trials;
            debugHelpers = team.logicalWorkers;
            debugPhysicalWorkers = team.physicalWorkers;
            debugCandidateCount = team.candidateCount;

            if (team.lastDeathX > 0.f) {
                lastDeathX = team.lastDeathX;
                if (deathRepeat > 0 && std::abs(lastDeathX - deathClusterX) <= 45.f) {
                    ++deathRepeat;
                    deathClusterX = (deathClusterX * 0.7f) + (lastDeathX * 0.3f);
                } else {
                    deathRepeat = 1;
                    deathClusterX = lastDeathX;
                }
            }

            if (stop.load())
                break;

            std::set<SearchInput> bestInputs = team.bestInputs;
            TrialResult bestTrial = team.bestTrial;
            bool haveBest = team.haveBest;

            if (haveBest && !bestTrial.dead)
                persistentLeader = bestInputs;
            else
                persistentLeader.clear();

            if (haveBest) {
                switch (team.bestRole) {
                    case TeamRole::Specialist: ++specialistWins; break;
                    case TeamRole::BruteForce: ++bruteWins; break;
                    case TeamRole::Refinement: ++refinementWins; break;
                }
            }

            debugClearance = haveBest ? bestTrial.minClearance : 0.f;
            emitTelemetry(
                1,
                haveBest ? roleReason(team.bestRole) : "team-no-candidate"
            );

            if (!haveBest || bestTrial.frame <= frame) {
                ++recoveryCount;
                persistentLeader.clear();
                int preferred = std::max(frame - fail, trueBestFrame - numAway);
                int target = std::clamp(preferred, 1, std::max(1, frame - 1));
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
                        persistentLeader.clear();
                        ++fullRestarts;
                    }
                } else if (fail > 100) {
                    fail += 50;
                }
                emitTelemetry(2, "rollback-no-advance");
                continue;
            }

            int applyUntil = frame;
            if (bestTrial.complete) {
                applyUntil = bestTrial.frame;
            } else if (!bestTrial.dead) {
                int advance = bestTrial.frame - frame;
                int safetyDivisor = focused
                    ? 9
                    : flightMode(mode) ||
                        mode == VehicleType::Robot ||
                        mode == VehicleType::Spider
                        ? 4 : 5;
                int safetyTail = focused
                    ? std::max(5, advance / safetyDivisor)
                    : std::max(12, advance / safetyDivisor);
                applyUntil = std::max(frame + 1, bestTrial.frame - safetyTail);
            } else {
                int deathBuffer = std::min(280, 90 + searchLevel * 12);
                int safeEnd = std::max(frame, bestTrial.frame - deathBuffer);
                int safeAdvance = safeEnd - frame;
                applyUntil = frame + safeAdvance / 2;
                ++deadCandidatesRejected;
                persistentLeader.clear();
            }

            if (applyUntil <= frame) {
                recoverFromBest(std::min(3000, 220 + recoveryCount * 220));
                emitTelemetry(2, "recover-zero-prefix");
                continue;
            }

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

            if (lvl.latestState().dead) {
                lastDeathX = lvl.latestState().pos.x;
                if (deathRepeat > 0 && std::abs(lastDeathX - deathClusterX) <= 45.f)
                    ++deathRepeat;
                else {
                    deathRepeat = 1;
                    deathClusterX = lastDeathX;
                }
                recoverFromBest(std::min(3200, 300 + recoveryCount * 220));
                emitTelemetry(2, "recover-applied-death");
                continue;
            }

            if (lvl.currentFrame() > trueBestFrame) {
                trueBestFrame = lvl.currentFrame();
                fail = 0;
                numAway = 1000;
            }

            bool advancedX = lvl.latestState().pos.x > furthestX + 1.f;
            if (advancedX || reachedGoal(lvl)) {
                furthestX = std::max(furthestX, lvl.latestState().pos.x);
                bestPlayable.capture(lvl);
                stagnantRounds = 0;
                recoveryCount = std::max(0, recoveryCount - 2);
                if (furthestX > deathClusterX + 90.f)
                    deathRepeat = 0;
                emitTelemetry(3, reachedGoal(lvl) ? "complete" : "advance");
            } else if (lvl.latestState().direction < 0) {
                stagnantRounds = 0;
            } else {
                ++stagnantRounds;
            }

            int stagnationLimit = focused || deathRepeat >= 3 ? 4 : 7;
            if (stagnantRounds >= stagnationLimit &&
                bestPlayable.frame() > 2 &&
                lvl.latestState().direction >= 0) {
                int retreat = std::min(
                    bestPlayable.frame() - 1,
                    240 + deathRepeat * 120 + std::min(recoveryCount, 10) * 180
                );
                recoverFromBest(retreat);
                stagnantRounds = 0;
                fail = 1;
                numAway = std::min(8000, 1000 + recoveryCount * 500);
                emitTelemetry(2, "self-correct-stagnation");
            }
        } catch (std::exception const&) {
            ++recoveredExceptions;
            recoverFromBest(std::min(3600, 420 + recoveredExceptions * 220));
            emitTelemetry(2, "recover-exception");
        } catch (...) {
            ++recoveredExceptions;
            recoverFromBest(std::min(3600, 420 + recoveredExceptions * 220));
            emitTelemetry(2, "recover-unknown-exception");
        }
    }

    if (reachedGoal(lvl) && !lvl.latestState().dead) {
        furthestX = std::max(furthestX, lvl.latestState().pos.x);
        bestPlayable.capture(lvl);
    }

    PathfinderResult result;
    Replay2 output;

    for (size_t i = 1; i < bestPlayable.p1.size(); ++i) {
        auto const& p1 = bestPlayable.p1[i];
        auto const& previousP1 = bestPlayable.p1[i - 1];

        if (p1.frame > 1 && p1.button != previousP1.button) {
            output.inputs.push_back(gdr::Input(p1.frame, 1, false, p1.button));
            result.inputs.push_back({
                static_cast<uint32_t>(p1.frame),
                false,
                p1.button,
                1
            });
        }

        if (i < bestPlayable.p2.size()) {
            auto const& p2 = bestPlayable.p2[i];
            auto const& previousP2 = bestPlayable.p2[i - 1];
            if (p2.dualActive &&
                p2.frame > 1 &&
                p2.button != previousP2.button) {
                output.inputs.push_back(gdr::Input(p2.frame, 1, true, p2.button));
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
    result.complete = bestPlayable.complete();
    result.progress = progressFor(
        furthestX,
        solveStartX,
        lvl.length,
        result.complete
    );

    std::ostringstream diagnostics;
    diagnostics
        << "solver=adaptive-team50-30thread-v3"
        << " progress=" << result.progress
        << " frame=" << bestPlayable.frame()
        << " routeX=" << bestPlayable.x()
        << " furthestX=" << furthestX
        << " endX=" << lvl.length
        << " recoveryCount=" << recoveryCount
        << " fullRestarts=" << fullRestarts
        << " hardestSearch=" << hardestSearchLevel
        << " totalTrials=" << totalTrials
        << " physicalWorkers=" << debugPhysicalWorkers
        << " logicalWorkers=" << debugHelpers
        << " focusedRounds=" << focusedRounds
        << " deathRepeat=" << deathRepeat
        << " specialistWins=" << specialistWins
        << " bruteWins=" << bruteWins
        << " refinementWins=" << refinementWins
        << " deadCandidatesRejected=" << deadCandidatesRejected
        << " lastDeathX=" << lastDeathX
        << " bestClearance=" << debugClearance
        << " prunedNoTouch=" << prunedNoTouch
        << " recoveredExceptions=" << recoveredExceptions
        << " moveTriggers=" << lvl.supportedMoveTriggers
        << " unsupportedMoves=" << lvl.unsupportedMoveTriggers
        << " movingObjects=" << lvl.movingObjectIDs.size()
        << " inputs=" << result.inputs.size()
        << " complete=" << (result.complete ? 1 : 0)
        << " stopped=" << (stop.load() ? 1 : 0);
    result.diagnostics = diagnostics.str();

    return result;
}
