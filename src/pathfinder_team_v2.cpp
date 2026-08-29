#include <array>
#include <mutex>
#include <utility>

#include <Orb.hpp>

// Reuse the simulator, candidate generators, scoring helpers and replay export
// implementation, but replace the old controller with the cooperative v2 pool.
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
    int workerCount = 30;
    int logicalWorkerCount = 50;
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
        ? 6
        : startMode == VehicleType::Robot ? 8 : 12;
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

    if (!trial.dead && !flightMode(mode)) {
        float xDelta = trial.x - best.x;
        if (std::abs(xDelta) <= 8.f) {
            if (toggleCount + 3 < bestToggleCount)
                return true;
            if (bestToggleCount + 6 < toggleCount)
                return false;
        }
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

std::set<SearchInput> makeBruteCandidate(
    Level2 const& base,
    int horizonFrames,
    int lane,
    int attempt,
    std::mt19937& rng
) {
    std::set<SearchInput> inputs;
    int frame = base.currentFrame();
    VehicleType mode = base.latestState().vehicle.type;
    bool dual = base.latestState().dualActive;

    int maxP1 = maxToggleBudget(mode) + 2 + (lane % 5) * 2;
    int maxP2 = dual
        ? maxToggleBudget(base.latestState2().vehicle.type) + 2 + (lane % 4) * 2
        : 0;

    int focusWindow = std::min(
        horizonFrames,
        72 + lane * std::max(24, horizonFrames / 16)
    );
    focusWindow = std::max(1, focusWindow);

    std::uniform_int_distribution<int> fullFrame(0, std::max(0, horizonFrames - 1));
    std::uniform_int_distribution<int> focusFrame(0, focusWindow - 1);
    std::uniform_int_distribution<int> p1Budget(0, std::max(0, maxP1));
    std::uniform_int_distribution<int> p2Budget(0, std::max(0, maxP2));

    int p1Count = p1Budget(rng);
    int p2Count = p2Budget(rng);

    auto randomOffset = [&](int index) {
        bool focused = ((index + attempt + lane) % 5) != 0;
        return focused ? focusFrame(rng) : fullFrame(rng);
    };

    for (int i = 0; i < p1Count; ++i)
        addToggle(inputs, frame, horizonFrames, randomOffset(i), false);
    for (int i = 0; i < p2Count; ++i)
        addToggle(inputs, frame, horizonFrames, randomOffset(i + p1Count), true);

    if ((lane + attempt) % 3 == 0) {
        int start = focusFrame(rng);
        int maxWidth = mode == VehicleType::Robot
            ? 36
            : mode == VehicleType::Spider
                ? 240
                : 96;
        int widthCap = std::max(1, std::min(maxWidth, horizonFrames - start - 1));
        if (widthCap > 0) {
            std::uniform_int_distribution<int> widthDist(1, widthCap);
            int width = widthDist(rng);
            addToggle(inputs, frame, horizonFrames, start, false);
            addToggle(inputs, frame, horizonFrames, start + width, false);
        }
    }

    return inputs;
}

std::set<SearchInput> mutateLeaderCandidate(
    Level2 const& base,
    std::set<SearchInput> const& rawLeader,
    int horizonFrames,
    int lane,
    int attempt,
    std::mt19937& rng
) {
    int frame = base.currentFrame();
    VehicleType mode = base.latestState().vehicle.type;
    bool dual = base.latestState().dualActive;
    bool dashing = base.latestState().dashing;

    std::set<SearchInput> route = windowedRoute(rawLeader, frame, horizonFrames);

    if (route.empty()) {
        int usable = std::max(1, std::min(horizonFrames, 480));
        int offset = (lane * 37 + attempt * 17) % usable;

        if (dashing) {
            int releaseAt = std::min(horizonFrames - 1, 36 + (lane % 10) * 24 + attempt * 3);
            if (releaseAt > 0)
                addToggle(route, frame, horizonFrames, releaseAt, false);
            return route;
        }

        addToggle(route, frame, horizonFrames, offset, false);
        int width = mode == VehicleType::Robot
            ? 1 + ((lane * 5 + attempt * 3) % 36)
            : mode == VehicleType::Spider
                ? 1 + ((lane + attempt) % 2 == 0
                    ? (lane % 6) + 1
                    : 48 + (lane % 5) * 36)
                : 4 + ((lane * 11 + attempt * 7) % 72);
        if (offset + width < horizonFrames)
            addToggle(route, frame, horizonFrames, offset + width, false);
        return route;
    }

    std::vector<SearchInput> keys(route.begin(), route.end());
    int strategy = (lane + attempt) % 7;
    std::uniform_int_distribution<size_t> pick(0, keys.size() - 1);
    SearchInput chosen = keys[pick(rng)];
    int chosenFrame = static_cast<int>(chosen >> 1);
    bool chosenP2 = (chosen & 1u) != 0;

    if (strategy == 0) {
        static constexpr int deltas[] = {-24, -12, -6, -3, -1, 1, 3, 6, 12, 24};
        int delta = deltas[(lane * 3 + attempt) % (sizeof(deltas) / sizeof(deltas[0]))];
        int shifted = std::clamp(chosenFrame + delta, frame, frame + horizonFrames - 1);
        route.erase(chosen);
        route.insert(inputKey(static_cast<uint32_t>(shifted), chosenP2));
    } else if (strategy == 1) {
        route.erase(chosen);
    } else if (strategy == 2) {
        int localWindow = std::max(1, std::min(horizonFrames, 360));
        std::uniform_int_distribution<int> dist(0, localWindow - 1);
        addToggle(route, frame, horizonFrames, dist(rng), false);
    } else if (strategy == 3) {
        int localWindow = std::max(1, std::min(horizonFrames, 420));
        std::uniform_int_distribution<int> dist(0, localWindow - 1);
        int start = dist(rng);
        int width = mode == VehicleType::Robot
            ? 1 + ((lane * 7 + attempt) % 36)
            : mode == VehicleType::Spider
                ? 1 + ((lane + attempt) % 2 == 0
                    ? (lane % 6) + 1
                    : 48 + (attempt % 8) * 24)
                : 4 + ((lane * 9 + attempt) % 96);
        addToggle(route, frame, horizonFrames, start, false);
        addToggle(route, frame, horizonFrames, start + width, false);
    } else if (strategy == 4) {
        int delta = ((lane + attempt) % 17) - 8;
        if (delta == 0)
            delta = lane % 2 == 0 ? -1 : 1;
        std::set<SearchInput> shiftedRoute;
        for (SearchInput key : route) {
            int f = static_cast<int>(key >> 1) + delta;
            bool p2 = (key & 1u) != 0;
            if (f >= frame && f < frame + horizonFrames)
                shiftedRoute.insert(inputKey(static_cast<uint32_t>(f), p2));
        }
        route = std::move(shiftedRoute);
    } else if (strategy == 5) {
        int delta = mode == VehicleType::Spider
            ? 24 + ((lane + attempt) % 8) * 24
            : mode == VehicleType::Robot
                ? 1 + ((lane + attempt) % 12)
                : 6 + ((lane + attempt) % 8) * 3;
        if ((attempt & 1) != 0)
            delta = -delta;
        int shifted = std::clamp(chosenFrame + delta, frame, frame + horizonFrames - 1);
        route.erase(chosen);
        route.insert(inputKey(static_cast<uint32_t>(shifted), chosenP2));
    } else {
        if (dual) {
            route.insert(inputKey(static_cast<uint32_t>(chosenFrame), !chosenP2));
        } else {
            int lateStart = std::max(0, horizonFrames / 3);
            std::uniform_int_distribution<int> late(lateStart, std::max(lateStart, horizonFrames - 1));
            addToggle(route, frame, horizonFrames, late(rng), false);
        }
    }

    return route;
}

bool normalJumpOrb(OrbType type) {
    return type == OrbType::Yellow ||
           type == OrbType::Blue ||
           type == OrbType::Pink ||
           type == OrbType::Red ||
           type == OrbType::Green ||
           type == OrbType::Black;
}

void appendUpcomingOrbCandidates(
    Level2 const& base,
    int horizonFrames,
    std::vector<std::set<SearchInput>>& candidates
) {
    Player const& player = base.latestState();
    if (player.direction <= 0 || player.dashing || flightMode(player.vehicle.type))
        return;

    int speedIndex = std::clamp(player.speed, 0, 4);
    double timeScale = std::clamp(static_cast<double>(player.timeWarp), 0.05, 4.0);
    double xPerSecond = player_speeds[speedIndex] * timeScale;
    if (xPerSecond < 1.0)
        return;

    float travelX = static_cast<float>(xPerSecond * horizonFrames / 240.0);
    float lowX = player.pos.x - 45.f;
    float highX = player.pos.x + travelX + 60.f;
    int firstSection = static_cast<int>(std::floor(lowX / Level::sectionSize));
    int lastSection = static_cast<int>(std::floor(highX / Level::sectionSize));

    Orb const* nearest = nullptr;
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
            if (dx < -45.f || dx > travelX + 60.f)
                continue;
            if (dx < nearestDx) {
                nearestDx = dx;
                nearest = orb;
            }
        }
    }

    if (nearest == nullptr)
        return;

    int frame = base.currentFrame();
    int center = static_cast<int>(std::lround(
        static_cast<double>(nearest->pos.x - player.pos.x) * 240.0 / xPerSecond
    ));

    static constexpr std::array<int, 17> coarseDeltas = {
        -24, -18, -14, -10, -8, -6, -4, -2, 0,
        2, 4, 6, 8, 10, 14, 18, 24
    };
    static constexpr std::array<int, 25> blueDeltas = {
        -24, -20, -16, -14, -12, -10, -8, -6, -4, -2, -1, 0, 1,
        2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24
    };
    static constexpr std::array<int, 6> widths = {1, 2, 4, 8, 16, 28};

    auto addOrbPulse = [&](int pressAt, int width) {
        if (pressAt < 0 || pressAt >= horizonFrames)
            return;

        std::set<SearchInput> pulse;
        int actualPress = pressAt;
        if (base.press1) {
            int releaseAt = std::max(0, pressAt - 3);
            addToggle(pulse, frame, horizonFrames, releaseAt, false);
            actualPress = std::max(releaseAt + 1, pressAt);
        }

        addToggle(pulse, frame, horizonFrames, actualPress, false);
        if (actualPress + width < horizonFrames)
            addToggle(pulse, frame, horizonFrames, actualPress + width, false);
        candidates.push_back(std::move(pulse));
    };

    if (nearest->type == OrbType::Blue) {
        for (int delta : blueDeltas) {
            int pressAt = center + delta;
            for (int width : widths)
                addOrbPulse(pressAt, width);
        }
    } else {
        for (int delta : coarseDeltas) {
            int pressAt = center + delta;
            for (int width : {1, 2, 4, 8})
                addOrbPulse(pressAt, width);
        }
    }
}

TeamBatchResult evaluateTeamBatch(
    Level2 const& base,
    std::vector<std::set<SearchInput>> const& specialists,
    std::set<SearchInput> const& seedLeader,
    int horizonFrames,
    int searchLevel,
    uint32_t baseSeed,
    std::atomic_bool& stop
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
    constexpr int kBruteLogicalLanes = 20;
    constexpr int kRefineLogicalLanes = 20;
    constexpr int kLogicalWorkers = kRoleThreads + kBruteLogicalLanes + kRefineLogicalLanes;

    int generatedAttempts = std::clamp(22 + searchLevel * 2, 22, 44);
    out.workerCount = kTotalThreads;
    out.logicalWorkerCount = kLogicalWorkers;
    out.candidateCount = static_cast<int>(
        specialists.size() + generatedAttempts * kRoleThreads * 2
    );

    Level2 compactBase = compactWorkerBase(base);

    std::atomic_bool batchSolved = false;
    std::atomic<uint64_t> trials = 0;
    std::mutex bestMutex;
    size_t bestToggleCount = std::numeric_limits<size_t>::max();

    std::mutex leaderMutex;
    std::set<SearchInput> liveLeader;
    TrialResult liveLeaderTrial {
        frame,
        base.latestState().pos.x,
        0.f,
        0.f,
        false,
        false
    };
    size_t liveLeaderToggleCount = std::numeric_limits<size_t>::max();
    bool haveLiveLeader = false;
    std::atomic<uint64_t> leaderVersion {0};
    std::atomic<float> leaderX {base.latestState().pos.x};
    std::atomic<size_t> leaderToggleCount {std::numeric_limits<size_t>::max()};

    auto maybePublishLeader = [&](std::set<SearchInput> const& candidate, TrialResult const& trial) {
        if (trial.dead)
            return;

        float publishedX = leaderX.load(std::memory_order_relaxed);
        size_t publishedToggles = leaderToggleCount.load(std::memory_order_relaxed);
        bool likelyImprovement =
            leaderVersion.load(std::memory_order_acquire) == 0 ||
            trial.complete ||
            trial.x > publishedX + 4.f ||
            candidate.size() + 4 < publishedToggles;
        if (!likelyImprovement)
            return;

        std::lock_guard<std::mutex> guard(leaderMutex);
        bool improve = !haveLiveLeader ||
            trial.complete ||
            trial.x > liveLeaderTrial.x + 4.f ||
            (std::abs(trial.x - liveLeaderTrial.x) <= 4.f &&
             candidate.size() + 4 < liveLeaderToggleCount);
        if (!improve)
            return;

        liveLeader = candidate;
        liveLeaderTrial = trial;
        liveLeaderToggleCount = candidate.size();
        haveLiveLeader = true;
        leaderX.store(trial.x, std::memory_order_relaxed);
        leaderToggleCount.store(candidate.size(), std::memory_order_relaxed);
        leaderVersion.fetch_add(1, std::memory_order_release);
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

    for (int lane = 0; lane < kRoleThreads; ++lane) {
        threads.emplace_back([&, lane] {
            Level2 worker = compactBase;
            prepareWorker(worker);
            WorkerBaseline baseline = captureWorkerBaseline(worker);
            LocalBest local;

            for (size_t index = static_cast<size_t>(lane);
                 index < specialists.size() &&
                 !stop.load() &&
                 !batchSolved.load(std::memory_order_acquire);
                 index += kRoleThreads) {
                TrialResult trial = tryInputsTeam(worker, baseline, specialists[index], horizonFrames);
                trials.fetch_add(1, std::memory_order_relaxed);
                considerLocal(local, specialists[index], trial, TeamRole::Specialist, mode);
                maybePublishLeader(specialists[index], trial);
                if (trial.complete)
                    batchSolved.store(true, std::memory_order_release);
            }
            mergeLocal(std::move(local));
        });
    }

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
                 !batchSolved.load(std::memory_order_acquire);
                 ++attempt) {
                int logicalLane = lane + ((attempt & 1) ? kRoleThreads : 0);
                auto candidate = makeBruteCandidate(
                    base,
                    horizonFrames,
                    logicalLane,
                    attempt,
                    localRng
                );
                TrialResult trial = tryInputsTeam(worker, baseline, candidate, horizonFrames);
                trials.fetch_add(1, std::memory_order_relaxed);
                considerLocal(local, candidate, trial, TeamRole::BruteForce, mode);
                maybePublishLeader(candidate, trial);
                if (trial.complete)
                    batchSolved.store(true, std::memory_order_release);
            }
            mergeLocal(std::move(local));
        });
    }

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
            std::set<SearchInput> cachedLeader;

            for (int attempt = 0;
                 attempt < generatedAttempts &&
                 !stop.load() &&
                 !batchSolved.load(std::memory_order_acquire);
                 ++attempt) {
                uint64_t version = leaderVersion.load(std::memory_order_acquire);
                if (version != cachedVersion) {
                    std::lock_guard<std::mutex> guard(leaderMutex);
                    cachedLeader = haveLiveLeader ? liveLeader : seedLeader;
                    cachedVersion = leaderVersion.load(std::memory_order_relaxed);
                }

                if (cachedLeader.empty()) {
                    if (!seedLeader.empty()) {
                        cachedLeader = seedLeader;
                    } else if (!specialists.empty()) {
                        size_t seedIndex = static_cast<size_t>(
                            (lane + attempt * kRoleThreads) % specialists.size()
                        );
                        cachedLeader = specialists[seedIndex];
                    }
                }

                int logicalLane = lane + ((attempt & 1) ? kRoleThreads : 0);
                auto candidate = mutateLeaderCandidate(
                    base,
                    cachedLeader,
                    horizonFrames,
                    logicalLane,
                    attempt,
                    localRng
                );
                TrialResult trial = tryInputsTeam(worker, baseline, candidate, horizonFrames);
                trials.fetch_add(1, std::memory_order_relaxed);
                considerLocal(local, candidate, trial, TeamRole::Refinement, mode);
                maybePublishLeader(candidate, trial);
                if (trial.complete)
                    batchSolved.store(true, std::memory_order_release);
            }
            mergeLocal(std::move(local));
        });
    }

    for (auto& thread : threads)
        thread.join();

    out.trials = trials.load(std::memory_order_relaxed);
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
    int debugSearchLevel = 0;
    int debugHorizon = 0;
    int debugCandidateCount = 0;
    int debugWorkers = 30;
    int debugLogicalWorkers = 50;
    int debugVehicleType = static_cast<int>(lvl.latestState().vehicle.type);
    uint64_t totalTrials = 0;

    float furthestX = lvl.latestState().pos.x;
    float lastDeathX = 0.f;
    float debugClearance = 0.f;
    Timeline bestPlayable(lvl);
    std::set<SearchInput> persistentLeader;

    auto emitTelemetry = [&](int phase, char const* reason) {
        if (!callback)
            return;

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
        telemetry.workerCount = debugWorkers;
        telemetry.phase = phase;
        telemetry.totalTrials = totalTrials;
        telemetry.mode = "team50-logical-30-thread-v2";
        telemetry.recoveryReason = reason;
        callback(telemetry);
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
                recoverFromBest(std::min(2880, 240 + recoveryCount * 240));
                emitTelemetry(2, "recover-dead-state");
                continue;
            }

            int frame = lvl.currentFrame();
            VehicleType mode = lvl.latestState().vehicle.type;
            int searchLevel = std::min(12, recoveryCount + stagnantRounds / 4);
            hardestSearchLevel = std::max(hardestSearchLevel, searchLevel);

            int horizonFrames = 720 + searchLevel * 80;
            if (flightMode(mode))
                horizonFrames += 240;
            else if (mode == VehicleType::Robot)
                horizonFrames += 120;
            else if (mode == VehicleType::Spider)
                horizonFrames = std::min(horizonFrames, 1200);
            horizonFrames = std::min(horizonFrames, 1920);

            bool dual = lvl.latestState().dualActive;
            auto specialists = structuredCandidates(
                frame,
                horizonFrames,
                mode,
                dual,
                lvl.press1,
                lvl.latestState().dashing
            );
            appendUpcomingOrbCandidates(lvl, horizonFrames, specialists);

            int generatedAttempts = std::clamp(22 + searchLevel * 2, 22, 44);
            debugVehicleType = static_cast<int>(mode);
            debugSearchLevel = searchLevel;
            debugHorizon = horizonFrames;
            debugWorkers = 30;
            debugLogicalWorkers = 50;
            debugCandidateCount = static_cast<int>(
                specialists.size() + generatedAttempts * 20
            );
            debugClearance = 0.f;
            emitTelemetry(0, "team-v2-search");

            TeamBatchResult team = evaluateTeamBatch(
                lvl,
                specialists,
                persistentLeader,
                horizonFrames,
                searchLevel,
                baseSeed ^ static_cast<uint32_t>(recoveryCount * 7919u),
                stop
            );

            totalTrials += team.trials;
            debugWorkers = team.workerCount;
            debugLogicalWorkers = team.logicalWorkerCount;
            debugCandidateCount = team.candidateCount;
            if (team.lastDeathX != 0.f)
                lastDeathX = team.lastDeathX;

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
                int safetyDivisor = flightMode(mode) ||
                    mode == VehicleType::Robot ||
                    mode == VehicleType::Spider
                    ? 4 : 5;
                int safetyTail = std::max(12, advance / safetyDivisor);
                applyUntil = std::max(frame + 1, bestTrial.frame - safetyTail);
            } else {
                int deathBuffer = std::min(300, 110 + searchLevel * 12);
                int safeEnd = std::max(frame, bestTrial.frame - deathBuffer);
                int safeAdvance = safeEnd - frame;
                applyUntil = frame + safeAdvance / 2;
                ++deadCandidatesRejected;
                persistentLeader.clear();
            }

            if (applyUntil <= frame) {
                recoverFromBest(std::min(2880, 240 + recoveryCount * 240));
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
                recoverFromBest(std::min(2880, 360 + recoveryCount * 240));
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
                emitTelemetry(3, reachedGoal(lvl) ? "complete" : "advance");
            } else if (lvl.latestState().direction < 0) {
                stagnantRounds = 0;
            } else {
                ++stagnantRounds;
            }

            if (stagnantRounds >= 10 &&
                bestPlayable.frame() > 2 &&
                lvl.latestState().direction >= 0) {
                int retreat = std::min(
                    bestPlayable.frame() - 1,
                    420 * (1 + std::min(recoveryCount, 10))
                );
                recoverFromBest(retreat);
                stagnantRounds = 0;
                fail = 1;
                numAway = std::min(8000, 1000 + recoveryCount * 500);
                emitTelemetry(2, "recover-stagnation");
            }
        } catch (std::exception const&) {
            ++recoveredExceptions;
            recoverFromBest(std::min(3600, 480 + recoveredExceptions * 240));
            emitTelemetry(2, "recover-exception");
        } catch (...) {
            ++recoveredExceptions;
            recoverFromBest(std::min(3600, 480 + recoveredExceptions * 240));
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
        << "solver=team50-logical-30-thread-v2"
        << " progress=" << result.progress
        << " frame=" << bestPlayable.frame()
        << " routeX=" << bestPlayable.x()
        << " furthestX=" << furthestX
        << " endX=" << lvl.length
        << " recoveryCount=" << recoveryCount
        << " fullRestarts=" << fullRestarts
        << " hardestSearch=" << hardestSearchLevel
        << " totalTrials=" << totalTrials
        << " physicalWorkers=" << debugWorkers
        << " logicalWorkers=" << debugLogicalWorkers
        << " specialistWins=" << specialistWins
        << " bruteWins=" << bruteWins
        << " refinementWins=" << refinementWins
        << " deadCandidatesRejected=" << deadCandidatesRejected
        << " lastDeathX=" << lastDeathX
        << " bestClearance=" << debugClearance
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
