#include "solver_dashboard.hpp"

// Reuse the tested v7 simulator/state machinery in this translation unit, but
// replace its controller with the smarter v8 orchestration below.
#define pathfind_v7 pathfind_v7_base_v8
#include "pathfinder_state_v7.cpp"
#undef pathfind_v7

namespace {

constexpr int kLogicalHelpersV8 = 100;
constexpr int kPhysicalThreadsV8 = 30;
constexpr int kGuidedSlotsV8 = 50;
constexpr int kExplorerSlotsV8 = 50;
constexpr int kArchiveLimitV8 = 420;
constexpr float kDeathBucketSizeV8 = 24.f;

struct TaskV8 {
    size_t parent = 0;
    MacroActionV7 action;
    uint64_t failureKey = 0;
};

struct DeathClusterV8 {
    float x = 0.f;
    int count = 0;
    int total = 0;
};

uint64_t failureKeyV8(SnapshotV7 const& snapshot, MacroActionV7 const& action) {
    size_t stateHash = StateKeyHashV7{}(stateKeyV7(snapshot, 6));
    uint64_t h = static_cast<uint64_t>(stateHash);
    h ^= static_cast<uint64_t>(action.signature) * 0x9e3779b97f4a7c15ull;
    h ^= h >> 29;
    h *= 0xbf58476d1ce4e5b9ull;
    h ^= h >> 31;
    return h;
}

DeathClusterV8 dominantDeathClusterV8(
    std::vector<float> const& deaths,
    float furthestX
) {
    DeathClusterV8 cluster;
    cluster.total = static_cast<int>(deaths.size());
    if (deaths.empty())
        return cluster;

    std::unordered_map<int, int> buckets;
    for (float x : deaths) {
        if (!std::isfinite(x))
            continue;
        // Old collisions far behind the frontier are not useful for deciding
        // what the current wall is.
        if (x + 900.f < furthestX)
            continue;
        int bucket = static_cast<int>(std::floor(x / kDeathBucketSizeV8));
        ++buckets[bucket];
    }

    int bestBucket = 0;
    int bestCount = 0;
    for (auto const& [bucket, count] : buckets) {
        if (count > bestCount) {
            bestBucket = bucket;
            bestCount = count;
        }
    }

    if (bestCount > 0) {
        cluster.x = (static_cast<float>(bestBucket) + 0.5f) * kDeathBucketSizeV8;
        cluster.count = bestCount;
    }
    return cluster;
}

std::vector<MacroActionV7> rescueActionsV8(
    Level2& probe,
    SnapshotV7 const& snapshot,
    int precisionLevel
) {
    restoreSnapshotV7(probe, snapshot);
    Player const& p1 = probe.latestState();
    VehicleType mode = p1.vehicle.type;
    int densePrecision = std::max(4, precisionLevel);
    int duration = baseSegmentFramesV7(mode, densePrecision, false);
    duration = std::clamp(duration + 4, flightMode(mode) ? 10 : 14, 34);

    std::vector<MacroActionV7> actions = guidedActionsV7(
        probe,
        snapshot,
        std::max(5, precisionLevel)
    );
    std::unordered_set<uint32_t> signatures;
    signatures.reserve(128);
    for (auto const& action : actions)
        signatures.insert(action.signature);

    auto addP1 = [&](std::vector<std::pair<int, bool>> states) {
        addUniqueActionV7(
            actions,
            makeActionV7(
                duration,
                snapshot.press1,
                snapshot.press2,
                std::move(states)
            ),
            signatures
        );
    };

    // Always include a true "do nothing" and a true "release" branch. A lot
    // of repeated spike deaths are caused by an unnecessary inherited press.
    addP1({});
    addP1({{0, false}});
    addP1({{0, true}});

    if (!flightMode(mode)) {
        std::array<int, 6> widths = mode == VehicleType::Robot
            ? std::array<int, 6>{1, 3, 6, 10, 16, 24}
            : mode == VehicleType::Spider
                ? std::array<int, 6>{1, 2, 3, 5, 8, 12}
                : std::array<int, 6>{1, 2, 4, 7, 11, 18};

        // One-frame start resolution is deliberate here. This is the local
        // surgical search used only when normal beam expansion has stalled.
        for (int start = 0; start < duration - 1; ++start) {
            for (int width : widths) {
                int pressAt = start;
                if (snapshot.press1 && pressAt == 0)
                    pressAt = 1;
                int releaseAt = std::min(duration - 1, pressAt + width);
                std::vector<std::pair<int, bool>> states;
                if (snapshot.press1)
                    states.push_back({0, false});
                states.push_back({pressAt, true});
                states.push_back({releaseAt, false});
                addP1(std::move(states));
                if (actions.size() >= 92)
                    break;
            }
            if (actions.size() >= 92)
                break;
        }
    } else {
        for (int split = 1; split < duration - 1; split += 2) {
            addP1({{0, false}, {split, true}});
            addP1({{0, true}, {split, false}});
            int tail = std::min(duration - 1, split + std::max(2, duration / 4));
            addP1({{0, false}, {split, true}, {tail, false}});
            if (actions.size() >= 88)
                break;
        }
    }

    // Orb presses are searched around the physical contact window at 1-frame
    // resolution, and early presses stay held through contact instead of being
    // spent before the player reaches the orb.
    restoreSnapshotV7(probe, snapshot);
    UpcomingOrbV7 orb = upcomingOrbV7(probe, duration);
    if (orb.found && orb.contactOffset >= 0) {
        int tail = (orb.type == OrbType::Blue || orb.type == OrbType::Green) ? 9 : 6;
        for (int delta = -9; delta <= 9; ++delta) {
            int pressAt = std::clamp(orb.contactOffset + delta, 0, duration - 2);
            if (snapshot.press1 && pressAt == 0)
                pressAt = 1;
            int releaseAt = std::min(
                duration - 1,
                std::max(pressAt + 2, orb.contactOffset + tail)
            );
            std::vector<std::pair<int, bool>> states;
            if (snapshot.press1)
                states.push_back({0, false});
            states.push_back({pressAt, true});
            states.push_back({releaseAt, false});
            addP1(std::move(states));
        }
    }

    if (actions.size() > 96)
        actions.resize(96);
    return actions;
}

std::vector<SearchNodeV7> rescueFrontierV8(
    std::vector<SearchNodeV7> const& archive,
    std::vector<SearchNodeV7> const& current,
    SearchNodeV7 const& initial,
    float focusX,
    int recoveryCount
) {
    if (!(focusX > 0.f)) {
        std::vector<SearchNodeV7> fallback = archive;
        fallback.insert(fallback.end(), current.begin(), current.end());
        if (fallback.empty())
            fallback.push_back(initial);
        return selectFrontierV7(std::move(fallback));
    }

    std::vector<SearchNodeV7> pool = archive;
    pool.insert(pool.end(), current.begin(), current.end());
    std::sort(pool.begin(), pool.end(), nodeBetterV7);

    std::vector<SearchNodeV7> selected;
    selected.reserve(kLogicalHelpersV8);
    std::unordered_set<CoarseKeyV7, CoarseKeyHashV7> used;

    float widen = std::min(420.f, static_cast<float>(recoveryCount) * 45.f);
    std::array<std::pair<float, float>, 4> bands = {{
        {20.f, 115.f + widen * 0.15f},
        {105.f, 230.f + widen * 0.35f},
        {210.f, 410.f + widen * 0.65f},
        {380.f, 720.f + widen}
    }};
    std::array<int, 4> quotas = {16, 14, 12, 8};

    for (size_t bandIndex = 0; bandIndex < bands.size(); ++bandIndex) {
        int added = 0;
        for (auto const& source : pool) {
            float lead = focusX - source.x;
            if (lead < bands[bandIndex].first || lead > bands[bandIndex].second)
                continue;
            CoarseKeyV7 key = coarseKeyV7(source);
            if (!used.insert(key).second)
                continue;
            SearchNodeV7 node = source;
            node.explorer = false;
            selected.push_back(std::move(node));
            if (++added >= quotas[bandIndex] || selected.size() >= kGuidedSlotsV8)
                break;
        }
        if (selected.size() >= kGuidedSlotsV8)
            break;
    }

    // Fill any missing guided slots with the strongest distinct states close
    // enough to influence the failure, but never only the single closest band.
    for (auto const& source : pool) {
        if (selected.size() >= kGuidedSlotsV8)
            break;
        if (source.x > focusX + 35.f || source.x + 950.f < focusX)
            continue;
        CoarseKeyV7 key = coarseKeyV7(source);
        if (!used.insert(key).second)
            continue;
        SearchNodeV7 node = source;
        node.explorer = false;
        selected.push_back(std::move(node));
    }

    // The explorer half deliberately ignores the rescue focus. These states
    // keep global routes alive so a mistaken local theory cannot trap everyone.
    std::vector<SearchNodeV7> broad = pool;
    auto broadSelected = selectFrontierV7(std::move(broad));
    for (auto const& source : broadSelected) {
        if (selected.size() >= kLogicalHelpersV8)
            break;
        CoarseKeyV7 key = coarseKeyV7(source);
        if (!used.insert(key).second)
            continue;
        SearchNodeV7 node = source;
        node.explorer = true;
        selected.push_back(std::move(node));
    }

    for (auto const& source : pool) {
        if (selected.size() >= kLogicalHelpersV8)
            break;
        SearchNodeV7 node = source;
        node.explorer = true;
        selected.push_back(std::move(node));
    }

    if (selected.empty())
        selected.push_back(initial);
    return selected;
}

void mergeArchiveV8(
    std::vector<SearchNodeV7>& archive,
    std::vector<SearchNodeV7> const& frontier
) {
    archive.insert(archive.end(), frontier.begin(), frontier.end());
    std::sort(archive.begin(), archive.end(), nodeBetterV7);

    std::unordered_set<CoarseKeyV7, CoarseKeyHashV7> seen;
    std::vector<SearchNodeV7> compact;
    compact.reserve(std::min<size_t>(archive.size(), kArchiveLimitV8));
    for (auto const& node : archive) {
        CoarseKeyV7 key = coarseKeyV7(node);
        if (!seen.insert(key).second)
            continue;
        compact.push_back(node);
        if (compact.size() >= kArchiveLimitV8)
            break;
    }
    archive = std::move(compact);
}

std::pair<int, int> roleCountsV8(std::vector<SearchNodeV7> const& frontier) {
    int guided = 0;
    int explorers = 0;
    for (auto const& node : frontier) {
        if (node.explorer)
            ++explorers;
        else
            ++guided;
    }
    return {guided, explorers};
}

PathfinderResult runSmartStateSearchV8(
    std::string const& lvlString,
    std::atomic_bool& stop,
    std::function<void(PathfinderTelemetry const&)> callback,
    float trustedEndX
) {
    Level2 root(lvlString);
    int prunedNoTouch = pruneNoTouchPhysicsV7(root, lvlString);

    float startX = root.latestState().pos.x;
    float inferredLength = root.length;
    bool hasTrustedEnd = std::isfinite(trustedEndX) && trustedEndX > startX + 30.f;
    if (hasTrustedEnd) {
        root.length = trustedEndX;
        root.lengthSource = "trusted-gd-v8";
    }

    SearchNodeV7 initial;
    initial.snapshot = captureSnapshotV7(root);
    initial.x = root.latestState().pos.x;
    initial.y = root.latestState().pos.y;
    initial.velocity = root.latestState().velocity;
    initial.minClearance = hazardClearance(root, root.latestState());
    initial.score = scoreStateV7(root, initial.snapshot, initial.minClearance, 0, startX);

    std::vector<SearchNodeV7> frontier {initial};
    std::vector<SearchNodeV7> archive {initial};
    SearchNodeV7 bestNode = initial;

    float furthestX = initial.x;
    float focusX = 0.f;
    float lastDeathX = 0.f;
    uint64_t totalTrials = 0;
    int precisionLevel = 0;
    int stallLayers = 0;
    int rescueLayers = 0;
    int recoveryCount = 0;
    int layer = 0;
    int lastProduced = 0;
    int lastUnique = 0;
    int lastDead = 0;
    int lastDuplicate = 0;
    int lastClusterCount = 0;
    bool stallRescue = false;

    std::unordered_map<StateKeyV7, float, StateKeyHashV7> visited;
    visited[stateKeyV7(initial.snapshot, precisionLevel)] = initial.score;
    std::unordered_map<uint64_t, int> failureMemory;
    failureMemory.reserve(8192);

    std::random_device rd;
    uint32_t baseSeed = rd() ^ static_cast<uint32_t>(std::hash<std::string>{}(lvlString));

    auto emit = [&](int phase,
                    std::string decision,
                    std::string reason,
                    int horizon,
                    int candidateCount,
                    int producedCount,
                    int uniqueCount,
                    int deadCount,
                    int duplicateCount,
                    int deathClusterCount) {
        PathfinderTelemetry telemetry;
        telemetry.progress = progressFor(furthestX, startX, root.length, bestNode.complete);
        telemetry.startX = startX;
        telemetry.currentX = bestNode.x;
        telemetry.furthestX = furthestX;
        telemetry.trustedEndX = hasTrustedEnd ? trustedEndX : 0.f;
        telemetry.inferredLength = inferredLength;
        telemetry.checkpointX = bestNode.x;
        telemetry.deathX = lastDeathX;
        telemetry.deathProgress = static_cast<float>(
            progressFor(lastDeathX, startX, root.length, false)
        );
        telemetry.bestClearance = bestNode.minClearance;
        telemetry.focusX = focusX;
        telemetry.frame = bestNode.snapshot.p1.frame;
        telemetry.checkpointFrame = bestNode.snapshot.p1.frame;
        telemetry.vehicleType = static_cast<int>(bestNode.snapshot.p1.vehicle.type);
        telemetry.searchLevel = precisionLevel;
        telemetry.horizonFrames = horizon;
        telemetry.candidateCount = candidateCount;
        telemetry.workerCount = kLogicalHelpersV8;
        telemetry.physicalThreadCount = kPhysicalThreadsV8;
        telemetry.phase = phase;
        telemetry.frontierCount = static_cast<int>(frontier.size());
        auto [guided, explorers] = roleCountsV8(frontier);
        telemetry.guidedCount = guided;
        telemetry.explorerCount = explorers;
        telemetry.archiveCount = static_cast<int>(archive.size());
        telemetry.producedCount = producedCount;
        telemetry.uniqueCount = uniqueCount;
        telemetry.deadCount = deadCount;
        telemetry.duplicateCount = duplicateCount;
        telemetry.stallLayers = stallLayers;
        telemetry.recoveryCount = recoveryCount;
        telemetry.deathClusterCount = deathClusterCount;
        telemetry.stallRescue = stallRescue;
        telemetry.totalTrials = totalTrials;
        telemetry.mode = "smart-state-space-beam100-v8";
        telemetry.decision = std::move(decision);
        telemetry.recoveryReason = std::move(reason);
        publishPathfinderTelemetryV8(telemetry);
        if (callback)
            callback(telemetry);
    };

    emit(
        0,
        "Mapping distinct physics states",
        "Starting 50 guided states + 50 independent explorers",
        baseSegmentFramesV7(root.latestState().vehicle.type, 0, false),
        1, 0, 1, 0, 0, 0
    );

    while (!frontier.empty() && !stop.load() && !bestNode.complete) {
        ++layer;
        float layerStartFurthest = furthestX;

        std::vector<TaskV8> tasks;
        tasks.reserve(frontier.size() * (stallRescue ? 48 : 20));
        Level2 actionProbe = compactWorkerV7(root);

        for (size_t i = 0; i < frontier.size(); ++i) {
            auto actions = frontier[i].explorer
                ? explorerActionsV7(
                    frontier[i].snapshot,
                    precisionLevel,
                    baseSeed ^ static_cast<uint32_t>(layer * 104729u) ^
                        static_cast<uint32_t>(i * 2654435761u)
                )
                : stallRescue
                    ? rescueActionsV8(actionProbe, frontier[i].snapshot, precisionLevel)
                    : guidedActionsV7(actionProbe, frontier[i].snapshot, precisionLevel);

            for (auto& action : actions) {
                uint64_t failureKey = failureKeyV8(frontier[i].snapshot, action);
                int failures = 0;
                if (auto it = failureMemory.find(failureKey); it != failureMemory.end())
                    failures = it->second;

                int banThreshold = frontier[i].explorer ? 4 : 2;
                if (failures >= banThreshold)
                    continue;
                tasks.push_back({i, std::move(action), failureKey});
            }
        }

        if (tasks.empty()) {
            ++recoveryCount;
            precisionLevel = std::min(6, precisionLevel + 1);
            stallRescue = true;
            focusX = lastDeathX > 0.f ? lastDeathX : furthestX + 120.f;
            frontier = rescueFrontierV8(archive, frontier, initial, focusX, recoveryCount);
            visited.clear();
            emit(
                2,
                "Reopening earlier states",
                "Every scheduled action was already known-dead; widening before the wall",
                0, 0, lastProduced, lastUnique, lastDead, lastDuplicate, lastClusterCount
            );
            continue;
        }

        emit(
            stallRescue ? 2 : 1,
            stallRescue ? "Surgical stall rescue" : "Expanding distinct states",
            stallRescue
                ? fmt::format("1-frame timing search around repeated failure near X {:.0f}", focusX)
                : "Testing short actions while preserving physically different routes",
            tasks.front().action.duration,
            static_cast<int>(tasks.size()),
            lastProduced,
            lastUnique,
            lastDead,
            lastDuplicate,
            lastClusterCount
        );

        std::atomic<size_t> nextTask {0};
        std::atomic<uint64_t> completedTasks {0};
        std::atomic<uint64_t> nextReport {96};
        std::atomic<float> liveBestX {furthestX};
        std::atomic<int> liveAlive {0};
        std::atomic<int> liveDead {0};
        std::mutex reportMutex;
        std::mutex resultMutex;

        std::vector<SearchNodeV7> produced;
        produced.reserve(tasks.size());
        std::vector<float> deathXs;
        deathXs.reserve(tasks.size() / 2 + 16);
        std::vector<uint64_t> failedKeys;
        failedKeys.reserve(tasks.size() / 2 + 16);

        int threadCount = std::min<int>(kPhysicalThreadsV8, static_cast<int>(tasks.size()));
        std::vector<std::thread> threads;
        threads.reserve(threadCount);

        for (int threadIndex = 0; threadIndex < threadCount; ++threadIndex) {
            threads.emplace_back([&, threadIndex] {
                Level2 worker = compactWorkerV7(root);
                std::vector<SearchNodeV7> localProduced;
                std::vector<float> localDeaths;
                std::vector<uint64_t> localFailedKeys;
                localProduced.reserve(tasks.size() / std::max(1, threadCount) + 8);

                while (!stop.load()) {
                    size_t taskIndex = nextTask.fetch_add(1, std::memory_order_relaxed);
                    if (taskIndex >= tasks.size())
                        break;

                    TaskV8 const& task = tasks[taskIndex];
                    SimResultV7 sim = simulateActionV7(
                        worker,
                        frontier[task.parent],
                        task.action,
                        startX
                    );

                    if (sim.dead) {
                        localDeaths.push_back(sim.deathX);
                        localFailedKeys.push_back(task.failureKey);
                        liveDead.fetch_add(1, std::memory_order_relaxed);
                    } else {
                        // A state that escapes the current death focus is much
                        // more valuable during rescue, but global X progress is
                        // still the primary ordering rule in nodeBetterV7.
                        if (stallRescue && focusX > 0.f) {
                            if (sim.node.x > focusX + 30.f)
                                sim.node.score += 50000.f;
                            else if (sim.node.x > focusX - 30.f)
                                sim.node.score += std::clamp(sim.node.minClearance, 0.f, 120.f) * 8.f;
                        }
                        float current = liveBestX.load(std::memory_order_relaxed);
                        while (sim.node.x > current &&
                               !liveBestX.compare_exchange_weak(
                                   current,
                                   sim.node.x,
                                   std::memory_order_relaxed
                               )) {}
                        localProduced.push_back(std::move(sim.node));
                        liveAlive.fetch_add(1, std::memory_order_relaxed);
                    }

                    uint64_t done = completedTasks.fetch_add(1, std::memory_order_relaxed) + 1;
                    uint64_t target = nextReport.load(std::memory_order_relaxed);
                    if (done >= target && nextReport.compare_exchange_strong(
                            target,
                            done + 96,
                            std::memory_order_relaxed
                        )) {
                        std::lock_guard<std::mutex> guard(reportMutex);
                        uint64_t savedTotal = totalTrials;
                        totalTrials = savedTotal + done;
                        float liveX = std::max(
                            furthestX,
                            liveBestX.load(std::memory_order_relaxed)
                        );
                        float savedFurthest = furthestX;
                        furthestX = liveX;
                        emit(
                            stallRescue ? 2 : 1,
                            stallRescue ? "Surgical stall rescue" : "Expanding distinct states",
                            stallRescue
                                ? fmt::format("Micro-timing around X {:.0f}; explorers still search globally", focusX)
                                : "Live batch: deduplicating equivalent physics states",
                            tasks[taskIndex].action.duration,
                            static_cast<int>(tasks.size()),
                            liveAlive.load(std::memory_order_relaxed),
                            lastUnique,
                            liveDead.load(std::memory_order_relaxed),
                            lastDuplicate,
                            lastClusterCount
                        );
                        furthestX = savedFurthest;
                        totalTrials = savedTotal;
                    }
                }

                std::lock_guard<std::mutex> guard(resultMutex);
                produced.insert(
                    produced.end(),
                    std::make_move_iterator(localProduced.begin()),
                    std::make_move_iterator(localProduced.end())
                );
                deathXs.insert(deathXs.end(), localDeaths.begin(), localDeaths.end());
                failedKeys.insert(failedKeys.end(), localFailedKeys.begin(), localFailedKeys.end());
            });
        }

        for (auto& thread : threads)
            thread.join();

        uint64_t layerTrials = completedTasks.load(std::memory_order_relaxed);
        totalTrials += layerTrials;
        for (uint64_t key : failedKeys)
            ++failureMemory[key];

        DeathClusterV8 cluster = dominantDeathClusterV8(deathXs, furthestX);
        if (cluster.count > 0) {
            lastDeathX = cluster.x;
            lastClusterCount = cluster.count;
        } else if (!deathXs.empty()) {
            lastDeathX = *std::max_element(deathXs.begin(), deathXs.end());
            lastClusterCount = 1;
        }

        std::unordered_map<StateKeyV7, SearchNodeV7, StateKeyHashV7> unique;
        unique.reserve(produced.size());
        int visitedRejected = 0;

        for (auto& node : produced) {
            StateKeyV7 key = stateKeyV7(node.snapshot, precisionLevel);
            auto visitedIt = visited.find(key);
            if (visitedIt != visited.end() && visitedIt->second >= node.score - 0.01f) {
                ++visitedRejected;
                continue;
            }
            visited[key] = node.score;

            auto it = unique.find(key);
            if (it == unique.end() || nodeBetterV7(node, it->second))
                unique[key] = std::move(node);
        }

        std::vector<SearchNodeV7> candidates;
        candidates.reserve(unique.size());
        for (auto& [_, node] : unique)
            candidates.push_back(std::move(node));

        lastProduced = static_cast<int>(produced.size());
        lastUnique = static_cast<int>(candidates.size());
        lastDead = static_cast<int>(deathXs.size());
        lastDuplicate = std::max(
            0,
            lastProduced - lastUnique + visitedRejected
        );

        bool improved = false;
        for (auto const& node : candidates) {
            if (node.complete || nodeBetterV7(node, bestNode)) {
                if (node.x > furthestX + 2.f || node.complete)
                    improved = true;
                if (nodeBetterV7(node, bestNode))
                    bestNode = node;
                furthestX = std::max(furthestX, node.x);
            }
        }
        improved = improved || furthestX > layerStartFurthest + 2.f;

        if (bestNode.complete)
            break;

        frontier = selectFrontierV7(std::move(candidates));
        if (!frontier.empty())
            mergeArchiveV8(archive, frontier);

        bool concentratedFailure =
            cluster.count >= 12 &&
            cluster.total > 0 &&
            cluster.count * 2 >= cluster.total &&
            cluster.x >= furthestX - 220.f &&
            cluster.x <= furthestX + 650.f;

        if (improved) {
            stallLayers = 0;
            if (stallRescue) {
                ++rescueLayers;
                if (focusX <= 0.f || furthestX > focusX + 90.f) {
                    stallRescue = false;
                    rescueLayers = 0;
                    focusX = 0.f;
                    if (precisionLevel > 2)
                        --precisionLevel;
                    emit(
                        3,
                        "Breakthrough: failure passed",
                        "A surviving state cleared the repeated-death zone; resuming broad search",
                        baseSegmentFramesV7(bestNode.snapshot.p1.vehicle.type, precisionLevel, false),
                        static_cast<int>(frontier.size()),
                        lastProduced,
                        lastUnique,
                        lastDead,
                        lastDuplicate,
                        lastClusterCount
                    );
                }
            } else if (precisionLevel > 0 && (layer % 5) == 0) {
                --precisionLevel;
            }
        } else {
            ++stallLayers;
            if (stallRescue)
                ++rescueLayers;
        }

        if (!stallRescue && (stallLayers >= 2 || concentratedFailure)) {
            ++recoveryCount;
            stallRescue = true;
            rescueLayers = 0;
            precisionLevel = std::max(4, precisionLevel);
            focusX = cluster.count > 0
                ? cluster.x
                : lastDeathX > 0.f ? lastDeathX : furthestX + 120.f;
            frontier = rescueFrontierV8(
                archive,
                frontier,
                initial,
                focusX,
                recoveryCount
            );
            visited.clear();
            for (auto const& node : frontier)
                visited[stateKeyV7(node.snapshot, precisionLevel)] = node.score;

            emit(
                2,
                "Stall rescue engaged",
                fmt::format(
                    "{} deaths clustered near X {:.0f}; rewinding several lead distances",
                    cluster.count,
                    focusX
                ),
                baseSegmentFramesV7(bestNode.snapshot.p1.vehicle.type, precisionLevel, false),
                static_cast<int>(frontier.size()),
                lastProduced,
                lastUnique,
                lastDead,
                lastDuplicate,
                lastClusterCount
            );
            continue;
        }

        if (stallRescue && (frontier.empty() || rescueLayers >= 9)) {
            ++recoveryCount;
            precisionLevel = std::min(6, precisionLevel + 1);
            rescueLayers = 0;
            frontier = rescueFrontierV8(
                archive,
                frontier,
                initial,
                focusX,
                recoveryCount + 3
            );
            visited.clear();
            for (auto const& node : frontier)
                visited[stateKeyV7(node.snapshot, precisionLevel)] = node.score;

            emit(
                2,
                "Widening stall rescue",
                fmt::format(
                    "Local timing was exhausted near X {:.0f}; reopening earlier approach states",
                    focusX
                ),
                baseSegmentFramesV7(bestNode.snapshot.p1.vehicle.type, precisionLevel, false),
                static_cast<int>(frontier.size()),
                lastProduced,
                lastUnique,
                lastDead,
                lastDuplicate,
                lastClusterCount
            );
        } else if (!stallRescue && improved) {
            emit(
                3,
                "Promoting new frontier",
                "Forward progress found; keeping the strongest and most diverse states",
                baseSegmentFramesV7(bestNode.snapshot.p1.vehicle.type, precisionLevel, false),
                static_cast<int>(frontier.size()),
                lastProduced,
                lastUnique,
                lastDead,
                lastDuplicate,
                lastClusterCount
            );
        }
    }

    PathfinderResult result;
    result.inputs = routeToInputsV7(bestNode.route);
    result.macro = inputsToMacroV7(result.inputs);
    result.complete = bestNode.complete;
    result.progress = progressFor(furthestX, startX, root.length, result.complete);

    std::ostringstream diagnostics;
    diagnostics
        << "solver=smart-state-space-beam100-v8"
        << " progress=" << result.progress
        << " routeX=" << bestNode.x
        << " furthestX=" << furthestX
        << " endX=" << root.length
        << " layers=" << layer
        << " precisionLevel=" << precisionLevel
        << " recoveryCount=" << recoveryCount
        << " totalTrials=" << totalTrials
        << " failureMemory=" << failureMemory.size()
        << " logicalHelpers=" << kLogicalHelpersV8
        << " physicalThreads=" << kPhysicalThreadsV8
        << " archive=" << archive.size()
        << " prunedNoTouch=" << prunedNoTouch
        << " inputs=" << result.inputs.size()
        << " complete=" << (result.complete ? 1 : 0)
        << " stopped=" << (stop.load() ? 1 : 0);
    result.diagnostics = diagnostics.str();

    emit(
        result.complete ? 4 : 2,
        result.complete ? "Candidate level clear found" : "Search stopped with best partial route",
        result.complete
            ? "The exact exported inputs still need fresh replay validation"
            : "No verified completion yet; returning the furthest surviving route",
        0,
        0,
        lastProduced,
        lastUnique,
        lastDead,
        lastDuplicate,
        lastClusterCount
    );
    return result;
}

} // namespace

PathfinderResult pathfind_v8(
    std::string const& lvlString,
    std::atomic_bool& stop,
    std::function<void(PathfinderTelemetry const&)> callback,
    float trustedEndX
) {
    std::function<void(PathfinderTelemetry const&)> guarded;
    if (callback) {
        guarded = [callback](PathfinderTelemetry const& incoming) {
            PathfinderTelemetry telemetry = incoming;
            if (telemetry.progress >= 100.0) {
                telemetry.progress = 99.99;
                telemetry.phase = 4;
                telemetry.decision = "Validating candidate clear";
                telemetry.recoveryReason = "Holding 100% until the exported replay proves the same finish";
            }
            callback(telemetry);
        };
    }

    PathfinderResult result = runSmartStateSearchV8(
        lvlString,
        stop,
        guarded,
        trustedEndX
    );

    if (!result.complete) {
        result.diagnostics += " replayValidation=not-needed";
        return result;
    }

    bool zeroInput = result.inputs.empty();
    ReplayValidationV7 validation = validateReplayV7(
        lvlString,
        result.inputs,
        trustedEndX
    );

    if (!validation.complete) {
        result.complete = false;
        Level2 progressLevel(lvlString);
        float startX = progressLevel.latestState().pos.x;
        float endX = (
            std::isfinite(trustedEndX) && trustedEndX > startX + 30.f
        ) ? trustedEndX : progressLevel.length;
        result.progress = progressFor(
            validation.furthestX,
            startX,
            endX,
            false
        );
        result.diagnostics += " replayValidation=FAILED";
        result.diagnostics += " validatedFrame=" + std::to_string(validation.frame);
        result.diagnostics += " validatedX=" + std::to_string(validation.furthestX);
        result.diagnostics += " validationDead=" + std::to_string(validation.dead ? 1 : 0);
        return result;
    }

    if (zeroInput)
        addValidatedIdleInputV7(result);

    result.complete = true;
    result.progress = 100.0;
    result.diagnostics += " replayValidation=PASS";
    result.diagnostics += " validatedFrame=" + std::to_string(validation.frame);
    result.diagnostics += " validatedX=" + std::to_string(validation.furthestX);
    return result;
}
