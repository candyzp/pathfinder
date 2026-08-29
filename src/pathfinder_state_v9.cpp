#include "solver_dashboard.hpp"

// V9 keeps the proven simulator/state primitives from v7, but replaces the
// controller. The important distinction is that "furthest seen" is speculative
// while "safe progress" is revocable and must be proven by surviving children.
#define pathfind_v7 pathfind_v7_base_v9
#include "pathfinder_state_v7.cpp"
#undef pathfind_v7

namespace {

constexpr int kLogicalHelpersV9 = 100;
constexpr int kPhysicalThreadsV9 = 30;
constexpr int kGuidedSlotsV9 = 50;
constexpr int kExplorerSlotsV9 = 50;
constexpr int kArchiveLimitV9 = 640;
constexpr float kDeathBucketV9 = 24.f;
constexpr float kEscapeMarginV9 = 150.f;
constexpr float kPoisonGuardV9 = 42.f;

struct TaskV9 {
    size_t parent = 0;
    MacroActionV7 action;
    uint64_t failureKey = 0;
};

struct DeathClusterV9 {
    float x = 0.f;
    int count = 0;
    int total = 0;
};

int rollbackDistanceV9(int level) {
    static constexpr std::array<int, 10> kDistances = {
        120, 220, 360, 540, 780, 1080, 1450, 1900, 2500, 3300
    };
    if (level < static_cast<int>(kDistances.size()))
        return kDistances[std::max(0, level)];
    int extra = level - static_cast<int>(kDistances.size()) + 1;
    return kDistances.back() + extra * 1000;
}

uint64_t coarseFailureKeyV9(
    SearchNodeV7 const& parent,
    MacroActionV7 const& action
) {
    CoarseKeyV7 state = coarseKeyV7(parent);
    uint64_t h = 1469598103934665603ull;
    auto mix = [&](uint64_t value) {
        h ^= value + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
        h *= 1099511628211ull;
    };

    // Deliberately coarser than v8. Nearby states and nearly identical timings
    // should share failure history instead of rediscovering the same wall as a
    // "new" idea one frame or a few pixels later.
    mix(static_cast<uint32_t>(state.x));
    mix(static_cast<uint32_t>(state.y));
    mix(static_cast<uint32_t>(state.velocity));
    mix(state.flags);
    mix(static_cast<uint32_t>(std::max(1, action.duration / 4)));
    mix(static_cast<uint32_t>(action.toggles.size()));
    for (auto const& toggle : action.toggles) {
        mix(static_cast<uint32_t>(std::max(0, toggle.offset) / 3));
        mix(toggle.player2 ? 2u : 1u);
    }
    return h;
}

DeathClusterV9 dominantDeathClusterV9(
    std::vector<float> const& deaths,
    float speculativeX
) {
    DeathClusterV9 result;
    result.total = static_cast<int>(deaths.size());
    if (deaths.empty())
        return result;

    std::unordered_map<int, int> buckets;
    for (float x : deaths) {
        if (!std::isfinite(x))
            continue;
        if (x + 1200.f < speculativeX)
            continue;
        ++buckets[static_cast<int>(std::floor(x / kDeathBucketV9))];
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
        result.x = (static_cast<float>(bestBucket) + 0.5f) * kDeathBucketV9;
        result.count = bestCount;
    }
    return result;
}

std::vector<MacroActionV7> rescueActionsV9(
    Level2& probe,
    SnapshotV7 const& snapshot,
    int precisionLevel
) {
    restoreSnapshotV7(probe, snapshot);
    Player const& p1 = probe.latestState();
    VehicleType mode = p1.vehicle.type;
    int duration = baseSegmentFramesV7(mode, std::max(4, precisionLevel), false);
    duration = std::clamp(duration + 8, flightMode(mode) ? 12 : 18, 42);

    std::vector<MacroActionV7> actions = guidedActionsV7(
        probe,
        snapshot,
        std::max(5, precisionLevel)
    );
    std::unordered_set<uint32_t> signatures;
    signatures.reserve(160);
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

    // Explicit alternatives to inherited behavior. A dead end can be caused by
    // a jump as easily as by the absence of one.
    addP1({});
    addP1({{0, false}});
    addP1({{0, true}});

    if (!flightMode(mode)) {
        std::array<int, 7> widths = mode == VehicleType::Robot
            ? std::array<int, 7>{1, 3, 6, 10, 15, 22, 30}
            : mode == VehicleType::Spider
                ? std::array<int, 7>{1, 2, 3, 5, 8, 11, 15}
                : std::array<int, 7>{1, 2, 4, 7, 11, 17, 25};

        // The local rescue pass is intentionally one-frame resolution.
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
                if (actions.size() >= 108)
                    break;
            }
            if (actions.size() >= 108)
                break;
        }
    } else {
        for (int split = 1; split < duration - 1; ++split) {
            addP1({{0, false}, {split, true}});
            addP1({{0, true}, {split, false}});
            int tail = std::min(duration - 1, split + std::max(2, duration / 4));
            addP1({{0, false}, {split, true}, {tail, false}});
            if (actions.size() >= 104)
                break;
        }
    }

    // Orb timing is centered on the actual contact window. Early presses remain
    // held through contact, so they are not spent before entering the orb.
    restoreSnapshotV7(probe, snapshot);
    UpcomingOrbV7 orb = upcomingOrbV7(probe, duration);
    if (orb.found && orb.contactOffset >= 0) {
        int tail = (orb.type == OrbType::Blue || orb.type == OrbType::Green) ? 10 : 7;
        for (int delta = -12; delta <= 12; ++delta) {
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

    if (actions.size() > 112)
        actions.resize(112);
    return actions;
}

void mergeArchiveV9(
    std::vector<SearchNodeV7>& archive,
    std::vector<SearchNodeV7> const& nodes
) {
    archive.insert(archive.end(), nodes.begin(), nodes.end());
    std::sort(archive.begin(), archive.end(), nodeBetterV7);

    std::unordered_set<CoarseKeyV7, CoarseKeyHashV7> seen;
    std::vector<SearchNodeV7> compact;
    compact.reserve(std::min<size_t>(archive.size(), kArchiveLimitV9));
    for (auto const& node : archive) {
        CoarseKeyV7 key = coarseKeyV7(node);
        if (!seen.insert(key).second)
            continue;
        compact.push_back(node);
        if (compact.size() >= kArchiveLimitV9)
            break;
    }
    archive = std::move(compact);
}

void quarantineDeadEndV9(
    std::vector<SearchNodeV7>& archive,
    float focusX
) {
    if (!(focusX > 0.f))
        return;
    archive.erase(
        std::remove_if(
            archive.begin(),
            archive.end(),
            [&](SearchNodeV7 const& node) {
                return !node.complete &&
                       node.x > focusX - kPoisonGuardV9 &&
                       node.x < focusX + kEscapeMarginV9;
            }
        ),
        archive.end()
    );
}

SearchNodeV7 rollbackAnchorV9(
    std::vector<SearchNodeV7> const& archive,
    std::vector<SearchNodeV7> const& frontier,
    SearchNodeV7 const& initial,
    float focusX,
    int rollbackDistance
) {
    float ceiling = focusX - static_cast<float>(rollbackDistance);
    SearchNodeV7 best = initial;
    bool found = false;

    auto consider = [&](SearchNodeV7 const& node) {
        if (node.complete || node.x > ceiling)
            return;
        if (!found || node.x > best.x + 0.5f ||
            (std::abs(node.x - best.x) <= 0.5f && nodeBetterV7(node, best))) {
            best = node;
            found = true;
        }
    };

    for (auto const& node : archive)
        consider(node);
    for (auto const& node : frontier)
        consider(node);
    return found ? best : initial;
}

std::vector<SearchNodeV7> rollbackFrontierV9(
    std::vector<SearchNodeV7> const& archive,
    std::vector<SearchNodeV7> const& current,
    SearchNodeV7 const& initial,
    float focusX,
    int rollbackDistance
) {
    std::vector<SearchNodeV7> pool = archive;
    pool.insert(pool.end(), current.begin(), current.end());
    if (pool.empty())
        pool.push_back(initial);
    std::sort(pool.begin(), pool.end(), nodeBetterV7);

    float targetX = focusX - static_cast<float>(rollbackDistance);
    float hardCeiling = focusX - kPoisonGuardV9;

    std::vector<SearchNodeV7> selected;
    selected.reserve(kLogicalHelpersV9);
    std::unordered_set<CoarseKeyV7, CoarseKeyHashV7> used;

    // Guided workers are deliberately moved around the progressively earlier
    // rollback target. This is the core "back up farther every time" behavior.
    std::array<std::pair<float, float>, 4> bands = {{
        {-80.f, 70.f},
        {-180.f, 120.f},
        {-340.f, 170.f},
        {-620.f, 220.f}
    }};
    std::array<int, 4> quotas = {18, 14, 10, 8};

    for (size_t b = 0; b < bands.size(); ++b) {
        int added = 0;
        for (auto const& source : pool) {
            if (source.x >= hardCeiling)
                continue;
            float delta = source.x - targetX;
            if (delta < bands[b].first || delta > bands[b].second)
                continue;
            CoarseKeyV7 key = coarseKeyV7(source);
            if (!used.insert(key).second)
                continue;
            SearchNodeV7 node = source;
            node.explorer = false;
            selected.push_back(std::move(node));
            if (++added >= quotas[b] || selected.size() >= kGuidedSlotsV9)
                break;
        }
        if (selected.size() >= kGuidedSlotsV9)
            break;
    }

    for (auto const& source : pool) {
        if (selected.size() >= kGuidedSlotsV9)
            break;
        if (source.x >= hardCeiling)
            continue;
        CoarseKeyV7 key = coarseKeyV7(source);
        if (!used.insert(key).second)
            continue;
        SearchNodeV7 node = source;
        node.explorer = false;
        selected.push_back(std::move(node));
    }

    // The explorer half stays broad, but it may not restart from inside the
    // currently poisoned basin. It must approach the wall from a genuinely
    // different earlier state.
    auto broad = selectFrontierV7(pool);
    for (auto const& source : broad) {
        if (selected.size() >= kLogicalHelpersV9)
            break;
        if (source.x > focusX - 20.f && source.x < focusX + kEscapeMarginV9)
            continue;
        CoarseKeyV7 key = coarseKeyV7(source);
        if (!used.insert(key).second)
            continue;
        SearchNodeV7 node = source;
        node.explorer = true;
        selected.push_back(std::move(node));
    }

    for (auto const& source : pool) {
        if (selected.size() >= kLogicalHelpersV9)
            break;
        if (source.x > focusX - 20.f && source.x < focusX + kEscapeMarginV9)
            continue;
        SearchNodeV7 node = source;
        node.explorer = true;
        selected.push_back(std::move(node));
    }

    if (selected.empty())
        selected.push_back(initial);
    return selected;
}

std::pair<int, int> roleCountsV9(std::vector<SearchNodeV7> const& frontier) {
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

PathfinderResult runDeadEndLearningV9(
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
        root.lengthSource = "trusted-gd-v9";
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
    SearchNodeV7 safeNode = initial;
    SearchNodeV7 seenNode = initial;
    SearchNodeV7 solutionNode = initial;

    float speculativeX = initial.x;
    float focusX = 0.f;
    float lastDeathX = 0.f;
    uint64_t totalTrials = 0;
    int precisionLevel = 0;
    int stallLayers = 0;
    int recoveryCount = 0;
    int deadEndLevel = 0;
    int rollbackDistance = 0;
    int rescueLayersAtDepth = 0;
    int sameBasinRounds = 0;
    int layer = 0;
    int lastProduced = 0;
    int lastUnique = 0;
    int lastDead = 0;
    int lastDuplicate = 0;
    int lastClusterCount = 0;
    bool deadEnd = false;
    bool complete = false;

    std::unordered_map<StateKeyV7, float, StateKeyHashV7> visited;
    visited[stateKeyV7(initial.snapshot, precisionLevel)] = initial.score;
    std::unordered_map<uint64_t, int> failureMemory;
    failureMemory.reserve(12000);

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
        telemetry.progress = progressFor(
            complete ? root.length : safeNode.x,
            startX,
            root.length,
            complete
        );
        telemetry.startX = startX;
        telemetry.currentX = safeNode.x;
        telemetry.furthestX = speculativeX;
        telemetry.trustedEndX = hasTrustedEnd ? trustedEndX : 0.f;
        telemetry.inferredLength = inferredLength;
        telemetry.checkpointX = safeNode.x;
        telemetry.deathX = lastDeathX;
        telemetry.deathProgress = static_cast<float>(
            progressFor(lastDeathX, startX, root.length, false)
        );
        telemetry.bestClearance = safeNode.minClearance;
        telemetry.focusX = focusX;
        telemetry.frame = safeNode.snapshot.p1.frame;
        telemetry.checkpointFrame = safeNode.snapshot.p1.frame;
        telemetry.vehicleType = static_cast<int>(safeNode.snapshot.p1.vehicle.type);
        telemetry.searchLevel = precisionLevel;
        telemetry.horizonFrames = horizon;
        telemetry.candidateCount = candidateCount;
        telemetry.workerCount = kLogicalHelpersV9;
        telemetry.physicalThreadCount = kPhysicalThreadsV9;
        telemetry.phase = phase;
        telemetry.frontierCount = static_cast<int>(frontier.size());
        auto [guided, explorers] = roleCountsV9(frontier);
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
        telemetry.rollbackDistance = rollbackDistance;
        telemetry.deadEndLevel = deadEndLevel;
        telemetry.stallRescue = deadEnd;
        telemetry.progressLocked = deadEnd;
        telemetry.totalTrials = totalTrials;
        telemetry.mode = "dead-end-backtracking-beam100-v9";
        telemetry.decision = std::move(decision);
        telemetry.recoveryReason = std::move(reason);
        publishPathfinderTelemetryV8(telemetry);
        if (callback)
            callback(telemetry);
    };

    auto reseedVisited = [&] {
        visited.clear();
        for (auto const& node : frontier)
            visited[stateKeyV7(node.snapshot, precisionLevel)] = node.score;
    };

    auto rollbackNow = [&](bool deepen, std::string reason) {
        deadEnd = true;
        if (deepen)
            ++deadEndLevel;
        rollbackDistance = rollbackDistanceV9(deadEndLevel);
        ++recoveryCount;
        ++precisionLevel;
        precisionLevel = std::clamp(precisionLevel, 4, 6);
        rescueLayersAtDepth = 0;
        sameBasinRounds = 0;

        quarantineDeadEndV9(archive, focusX);
        SearchNodeV7 anchor = rollbackAnchorV9(
            archive,
            frontier,
            initial,
            focusX,
            rollbackDistance
        );

        // This is intentionally revocable. If the old "progress" fed a dead
        // end, it is no longer trusted and the visible percentage may go back.
        safeNode = anchor;
        frontier = rollbackFrontierV9(
            archive,
            frontier,
            initial,
            focusX,
            rollbackDistance
        );
        mergeArchiveV9(archive, {safeNode});
        reseedVisited();

        emit(
            2,
            deepen ? "Dead end persisted: rolling back farther" : "Dead end detected: revoking unsafe progress",
            fmt::format(
                "{} | safe X {:.0f}, wall X {:.0f}, rollback {} units",
                reason,
                safeNode.x,
                focusX,
                rollbackDistance
            ),
            baseSegmentFramesV7(safeNode.snapshot.p1.vehicle.type, precisionLevel, false),
            static_cast<int>(frontier.size()),
            lastProduced,
            lastUnique,
            lastDead,
            lastDuplicate,
            lastClusterCount
        );
    };

    emit(
        0,
        "Learning safe route history",
        "Progress only becomes trusted after a surviving child proves the parent was not a dead end",
        baseSegmentFramesV7(root.latestState().vehicle.type, 0, false),
        1, 0, 1, 0, 0, 0
    );

    while (!frontier.empty() && !stop.load() && !complete) {
        ++layer;
        float safeAtLayerStart = safeNode.x;

        std::vector<TaskV9> tasks;
        tasks.reserve(frontier.size() * (deadEnd ? 70 : 24));
        Level2 actionProbe = compactWorkerV7(root);

        for (size_t i = 0; i < frontier.size(); ++i) {
            auto actions = frontier[i].explorer
                ? explorerActionsV7(
                    frontier[i].snapshot,
                    precisionLevel,
                    baseSeed ^ static_cast<uint32_t>(layer * 104729u) ^
                        static_cast<uint32_t>(i * 2654435761u)
                )
                : deadEnd
                    ? rescueActionsV9(actionProbe, frontier[i].snapshot, precisionLevel)
                    : guidedActionsV7(actionProbe, frontier[i].snapshot, precisionLevel);

            for (auto& action : actions) {
                uint64_t key = coarseFailureKeyV9(frontier[i], action);
                int failures = 0;
                if (auto it = failureMemory.find(key); it != failureMemory.end())
                    failures = it->second;
                int banThreshold = frontier[i].explorer ? 5 : 2;
                if (failures >= banThreshold)
                    continue;
                tasks.push_back({i, std::move(action), key});
            }
        }

        if (tasks.empty()) {
            if (!(focusX > 0.f))
                focusX = lastDeathX > 0.f ? lastDeathX : speculativeX + 120.f;
            rollbackNow(deadEnd, "All actions from the current approach are already known failures");
            continue;
        }

        emit(
            deadEnd ? 2 : 1,
            deadEnd ? "Escaping poisoned dead-end basin" : "Testing forward states",
            deadEnd
                ? fmt::format(
                    "Rollback depth {} ({} units); safe progress frozen until a route survives past X {:.0f}",
                    deadEndLevel,
                    rollbackDistance,
                    focusX + kEscapeMarginV9
                )
                : "50 guided states learn the route while 50 explorers preserve unrelated alternatives",
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
        std::atomic<float> liveSeenX {speculativeX};
        std::atomic<int> liveAlive {0};
        std::atomic<int> liveDead {0};
        std::mutex reportMutex;
        std::mutex resultMutex;

        std::vector<SearchNodeV7> produced;
        std::vector<float> deathXs;
        std::vector<uint64_t> failedKeys;
        produced.reserve(tasks.size());
        deathXs.reserve(tasks.size() / 2 + 16);
        failedKeys.reserve(tasks.size() / 2 + 16);

        std::vector<float> parentMaxAlive(frontier.size(), -std::numeric_limits<float>::infinity());
        std::vector<int> parentAlive(frontier.size(), 0);
        std::vector<int> parentDead(frontier.size(), 0);

        int threadCount = std::min<int>(kPhysicalThreadsV9, static_cast<int>(tasks.size()));
        std::vector<std::thread> threads;
        threads.reserve(threadCount);

        for (int threadIndex = 0; threadIndex < threadCount; ++threadIndex) {
            threads.emplace_back([&, threadIndex] {
                Level2 worker = compactWorkerV7(root);
                std::vector<SearchNodeV7> localProduced;
                std::vector<float> localDeaths;
                std::vector<uint64_t> localFailedKeys;
                std::vector<float> localParentMax(frontier.size(), -std::numeric_limits<float>::infinity());
                std::vector<int> localParentAlive(frontier.size(), 0);
                std::vector<int> localParentDead(frontier.size(), 0);
                localProduced.reserve(tasks.size() / std::max(1, threadCount) + 8);

                while (!stop.load()) {
                    size_t taskIndex = nextTask.fetch_add(1, std::memory_order_relaxed);
                    if (taskIndex >= tasks.size())
                        break;
                    TaskV9 const& task = tasks[taskIndex];

                    SimResultV7 sim = simulateActionV7(
                        worker,
                        frontier[task.parent],
                        task.action,
                        startX
                    );

                    if (sim.dead) {
                        localDeaths.push_back(sim.deathX);
                        localFailedKeys.push_back(task.failureKey);
                        ++localParentDead[task.parent];
                        liveDead.fetch_add(1, std::memory_order_relaxed);
                    } else {
                        ++localParentAlive[task.parent];
                        localParentMax[task.parent] = std::max(
                            localParentMax[task.parent],
                            sim.node.x
                        );
                        float current = liveSeenX.load(std::memory_order_relaxed);
                        while (sim.node.x > current &&
                               !liveSeenX.compare_exchange_weak(
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
                        uint64_t savedTrials = totalTrials;
                        float savedSeen = speculativeX;
                        totalTrials = savedTrials + done;
                        speculativeX = std::max(
                            speculativeX,
                            liveSeenX.load(std::memory_order_relaxed)
                        );
                        emit(
                            deadEnd ? 2 : 1,
                            deadEnd ? "Escaping poisoned dead-end basin" : "Testing forward states",
                            deadEnd
                                ? fmt::format(
                                    "Safe X {:.0f} locked; seen X {:.0f}; backing from wall X {:.0f}",
                                    safeNode.x,
                                    speculativeX,
                                    focusX
                                )
                                : "Live batch: deaths are evidence only, never progress",
                            tasks[taskIndex].action.duration,
                            static_cast<int>(tasks.size()),
                            liveAlive.load(std::memory_order_relaxed),
                            lastUnique,
                            liveDead.load(std::memory_order_relaxed),
                            lastDuplicate,
                            lastClusterCount
                        );
                        speculativeX = savedSeen;
                        totalTrials = savedTrials;
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
                for (size_t i = 0; i < frontier.size(); ++i) {
                    parentMaxAlive[i] = std::max(parentMaxAlive[i], localParentMax[i]);
                    parentAlive[i] += localParentAlive[i];
                    parentDead[i] += localParentDead[i];
                }
            });
        }

        for (auto& thread : threads)
            thread.join();

        totalTrials += completedTasks.load(std::memory_order_relaxed);
        for (uint64_t key : failedKeys)
            ++failureMemory[key];

        DeathClusterV9 cluster = dominantDeathClusterV9(deathXs, speculativeX);
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
        lastDuplicate = std::max(0, lastProduced - lastUnique + visitedRejected);

        SearchNodeV7 escapedNode;
        bool escaped = false;
        for (auto const& node : candidates) {
            if (node.complete) {
                if (!complete || nodeBetterV7(node, solutionNode))
                    solutionNode = node;
                complete = true;
            }
            if (node.x > speculativeX + 0.5f || nodeBetterV7(node, seenNode)) {
                if (node.x > seenNode.x + 0.5f || nodeBetterV7(node, seenNode))
                    seenNode = node;
                speculativeX = std::max(speculativeX, node.x);
            }
            if (deadEnd && node.x > focusX + kEscapeMarginV9) {
                if (!escaped || nodeBetterV7(node, escapedNode)) {
                    escapedNode = node;
                    escaped = true;
                }
            }
        }

        if (complete)
            break;

        // Promote safe progress from a PARENT only after a child survived an
        // entire segment beyond it. This one-layer proof prevents the visible
        // percentage from celebrating every speculative brush with a wall.
        bool safeAdvanced = false;
        if (!deadEnd) {
            SearchNodeV7 proven = safeNode;
            bool foundProven = false;
            for (size_t i = 0; i < frontier.size(); ++i) {
                if (parentAlive[i] <= 0)
                    continue;
                float proofDistance = flightMode(frontier[i].snapshot.p1.vehicle.type) ? 18.f : 24.f;
                if (parentMaxAlive[i] < frontier[i].x + proofDistance)
                    continue;
                if (cluster.count > 0 && frontier[i].x > cluster.x - 105.f)
                    continue;
                if (frontier[i].x <= safeNode.x + 1.f)
                    continue;
                if (!foundProven || nodeBetterV7(frontier[i], proven)) {
                    proven = frontier[i];
                    foundProven = true;
                }
            }
            if (foundProven) {
                safeNode = proven;
                safeAdvanced = safeNode.x > safeAtLayerStart + 1.f;
            }
        }

        bool concentratedFailure =
            cluster.count >= 8 &&
            cluster.total > 0 &&
            cluster.count * 3 >= cluster.total &&
            cluster.x >= safeNode.x - 80.f &&
            cluster.x <= speculativeX + 260.f;

        if (safeAdvanced)
            stallLayers = 0;
        else
            ++stallLayers;

        if (deadEnd) {
            ++rescueLayersAtDepth;
            bool sameBasin = cluster.count > 0 && std::abs(cluster.x - focusX) <= 72.f;
            sameBasinRounds = sameBasin ? sameBasinRounds + 1 : 0;

            if (escaped) {
                safeNode = escapedNode;
                deadEnd = false;
                deadEndLevel = 0;
                rollbackDistance = 0;
                rescueLayersAtDepth = 0;
                sameBasinRounds = 0;
                stallLayers = 0;
                focusX = 0.f;
                if (precisionLevel > 2)
                    --precisionLevel;

                frontier = selectFrontierV7(std::move(candidates));
                mergeArchiveV9(archive, frontier);
                reseedVisited();
                emit(
                    3,
                    "Dead end escaped: promoting verified route",
                    "A living state survived beyond the old death wall; safe progress is unlocked again",
                    baseSegmentFramesV7(safeNode.snapshot.p1.vehicle.type, precisionLevel, false),
                    static_cast<int>(frontier.size()),
                    lastProduced,
                    lastUnique,
                    lastDead,
                    lastDuplicate,
                    lastClusterCount
                );
                continue;
            }

            frontier = selectFrontierV7(std::move(candidates));
            if (!frontier.empty())
                mergeArchiveV9(archive, frontier);

            // Do not spend dozens of layers micro-timing the same basin. Three
            // failed rescue generations, two repeated basin confirmations, or
            // an empty frontier immediately forces a deeper rollback.
            if (frontier.empty() || rescueLayersAtDepth >= 3 || sameBasinRounds >= 2) {
                if (cluster.count > 0 && std::abs(cluster.x - focusX) <= 110.f)
                    focusX = focusX * 0.75f + cluster.x * 0.25f;
                rollbackNow(true, "The same death basin survived the previous rescue depth");
                continue;
            }

            emit(
                2,
                "Testing alternate approach from rollback anchor",
                fmt::format(
                    "Safe X {:.0f} remains frozen; must pass X {:.0f} before progress can be trusted",
                    safeNode.x,
                    focusX + kEscapeMarginV9
                ),
                frontier.empty() ? 0 : baseSegmentFramesV7(frontier.front().snapshot.p1.vehicle.type, precisionLevel, false),
                static_cast<int>(frontier.size()),
                lastProduced,
                lastUnique,
                lastDead,
                lastDuplicate,
                lastClusterCount
            );
            continue;
        }

        frontier = selectFrontierV7(std::move(candidates));
        if (!frontier.empty())
            mergeArchiveV9(archive, frontier);

        bool shouldEnterDeadEnd =
            frontier.empty() ||
            concentratedFailure ||
            (stallLayers >= 2 && cluster.count >= 4);

        if (shouldEnterDeadEnd) {
            focusX = cluster.count > 0
                ? cluster.x
                : lastDeathX > 0.f ? lastDeathX : speculativeX;
            deadEndLevel = 0;
            rollbackDistance = rollbackDistanceV9(deadEndLevel);
            rollbackNow(
                false,
                cluster.count > 0
                    ? fmt::format("{} repeated deaths identify the wall", cluster.count)
                    : "The frontier stopped producing verified progress"
            );
            continue;
        }

        if (safeAdvanced) {
            emit(
                3,
                "Verified progress promoted",
                "A surviving child proved this checkpoint can continue, so SAFE progress moved forward",
                baseSegmentFramesV7(safeNode.snapshot.p1.vehicle.type, precisionLevel, false),
                static_cast<int>(frontier.size()),
                lastProduced,
                lastUnique,
                lastDead,
                lastDuplicate,
                lastClusterCount
            );
        } else {
            emit(
                1,
                "Holding speculative progress",
                "Helpers reached farther X, but SAFE progress will not move until continuation is proven",
                frontier.empty() ? 0 : baseSegmentFramesV7(frontier.front().snapshot.p1.vehicle.type, precisionLevel, false),
                static_cast<int>(frontier.size()),
                lastProduced,
                lastUnique,
                lastDead,
                lastDuplicate,
                lastClusterCount
            );
        }
    }

    SearchNodeV7 const& outputNode = complete ? solutionNode : safeNode;
    PathfinderResult result;
    result.inputs = routeToInputsV7(outputNode.route);
    result.macro = inputsToMacroV7(result.inputs);
    result.complete = complete;
    result.progress = progressFor(
        complete ? root.length : safeNode.x,
        startX,
        root.length,
        complete
    );

    std::ostringstream diagnostics;
    diagnostics
        << "solver=dead-end-backtracking-beam100-v9"
        << " progress=" << result.progress
        << " safeX=" << safeNode.x
        << " speculativeX=" << speculativeX
        << " focusX=" << focusX
        << " rollbackDistance=" << rollbackDistance
        << " deadEndLevel=" << deadEndLevel
        << " endX=" << root.length
        << " layers=" << layer
        << " precisionLevel=" << precisionLevel
        << " recoveryCount=" << recoveryCount
        << " totalTrials=" << totalTrials
        << " failureMemory=" << failureMemory.size()
        << " logicalHelpers=" << kLogicalHelpersV9
        << " physicalThreads=" << kPhysicalThreadsV9
        << " archive=" << archive.size()
        << " prunedNoTouch=" << prunedNoTouch
        << " inputs=" << result.inputs.size()
        << " complete=" << (result.complete ? 1 : 0)
        << " stopped=" << (stop.load() ? 1 : 0);
    result.diagnostics = diagnostics.str();

    emit(
        result.complete ? 4 : 2,
        result.complete ? "Candidate clear found" : "Stopped on last verified-safe route",
        result.complete
            ? "The exported inputs still have to prove the finish in a fresh replay"
            : "Speculative/dead-end progress was intentionally discarded instead of being accepted",
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

PathfinderResult pathfind_v9(
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
                telemetry.recoveryReason = "100% stays locked until the exported replay reproduces the finish";
            }
            callback(telemetry);
        };
    }

    PathfinderResult result = runDeadEndLearningV9(
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
