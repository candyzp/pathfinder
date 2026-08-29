// V10 turns the advertised 100 helpers into 100 logical jobs per generation.
// It keeps V9/V7 simulation primitives, but uses a new controller where each
// helper owns exactly one task. Recovery never treats a rollback anchor as new
// forward progress, and repeated deaths at the same wall monotonically increase
// rollback depth until a living route escapes beyond the poisoned basin.
#define pathfind_v9 pathfind_v9_base_v10
#include "pathfinder_state_v9.cpp"
#undef pathfind_v9

namespace {

constexpr int kLogicalHelpersV10 = 100;
constexpr int kHelperSquadsV10 = 10;
constexpr int kHelpersPerSquadV10 = 10;
constexpr int kPhysicalThreadsV10 = 30;
constexpr int kArchiveLimitV10 = 900;
constexpr float kEscapeMarginV10 = 165.f;
constexpr float kSameWallToleranceV10 = 84.f;
constexpr float kRollbackPoisonGuardV10 = 48.f;

static_assert(kHelperSquadsV10 * kHelpersPerSquadV10 == kLogicalHelpersV10);

struct TaskV10 {
    int helperId = 0;
    int squad = 0;
    size_t parent = 0;
    MacroActionV7 action;
    uint64_t failureKey = 0;
};

uint64_t assignmentKeyV10(SearchNodeV7 const& parent, MacroActionV7 const& action) {
    uint64_t h = coarseFailureKeyV9(parent, action);
    h ^= static_cast<uint64_t>(action.signature) * 0x9e3779b97f4a7c15ull;
    h ^= static_cast<uint64_t>(std::max(0, action.duration)) * 0xbf58476d1ce4e5b9ull;
    for (auto const& toggle : action.toggles) {
        uint64_t v = static_cast<uint64_t>(std::max(0, toggle.offset) + 1);
        v = (v << 2) ^ static_cast<uint64_t>(toggle.player2 ? 2u : 1u);
        h ^= v + 0x94d049bb133111ebull + (h << 6) + (h >> 2);
    }
    return h;
}

MacroActionV7 fallbackActionV10(
    SearchNodeV7 const& parent,
    int helperId,
    int precisionLevel,
    int salt
) {
    VehicleType mode = parent.snapshot.p1.vehicle.type;
    int duration = baseSegmentFramesV7(mode, std::max(precisionLevel, 2), false);
    duration = std::clamp(duration + (helperId / 10) - 4, 10, 48);

    int usable = std::max(3, duration - 2);
    int a = 1 + ((helperId * 7 + salt * 11) % usable);
    int b = 1 + ((helperId * 13 + salt * 5 + 3) % usable);
    if (a > b)
        std::swap(a, b);
    if (a == b)
        b = std::min(duration - 1, a + 1);

    std::vector<std::pair<int, bool>> states;
    bool startPressed = parent.snapshot.press1;
    if (startPressed)
        states.push_back({0, false});

    bool firstState = ((helperId + salt) & 1) == 0;
    states.push_back({std::clamp(a, 0, duration - 1), firstState});
    states.push_back({std::clamp(b, 0, duration - 1), !firstState});

    int c = 1 + ((helperId * 17 + salt * 19 + 1) % usable);
    if ((helperId + salt) % 3 == 0)
        states.push_back({std::clamp(c, 0, duration - 1), firstState});

    std::sort(states.begin(), states.end(), [](auto const& lhs, auto const& rhs) {
        return lhs.first < rhs.first;
    });
    states.erase(
        std::unique(states.begin(), states.end(), [](auto const& lhs, auto const& rhs) {
            return lhs.first == rhs.first && lhs.second == rhs.second;
        }),
        states.end()
    );

    return makeActionV7(
        duration,
        parent.snapshot.press1,
        parent.snapshot.press2,
        std::move(states)
    );
}

std::vector<SearchNodeV7> helperParentPoolV10(
    std::vector<SearchNodeV7> const& frontier,
    std::vector<SearchNodeV7> const& archive,
    SearchNodeV7 const& initial,
    bool deadEnd,
    float focusX,
    int rollbackDistance
) {
    if (deadEnd) {
        auto rolled = rollbackFrontierV9(
            archive,
            frontier,
            initial,
            focusX,
            rollbackDistance
        );
        if (!rolled.empty())
            return rolled;
    }

    std::vector<SearchNodeV7> combined = frontier;
    combined.insert(combined.end(), archive.begin(), archive.end());
    if (combined.empty())
        combined.push_back(initial);

    std::sort(combined.begin(), combined.end(), nodeBetterV7);
    std::unordered_set<CoarseKeyV7, CoarseKeyHashV7> seen;
    std::vector<SearchNodeV7> unique;
    unique.reserve(std::min<size_t>(combined.size(), 320));

    // Keep good states, but do not let one nearly-identical prefix occupy the
    // whole parent pool. The helper action layer supplies the second axis of
    // diversity, so even a tiny early frontier still gets 100 different jobs.
    for (auto const& node : combined) {
        CoarseKeyV7 key = coarseKeyV7(node);
        if (!seen.insert(key).second)
            continue;
        unique.push_back(node);
        if (unique.size() >= 320)
            break;
    }

    if (unique.empty())
        unique.push_back(initial);
    return unique;
}

std::vector<MacroActionV7> actionsForHelperV10(
    Level2& actionProbe,
    SearchNodeV7 const& parent,
    int squad,
    int helperId,
    int precisionLevel,
    bool deadEnd,
    uint32_t seed
) {
    // Ten squads of ten. Squads 0-4 are deterministic/guided timing lanes;
    // squads 5-9 are independent explorers with different random seeds.
    // During recovery, rescue timing replaces the guided half so the solver
    // actively attacks the wall instead of replaying the route that found it.
    if (squad < 5) {
        if (deadEnd || squad == 2 || squad == 4)
            return rescueActionsV9(actionProbe, parent.snapshot, precisionLevel + squad / 2);
        return guidedActionsV7(actionProbe, parent.snapshot, precisionLevel + (squad & 1));
    }

    return explorerActionsV7(
        parent.snapshot,
        precisionLevel + ((squad - 5) / 2),
        seed ^ static_cast<uint32_t>(helperId * 2654435761u) ^
            static_cast<uint32_t>((squad + 1) * 2246822519u)
    );
}

std::vector<TaskV10> buildHelperTasksV10(
    Level2& actionProbe,
    std::vector<SearchNodeV7> const& parents,
    std::unordered_map<uint64_t, int> const& failureMemory,
    int precisionLevel,
    bool deadEnd,
    uint32_t baseSeed,
    int layer
) {
    std::vector<TaskV10> tasks;
    tasks.reserve(kLogicalHelpersV10);
    if (parents.empty())
        return tasks;

    std::unordered_set<uint64_t> usedAssignments;
    usedAssignments.reserve(kLogicalHelpersV10 * 3);

    for (int helperId = 0; helperId < kLogicalHelpersV10; ++helperId) {
        int squad = helperId / kHelpersPerSquadV10;
        int lane = helperId % kHelpersPerSquadV10;

        // Different squads stride through the parent pool differently. This is
        // intentional: helper 73 is not helper 72 with one timing bit changed.
        size_t parentIndex = static_cast<size_t>(
            (helperId * 37 + squad * 17 + lane * 11 + layer * 7) %
            static_cast<int>(parents.size())
        );
        SearchNodeV7 const& parent = parents[parentIndex];

        uint32_t seed = baseSeed ^ static_cast<uint32_t>(layer * 104729u) ^
            static_cast<uint32_t>((helperId + 1) * 3266489917u);
        auto actions = actionsForHelperV10(
            actionProbe,
            parent,
            squad,
            helperId,
            precisionLevel,
            deadEnd,
            seed
        );

        bool assigned = false;
        if (!actions.empty()) {
            size_t start = static_cast<size_t>(
                (lane * 7 + squad * 13 + layer * 3) % static_cast<int>(actions.size())
            );
            for (size_t attempt = 0; attempt < actions.size(); ++attempt) {
                MacroActionV7 const& action = actions[(start + attempt) % actions.size()];
                uint64_t failureKey = coarseFailureKeyV9(parent, action);
                int failures = 0;
                if (auto it = failureMemory.find(failureKey); it != failureMemory.end())
                    failures = it->second;
                int banThreshold = squad >= 5 ? 5 : 2;
                if (failures >= banThreshold)
                    continue;

                uint64_t assignment = assignmentKeyV10(parent, action);
                if (!usedAssignments.insert(assignment).second)
                    continue;

                tasks.push_back({helperId, squad, parentIndex, action, failureKey});
                assigned = true;
                break;
            }
        }

        // If a role's normal action catalogue has been exhausted by failure
        // memory, synthesize a new timing pattern rather than silently deleting
        // that helper. The contract is 100 helpers -> 100 owned jobs.
        for (int salt = 0; !assigned && salt < 48; ++salt) {
            MacroActionV7 action = fallbackActionV10(
                parent,
                helperId,
                precisionLevel,
                layer * 53 + salt
            );
            uint64_t failureKey = coarseFailureKeyV9(parent, action);
            int failures = 0;
            if (auto it = failureMemory.find(failureKey); it != failureMemory.end())
                failures = it->second;
            if (failures >= 6)
                continue;
            uint64_t assignment = assignmentKeyV10(parent, action);
            if (!usedAssignments.insert(assignment).second)
                continue;
            tasks.push_back({helperId, squad, parentIndex, std::move(action), failureKey});
            assigned = true;
        }

        // This should be practically unreachable, but keep the helper present
        // even if every synthesized timing hashes to an exhausted family.
        if (!assigned) {
            MacroActionV7 action = fallbackActionV10(
                parent,
                helperId,
                precisionLevel,
                layer * 977 + helperId
            );
            uint64_t failureKey = coarseFailureKeyV9(parent, action);
            tasks.push_back({helperId, squad, parentIndex, std::move(action), failureKey});
        }
    }

    return tasks;
}

PathfinderResult runIndependentHelpersV10(
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
        root.lengthSource = "trusted-gd-v10";
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
    float recoveryOriginSafeX = initial.x;
    float rollbackAnchorX = initial.x;
    uint64_t totalTrials = 0;
    int precisionLevel = 0;
    int stallLayers = 0;
    int recoveryCount = 0;
    int deadEndLevel = 0;
    int rollbackDistance = 0;
    int recoveryLayersAtDepth = 0;
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
    failureMemory.reserve(16000);

    std::random_device rd;
    uint32_t baseSeed = rd() ^ static_cast<uint32_t>(std::hash<std::string>{}(lvlString));

    auto emit = [&](int phase,
                    std::string decision,
                    std::string reason,
                    int horizon,
                    int candidateCount) {
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
        telemetry.workerCount = kLogicalHelpersV10;
        telemetry.physicalThreadCount = kPhysicalThreadsV10;
        telemetry.phase = phase;
        telemetry.frontierCount = static_cast<int>(frontier.size());
        telemetry.guidedCount = 50;
        telemetry.explorerCount = 50;
        telemetry.archiveCount = static_cast<int>(archive.size());
        telemetry.producedCount = lastProduced;
        telemetry.uniqueCount = lastUnique;
        telemetry.deadCount = lastDead;
        telemetry.duplicateCount = lastDuplicate;
        telemetry.stallLayers = stallLayers;
        telemetry.recoveryCount = recoveryCount;
        telemetry.deathClusterCount = lastClusterCount;
        telemetry.rollbackDistance = rollbackDistance;
        telemetry.deadEndLevel = deadEndLevel;
        telemetry.stallRescue = deadEnd;
        telemetry.progressLocked = deadEnd;
        telemetry.totalTrials = totalTrials;
        telemetry.mode = "independent-100-helpers-10x10-v10";
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
        if (!deadEnd) {
            deadEnd = true;
            deadEndLevel = 0;
            recoveryOriginSafeX = safeNode.x;
        } else if (deepen) {
            ++deadEndLevel;
        }

        rollbackDistance = rollbackDistanceV9(deadEndLevel);
        ++recoveryCount;
        ++precisionLevel;
        precisionLevel = std::clamp(precisionLevel, 4, 6);
        recoveryLayersAtDepth = 0;

        quarantineDeadEndV9(archive, focusX);
        SearchNodeV7 anchor = rollbackAnchorV9(
            archive,
            frontier,
            initial,
            focusX,
            rollbackDistance
        );
        rollbackAnchorX = anchor.x;

        // Critical anti-loop rule: the rollback anchor is a SEARCH ORIGIN, not
        // newly-earned progress. SAFE may be revoked backwards here, but it is
        // locked and cannot advance again until a living route clears the old
        // death wall + escape margin.
        safeNode = anchor;
        frontier = rollbackFrontierV9(
            archive,
            frontier,
            initial,
            focusX,
            rollbackDistance
        );
        mergeArchiveV9(archive, {anchor});
        reseedVisited();

        emit(
            2,
            deepen ? "Same wall again: backing up farther" : "Dead end: revoking unsafe checkpoint",
            fmt::format(
                "{} | pre-dead-end SAFE X {:.0f}; rollback origin X {:.0f}; wall X {:.0f}; depth {} ({} units). Rollback X is not counted as new progress.",
                reason,
                recoveryOriginSafeX,
                rollbackAnchorX,
                focusX,
                deadEndLevel,
                rollbackDistance
            ),
            baseSegmentFramesV7(safeNode.snapshot.p1.vehicle.type, precisionLevel, false),
            kLogicalHelpersV10
        );
    };

    emit(
        0,
        "100 helpers ready: 10 squads x 10 owned jobs",
        "Each helper receives one independent action assignment; rollback anchors never count as forward progress",
        baseSegmentFramesV7(root.latestState().vehicle.type, 0, false),
        kLogicalHelpersV10
    );

    while (!frontier.empty() && !stop.load() && !complete) {
        ++layer;
        float safeAtLayerStart = safeNode.x;

        auto parents = helperParentPoolV10(
            frontier,
            archive,
            initial,
            deadEnd,
            focusX,
            rollbackDistance
        );
        if (parents.empty())
            parents.push_back(initial);

        Level2 actionProbe = compactWorkerV7(root);
        std::vector<TaskV10> tasks = buildHelperTasksV10(
            actionProbe,
            parents,
            failureMemory,
            precisionLevel,
            deadEnd,
            baseSeed,
            layer
        );

        if (tasks.empty()) {
            if (!(focusX > 0.f))
                focusX = lastDeathX > 0.f ? lastDeathX : speculativeX + 120.f;
            rollbackNow(deadEnd, "No helper could produce a fresh action");
            continue;
        }

        emit(
            deadEnd ? 2 : 1,
            deadEnd ? "100 helpers attacking alternate approaches" : "100 helpers running independent jobs",
            deadEnd
                ? fmt::format(
                    "SAFE is frozen at rollback origin X {:.0f}; no checkpoint promotion until a living route passes X {:.0f}",
                    rollbackAnchorX,
                    focusX + kEscapeMarginV10
                )
                : "Ten squads cover guided timing, rescue timing, and five separately seeded explorer families",
            tasks.front().action.duration,
            static_cast<int>(tasks.size())
        );

        std::atomic<size_t> nextTask {0};
        std::atomic<uint64_t> completedTasks {0};
        std::atomic<float> liveSeenX {speculativeX};
        std::mutex resultMutex;

        std::vector<SearchNodeV7> produced;
        std::vector<float> deathXs;
        std::vector<uint64_t> failedKeys;
        produced.reserve(tasks.size());
        deathXs.reserve(tasks.size());
        failedKeys.reserve(tasks.size());

        std::vector<float> parentMaxAlive(parents.size(), -std::numeric_limits<float>::infinity());
        std::vector<int> parentAlive(parents.size(), 0);
        std::vector<int> parentDead(parents.size(), 0);

        int threadCount = std::min<int>(kPhysicalThreadsV10, static_cast<int>(tasks.size()));
        std::vector<std::thread> threads;
        threads.reserve(threadCount);

        for (int threadIndex = 0; threadIndex < threadCount; ++threadIndex) {
            threads.emplace_back([&, threadIndex] {
                Level2 worker = compactWorkerV7(root);
                std::vector<SearchNodeV7> localProduced;
                std::vector<float> localDeaths;
                std::vector<uint64_t> localFailed;
                std::vector<float> localParentMax(parents.size(), -std::numeric_limits<float>::infinity());
                std::vector<int> localParentAlive(parents.size(), 0);
                std::vector<int> localParentDead(parents.size(), 0);

                while (!stop.load()) {
                    size_t index = nextTask.fetch_add(1, std::memory_order_relaxed);
                    if (index >= tasks.size())
                        break;
                    TaskV10 const& task = tasks[index];
                    SimResultV7 sim = simulateActionV7(
                        worker,
                        parents[task.parent],
                        task.action,
                        startX
                    );

                    if (sim.dead) {
                        localDeaths.push_back(sim.deathX);
                        localFailed.push_back(task.failureKey);
                        ++localParentDead[task.parent];
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
                    }
                    completedTasks.fetch_add(1, std::memory_order_relaxed);
                }

                std::lock_guard<std::mutex> guard(resultMutex);
                produced.insert(
                    produced.end(),
                    std::make_move_iterator(localProduced.begin()),
                    std::make_move_iterator(localProduced.end())
                );
                deathXs.insert(deathXs.end(), localDeaths.begin(), localDeaths.end());
                failedKeys.insert(failedKeys.end(), localFailed.begin(), localFailed.end());
                for (size_t i = 0; i < parents.size(); ++i) {
                    parentMaxAlive[i] = std::max(parentMaxAlive[i], localParentMax[i]);
                    parentAlive[i] += localParentAlive[i];
                    parentDead[i] += localParentDead[i];
                }
            });
        }

        for (auto& thread : threads)
            thread.join();

        totalTrials += completedTasks.load(std::memory_order_relaxed);
        speculativeX = std::max(speculativeX, liveSeenX.load(std::memory_order_relaxed));
        for (uint64_t key : failedKeys)
            ++failureMemory[key];

        DeathClusterV9 cluster = dominantDeathClusterV9(deathXs, speculativeX);
        if (cluster.count > 0) {
            lastDeathX = cluster.x;
            lastClusterCount = cluster.count;
        } else if (!deathXs.empty()) {
            lastDeathX = *std::max_element(deathXs.begin(), deathXs.end());
            lastClusterCount = 1;
        } else {
            lastClusterCount = 0;
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
            if (node.x > seenNode.x + 0.5f || nodeBetterV7(node, seenNode))
                seenNode = node;
            speculativeX = std::max(speculativeX, node.x);

            if (deadEnd && node.x > focusX + kEscapeMarginV10) {
                if (!escaped || nodeBetterV7(node, escapedNode)) {
                    escapedNode = node;
                    escaped = true;
                }
            }
        }

        if (complete)
            break;

        bool safeAdvanced = false;
        if (!deadEnd) {
            SearchNodeV7 proven = safeNode;
            bool foundProven = false;
            for (size_t i = 0; i < parents.size(); ++i) {
                if (parentAlive[i] <= 0)
                    continue;
                float proofDistance = flightMode(parents[i].snapshot.p1.vehicle.type) ? 18.f : 24.f;
                if (parentMaxAlive[i] < parents[i].x + proofDistance)
                    continue;
                if (cluster.count > 0 && parents[i].x > cluster.x - 105.f)
                    continue;
                if (parents[i].x <= safeNode.x + 1.f)
                    continue;
                if (!foundProven || nodeBetterV7(parents[i], proven)) {
                    proven = parents[i];
                    foundProven = true;
                }
            }
            if (foundProven) {
                safeNode = proven;
                safeAdvanced = safeNode.x > safeAtLayerStart + 1.f;
            }
        }

        bool concentratedFailure =
            cluster.count >= 10 &&
            cluster.total > 0 &&
            cluster.count * 5 >= cluster.total * 2 &&
            cluster.x >= safeNode.x - 90.f &&
            cluster.x <= speculativeX + 280.f;

        if (!deadEnd) {
            stallLayers = safeAdvanced ? 0 : stallLayers + 1;
            frontier = selectFrontierV7(std::move(candidates));
            if (!frontier.empty()) {
                mergeArchiveV9(archive, frontier);
                if (archive.size() > kArchiveLimitV10)
                    archive.resize(kArchiveLimitV10);
            }

            bool enterDeadEnd =
                frontier.empty() ||
                concentratedFailure ||
                (stallLayers >= 2 && cluster.count >= 4);

            if (enterDeadEnd) {
                focusX = cluster.count > 0
                    ? cluster.x
                    : lastDeathX > 0.f ? lastDeathX : speculativeX;
                rollbackNow(
                    false,
                    cluster.count > 0
                        ? fmt::format("{} helpers converged on the same death wall", cluster.count)
                        : "No surviving frontier produced verifiable progress"
                );
                continue;
            }

            emit(
                safeAdvanced ? 3 : 1,
                safeAdvanced ? "Verified checkpoint advanced" : "Speculative progress held",
                safeAdvanced
                    ? "A living continuation proved the parent state; SAFE moved forward"
                    : "Farther X is only speculative until a child survives beyond its parent",
                frontier.empty() ? 0 : baseSegmentFramesV7(frontier.front().snapshot.p1.vehicle.type, precisionLevel, false),
                kLogicalHelpersV10
            );
            continue;
        }

        // Recovery path: SAFE is hard-locked. A rollback anchor can never
        // become a new checkpoint merely because it is farther than a previous
        // rollback anchor. Only crossing the old wall + margin unlocks progress.
        ++recoveryLayersAtDepth;
        if (escaped) {
            safeNode = escapedNode;
            deadEnd = false;
            deadEndLevel = 0;
            rollbackDistance = 0;
            recoveryLayersAtDepth = 0;
            stallLayers = 0;
            focusX = 0.f;
            if (precisionLevel > 2)
                --precisionLevel;

            frontier = selectFrontierV7(std::move(candidates));
            if (frontier.empty())
                frontier.push_back(safeNode);
            mergeArchiveV9(archive, frontier);
            reseedVisited();

            emit(
                3,
                "Dead end escaped: checkpoint unlocked",
                "A living route crossed the old death wall plus the escape margin; rollback progress is finally allowed to become SAFE",
                baseSegmentFramesV7(safeNode.snapshot.p1.vehicle.type, precisionLevel, false),
                kLogicalHelpersV10
            );
            continue;
        }

        frontier = selectFrontierV7(std::move(candidates));
        if (!frontier.empty())
            mergeArchiveV9(archive, frontier);

        bool sameWall =
            cluster.count >= 4 &&
            std::abs(cluster.x - focusX) <= kSameWallToleranceV10;
        bool wallDominates =
            sameWall &&
            cluster.total > 0 &&
            cluster.count * 3 >= cluster.total;

        // This is the anti-infinite-loop gear. A repeated wall immediately
        // increases rollback depth. Even without a clean cluster, two recovery
        // generations at one depth are the maximum before backing up farther.
        if (frontier.empty() || wallDominates || recoveryLayersAtDepth >= 2) {
            if (sameWall)
                focusX = focusX * 0.8f + cluster.x * 0.2f;
            rollbackNow(
                true,
                wallDominates
                    ? fmt::format("{} recovery helpers died in the same basin again", cluster.count)
                    : frontier.empty()
                        ? "The recovery frontier died out"
                        : "This rollback depth failed to escape after two generations"
            );
            continue;
        }

        emit(
            2,
            "Recovery still locked",
            fmt::format(
                "Rollback origin X {:.0f} is only a search base. SAFE cannot move until a survivor passes X {:.0f}",
                rollbackAnchorX,
                focusX + kEscapeMarginV10
            ),
            frontier.empty() ? 0 : baseSegmentFramesV7(frontier.front().snapshot.p1.vehicle.type, precisionLevel, false),
            kLogicalHelpersV10
        );
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
        << "solver=independent-100-helpers-10x10-v10"
        << " progress=" << result.progress
        << " safeX=" << safeNode.x
        << " speculativeX=" << speculativeX
        << " focusX=" << focusX
        << " rollbackAnchorX=" << rollbackAnchorX
        << " recoveryOriginSafeX=" << recoveryOriginSafeX
        << " rollbackDistance=" << rollbackDistance
        << " deadEndLevel=" << deadEndLevel
        << " endX=" << root.length
        << " layers=" << layer
        << " precisionLevel=" << precisionLevel
        << " recoveryCount=" << recoveryCount
        << " totalTrials=" << totalTrials
        << " failureMemory=" << failureMemory.size()
        << " logicalHelpers=" << kLogicalHelpersV10
        << " helperSquads=" << kHelperSquadsV10
        << " helpersPerSquad=" << kHelpersPerSquadV10
        << " physicalThreads=" << kPhysicalThreadsV10
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
            ? "The exported route still must reproduce the finish in a fresh replay"
            : "Dead-end and rollback-only positions were discarded instead of being promoted",
        0,
        0
    );
    return result;
}

} // namespace

PathfinderResult pathfind_v10(
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
                telemetry.recoveryReason = "100% stays locked until a fresh replay reproduces the finish";
            }
            callback(telemetry);
        };
    }

    PathfinderResult result = runIndependentHelpersV10(
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
