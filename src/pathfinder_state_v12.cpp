// V12: 100 logical helpers = 10 trainer teams x 10 helpers.
// Each team owns one evolving rally route. When a helper pushes that route
// farther, the whole team follows the new route next generation. Repeated
// deaths at the same wall cause only that team to revert/backtrack farther.
// The ten teams use different search personalities so one bad idea never
// collapses the whole solver into a single dead-end basin.
#define pathfind_v11 pathfind_v11_base_v12
#include "pathfinder_state_v11.cpp"
#undef pathfind_v11

namespace {

constexpr int kTrainerTeamsV12 = 10;
constexpr int kHelpersPerTeamV12 = 10;
constexpr int kLogicalHelpersV12 = 100;
constexpr int kPhysicalThreadsV12 = 30;
constexpr float kEscapeMarginV12 = 165.f;
constexpr float kSameWallToleranceV12 = 84.f;

static_assert(kTrainerTeamsV12 * kHelpersPerTeamV12 == kLogicalHelpersV12);

struct RallyTaskV12 {
    int team = 0;
    int helper = 0;
    MacroActionV7 action;
    uint64_t failureKey = 0;
};

struct RallyBatchV12 {
    std::vector<SearchNodeV7> produced;
    std::vector<float> deaths;
    std::vector<uint64_t> failedKeys;
    float maxAliveX = -std::numeric_limits<float>::infinity();
    int alive = 0;
    int dead = 0;
};

std::vector<MacroActionV7> rallyActionsV12(
    Level2& probe,
    SearchNodeV7 const& parent,
    TeamStateV11 const& team,
    int helper,
    uint32_t seed
) {
    int p = team.precisionLevel;

    // During a dead end, different teams deliberately attack the wall with
    // different mixes. This gives the solver ten competing recovery methods
    // instead of one global rollback policy copied one hundred times.
    if (team.deadEnd) {
        switch (team.id) {
            case 0:
                return helper < 8
                    ? rescueActionsV9(probe, parent.snapshot, std::max(4, p))
                    : guidedActionsV7(probe, parent.snapshot, std::max(4, p));
            case 1:
                return helper < 7
                    ? rescueActionsV9(probe, parent.snapshot, std::max(5, p + 1))
                    : explorerActionsV7(parent.snapshot, std::max(4, p), seed);
            case 2:
                return helper < 6
                    ? guidedActionsV7(probe, parent.snapshot, std::max(5, p + 1))
                    : rescueActionsV9(probe, parent.snapshot, std::max(5, p + 1));
            case 3:
                return helper < 3
                    ? rescueActionsV9(probe, parent.snapshot, std::max(4, p))
                    : explorerActionsV7(parent.snapshot, std::max(4, p), seed ^ 0x9e3779b9u);
            case 4:
                return helper < 2
                    ? guidedActionsV7(probe, parent.snapshot, std::max(4, p))
                    : explorerActionsV7(parent.snapshot, std::max(5, p + 1), seed ^ 0x85ebca6bu);
            case 5:
                return helper % 2 == 0
                    ? rescueActionsV9(probe, parent.snapshot, std::max(4, p))
                    : explorerActionsV7(parent.snapshot, std::max(4, p), seed);
            case 6:
                return helper < 7
                    ? rescueActionsV9(probe, parent.snapshot, std::max(6, p + 2))
                    : guidedActionsV7(probe, parent.snapshot, std::max(5, p + 1));
            case 7:
                return helper < 4
                    ? guidedActionsV7(probe, parent.snapshot, std::max(4, p))
                    : explorerActionsV7(parent.snapshot, std::max(4, p), seed ^ 0xc2b2ae35u);
            case 8:
                return helper < 5
                    ? rescueActionsV9(probe, parent.snapshot, std::max(5, p + 1))
                    : explorerActionsV7(parent.snapshot, std::max(5, p + 1), seed ^ 0x27d4eb2fu);
            default:
                if (helper < 3)
                    return guidedActionsV7(probe, parent.snapshot, std::max(5, p + 1));
                if (helper < 7)
                    return rescueActionsV9(probe, parent.snapshot, std::max(5, p + 1));
                return explorerActionsV7(parent.snapshot, std::max(5, p + 1), seed ^ 0x165667b1u);
        }
    }

    // Normal training personalities. Every helper in a team follows the same
    // rally route, but their timing/method differs so the team can improve it.
    return actionsForTeamV11(probe, parent, team, helper, seed);
}

std::vector<RallyTaskV12> buildRallyTasksV12(
    Level2& probe,
    TeamStateV11 const& team,
    SearchNodeV7 const& rallyNode,
    uint32_t teamSeed,
    int generation
) {
    std::vector<RallyTaskV12> tasks;
    tasks.reserve(kHelpersPerTeamV12);
    std::unordered_set<uint64_t> used;
    used.reserve(40);

    for (int helper = 0; helper < kHelpersPerTeamV12; ++helper) {
        uint32_t seed = teamSeed ^ static_cast<uint32_t>(generation * 104729u) ^
            static_cast<uint32_t>((helper + 1) * 3266489917u);
        auto actions = rallyActionsV12(probe, rallyNode, team, helper, seed);

        bool assigned = false;
        if (!actions.empty()) {
            size_t start = static_cast<size_t>(
                (helper * 13 + team.id * 19 + generation * 7) %
                static_cast<int>(actions.size())
            );
            for (size_t attempt = 0; attempt < actions.size(); ++attempt) {
                MacroActionV7 const& action = actions[(start + attempt) % actions.size()];
                uint64_t failureKey = coarseFailureKeyV9(rallyNode, action);
                int failures = 0;
                if (auto it = team.failureMemory.find(failureKey); it != team.failureMemory.end())
                    failures = it->second;
                if (failures >= (team.deadEnd ? 5 : 3))
                    continue;

                uint64_t assignment = assignmentKeyV10(rallyNode, action);
                if (!used.insert(assignment).second)
                    continue;
                tasks.push_back({team.id, helper, action, failureKey});
                assigned = true;
                break;
            }
        }

        // Keep the team at exactly ten active helpers even after familiar
        // timings have been learned as failures.
        for (int salt = 0; !assigned && salt < 64; ++salt) {
            int logicalHelper = team.id * kHelpersPerTeamV12 + helper;
            MacroActionV7 action = fallbackActionV10(
                rallyNode,
                logicalHelper,
                team.precisionLevel,
                generation * 97 + team.id * 31 + salt
            );
            uint64_t failureKey = coarseFailureKeyV9(rallyNode, action);
            int failures = 0;
            if (auto it = team.failureMemory.find(failureKey); it != team.failureMemory.end())
                failures = it->second;
            if (failures >= 7)
                continue;
            uint64_t assignment = assignmentKeyV10(rallyNode, action);
            if (!used.insert(assignment).second)
                continue;
            tasks.push_back({team.id, helper, std::move(action), failureKey});
            assigned = true;
        }

        if (!assigned) {
            int logicalHelper = team.id * kHelpersPerTeamV12 + helper;
            MacroActionV7 action = fallbackActionV10(
                rallyNode,
                logicalHelper,
                team.precisionLevel,
                generation * 997 + helper
            );
            tasks.push_back({
                team.id,
                helper,
                action,
                coarseFailureKeyV9(rallyNode, action)
            });
        }
    }

    return tasks;
}

SearchNodeV7 chooseRallyWinnerV12(
    SearchNodeV7 const& previous,
    std::vector<SearchNodeV7> const& candidates
) {
    SearchNodeV7 winner = previous;
    bool found = false;
    for (auto const& node : candidates) {
        if (!found || node.x > winner.x + 0.5f ||
            (std::abs(node.x - winner.x) <= 0.5f && nodeBetterV7(node, winner))) {
            winner = node;
            found = true;
        }
    }
    return found ? winner : previous;
}

PathfinderResult runRallyTrainerV12(
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
        root.lengthSource = "trusted-gd-v12";
    }

    SearchNodeV7 initial;
    initial.snapshot = captureSnapshotV7(root);
    initial.x = root.latestState().pos.x;
    initial.y = root.latestState().pos.y;
    initial.velocity = root.latestState().velocity;
    initial.minClearance = hazardClearance(root, root.latestState());
    initial.score = scoreStateV7(root, initial.snapshot, initial.minClearance, 0, startX);

    std::array<TeamStateV11, kTrainerTeamsV12> teams;
    std::array<SearchNodeV7, kTrainerTeamsV12> rallyNodes;
    std::array<uint32_t, kTrainerTeamsV12> teamSeeds {};
    std::array<int, kTrainerTeamsV12> sameWallRounds {};

    std::random_device rd;
    uint32_t baseSeed = rd() ^ static_cast<uint32_t>(std::hash<std::string>{}(lvlString));

    for (int i = 0; i < kTrainerTeamsV12; ++i) {
        TeamStateV11& team = teams[i];
        team.id = i;
        team.frontier = {initial};
        team.archive = {initial};
        team.safeNode = initial;
        team.seenNode = initial;
        team.solutionNode = initial;
        team.speculativeX = initial.x;
        team.recoveryOriginSafeX = initial.x;
        team.rollbackAnchorX = initial.x;
        team.visited[stateKeyV7(initial.snapshot, 0)] = initial.score;
        team.failureMemory.reserve(3000);
        rallyNodes[i] = initial;
        teamSeeds[i] = baseSeed ^ static_cast<uint32_t>((i + 1) * 0x9e3779b9u);
    }

    uint64_t totalTrials = 0;
    int generation = 0;
    int winningTeam = -1;

    auto emit = [&](int phase, std::string decision, std::string reason) {
        int bestIndex = bestTeamIndexV11(teams);
        TeamStateV11 const& best = teams[bestIndex];
        int recovering = 0;
        int totalFrontier = 0;
        int totalArchive = 0;
        int totalProduced = 0;
        int totalUnique = 0;
        int totalDead = 0;
        int totalDuplicate = 0;
        int totalRecoveries = 0;
        float globalSeen = initial.x;
        for (auto const& team : teams) {
            recovering += team.deadEnd ? 1 : 0;
            totalFrontier += static_cast<int>(team.frontier.size());
            totalArchive += static_cast<int>(team.archive.size());
            totalProduced += team.lastProduced;
            totalUnique += team.lastUnique;
            totalDead += team.lastDead;
            totalDuplicate += team.lastDuplicate;
            totalRecoveries += team.recoveryCount;
            globalSeen = std::max(globalSeen, team.speculativeX);
        }

        PathfinderTelemetry telemetry;
        telemetry.progress = progressFor(
            winningTeam >= 0 ? root.length : best.safeNode.x,
            startX,
            root.length,
            winningTeam >= 0
        );
        telemetry.startX = startX;
        telemetry.currentX = best.safeNode.x;
        telemetry.furthestX = globalSeen;
        telemetry.trustedEndX = hasTrustedEnd ? trustedEndX : 0.f;
        telemetry.inferredLength = inferredLength;
        telemetry.checkpointX = best.safeNode.x;
        telemetry.deathX = best.lastDeathX;
        telemetry.deathProgress = static_cast<float>(
            progressFor(best.lastDeathX, startX, root.length, false)
        );
        telemetry.bestClearance = best.safeNode.minClearance;
        telemetry.focusX = best.focusX;
        telemetry.frame = best.safeNode.snapshot.p1.frame;
        telemetry.checkpointFrame = best.safeNode.snapshot.p1.frame;
        telemetry.vehicleType = static_cast<int>(best.safeNode.snapshot.p1.vehicle.type);
        telemetry.searchLevel = best.precisionLevel;
        telemetry.horizonFrames = baseSegmentFramesV7(
            best.safeNode.snapshot.p1.vehicle.type,
            best.precisionLevel,
            false
        );
        telemetry.candidateCount = kLogicalHelpersV12;
        telemetry.workerCount = kLogicalHelpersV12;
        telemetry.physicalThreadCount = kPhysicalThreadsV12;
        telemetry.phase = phase;
        telemetry.frontierCount = totalFrontier;
        telemetry.guidedCount = 50;
        telemetry.explorerCount = 50;
        telemetry.archiveCount = totalArchive;
        telemetry.producedCount = totalProduced;
        telemetry.uniqueCount = totalUnique;
        telemetry.deadCount = totalDead;
        telemetry.duplicateCount = totalDuplicate;
        telemetry.stallLayers = best.stallLayers;
        telemetry.recoveryCount = totalRecoveries;
        telemetry.deathClusterCount = best.lastClusterCount;
        telemetry.rollbackDistance = best.rollbackDistance;
        telemetry.deadEndLevel = best.deadEndLevel;
        telemetry.stallRescue = recovering > 0;
        telemetry.progressLocked = recovering == kTrainerTeamsV12;
        telemetry.totalTrials = totalTrials;
        telemetry.mode = "100-helpers-10-rally-teams-v12";
        telemetry.decision = std::move(decision);
        telemetry.recoveryReason = fmt::format(
            "{} | leading team {} safe {:.0f}, rally {:.0f}; {} teams recovering",
            reason,
            bestIndex + 1,
            best.safeNode.x,
            rallyNodes[bestIndex].x,
            recovering
        );
        publishPathfinderTelemetryV8(telemetry);
        if (callback)
            callback(telemetry);
    };

    emit(
        0,
        "100 helpers formed 10 rally teams",
        "Each team has 10 helpers following one learned route; teams only revert their own route after repeated deaths"
    );

    while (!stop.load() && winningTeam < 0) {
        ++generation;
        Level2 actionProbe = compactWorkerV7(root);
        std::vector<RallyTaskV12> tasks;
        tasks.reserve(kLogicalHelpersV12);

        for (int t = 0; t < kTrainerTeamsV12; ++t) {
            auto teamTasks = buildRallyTasksV12(
                actionProbe,
                teams[t],
                rallyNodes[t],
                teamSeeds[t],
                generation
            );
            tasks.insert(
                tasks.end(),
                std::make_move_iterator(teamTasks.begin()),
                std::make_move_iterator(teamTasks.end())
            );
        }

        if (tasks.empty())
            break;

        emit(
            1,
            "Teams following their learned routes",
            "When a team finds farther living progress, all ten of its helpers rally around that new path next generation"
        );

        std::atomic<size_t> nextTask {0};
        std::mutex resultMutex;
        std::array<RallyBatchV12, kTrainerTeamsV12> batches;

        int threadCount = std::min<int>(kPhysicalThreadsV12, static_cast<int>(tasks.size()));
        std::vector<std::thread> threads;
        threads.reserve(threadCount);

        for (int threadIndex = 0; threadIndex < threadCount; ++threadIndex) {
            threads.emplace_back([&, threadIndex] {
                Level2 worker = compactWorkerV7(root);
                std::array<RallyBatchV12, kTrainerTeamsV12> local;

                while (!stop.load()) {
                    size_t index = nextTask.fetch_add(1, std::memory_order_relaxed);
                    if (index >= tasks.size())
                        break;
                    RallyTaskV12 const& task = tasks[index];
                    SearchNodeV7 const& parent = rallyNodes[task.team];
                    SimResultV7 sim = simulateActionV7(worker, parent, task.action, startX);
                    RallyBatchV12& batch = local[task.team];

                    if (sim.dead) {
                        ++batch.dead;
                        batch.deaths.push_back(sim.deathX);
                        batch.failedKeys.push_back(task.failureKey);
                    } else {
                        ++batch.alive;
                        batch.maxAliveX = std::max(batch.maxAliveX, sim.node.x);
                        batch.produced.push_back(std::move(sim.node));
                    }
                }

                std::lock_guard<std::mutex> guard(resultMutex);
                for (int t = 0; t < kTrainerTeamsV12; ++t) {
                    RallyBatchV12& dst = batches[t];
                    RallyBatchV12& src = local[t];
                    dst.alive += src.alive;
                    dst.dead += src.dead;
                    dst.maxAliveX = std::max(dst.maxAliveX, src.maxAliveX);
                    dst.produced.insert(
                        dst.produced.end(),
                        std::make_move_iterator(src.produced.begin()),
                        std::make_move_iterator(src.produced.end())
                    );
                    dst.deaths.insert(dst.deaths.end(), src.deaths.begin(), src.deaths.end());
                    dst.failedKeys.insert(dst.failedKeys.end(), src.failedKeys.begin(), src.failedKeys.end());
                }
            });
        }

        for (auto& thread : threads)
            thread.join();

        for (int t = 0; t < kTrainerTeamsV12; ++t) {
            TeamStateV11& team = teams[t];
            RallyBatchV12& batch = batches[t];
            totalTrials += static_cast<uint64_t>(batch.alive + batch.dead);
            team.trials += static_cast<uint64_t>(batch.alive + batch.dead);
            for (uint64_t key : batch.failedKeys)
                ++team.failureMemory[key];

            DeathClusterV9 cluster = dominantDeathClusterV9(batch.deaths, team.speculativeX);
            if (cluster.count > 0) {
                team.lastDeathX = cluster.x;
                team.lastClusterCount = cluster.count;
            } else if (!batch.deaths.empty()) {
                team.lastDeathX = *std::max_element(batch.deaths.begin(), batch.deaths.end());
                team.lastClusterCount = 1;
            } else {
                team.lastClusterCount = 0;
            }

            std::unordered_map<StateKeyV7, SearchNodeV7, StateKeyHashV7> unique;
            unique.reserve(batch.produced.size());
            int duplicate = 0;
            for (auto& node : batch.produced) {
                StateKeyV7 key = stateKeyV7(node.snapshot, team.precisionLevel);
                auto it = unique.find(key);
                if (it == unique.end() || nodeBetterV7(node, it->second))
                    unique[key] = std::move(node);
                else
                    ++duplicate;
            }

            std::vector<SearchNodeV7> candidates;
            candidates.reserve(unique.size());
            for (auto& [_, node] : unique)
                candidates.push_back(std::move(node));
            std::sort(candidates.begin(), candidates.end(), nodeBetterV7);

            team.lastProduced = static_cast<int>(batch.produced.size());
            team.lastUnique = static_cast<int>(candidates.size());
            team.lastDead = batch.dead;
            team.lastDuplicate = duplicate;

            SearchNodeV7 previousRally = rallyNodes[t];
            SearchNodeV7 newRally = chooseRallyWinnerV12(previousRally, candidates);
            bool rallyAdvanced = newRally.x > previousRally.x + 0.5f;

            for (auto const& node : candidates) {
                if (node.complete) {
                    team.solutionNode = node;
                    team.complete = true;
                    winningTeam = t;
                    break;
                }
                team.speculativeX = std::max(team.speculativeX, node.x);
                if (node.x > team.seenNode.x + 0.5f || nodeBetterV7(node, team.seenNode))
                    team.seenNode = node;
            }
            if (winningTeam >= 0)
                break;

            if (!team.deadEnd) {
                // A rally node becomes SAFE only after its children demonstrate
                // that it can continue. The new child is the next route the ten
                // helpers follow, but it remains speculative for one generation.
                if (batch.alive > 0 && batch.maxAliveX >= previousRally.x +
                    (flightMode(previousRally.snapshot.p1.vehicle.type) ? 18.f : 24.f)) {
                    if (previousRally.x > team.safeNode.x + 1.f)
                        team.safeNode = previousRally;
                }

                if (rallyAdvanced) {
                    rallyNodes[t] = newRally;
                    team.frontier = {newRally};
                    mergeArchiveV9(team.archive, {newRally, team.safeNode});
                    trimTeamStateV11(team);
                    team.stallLayers = 0;
                    sameWallRounds[t] = 0;
                    continue;
                }

                ++team.stallLayers;
                bool sameWall =
                    cluster.count >= 4 &&
                    std::abs(cluster.x - team.lastDeathX) <= kSameWallToleranceV12;
                sameWallRounds[t] = sameWall ? sameWallRounds[t] + 1 : 0;

                bool mostlyDead = batch.dead >= 6;
                if (candidates.empty() || mostlyDead || sameWallRounds[t] >= 2 || team.stallLayers >= 3) {
                    team.focusX = cluster.count > 0
                        ? cluster.x
                        : team.lastDeathX > 0.f ? team.lastDeathX : previousRally.x;
                    rollbackTeamV11(team, initial, false);
                    rallyNodes[t] = team.frontier.empty() ? team.safeNode : team.frontier.front();
                    sameWallRounds[t] = 0;
                }
                continue;
            }

            // Recovery is team-local. Living children may become the team's
            // rally route, but SAFE stays locked until the old wall is cleared.
            ++team.recoveryLayersAtDepth;
            bool escaped = false;
            SearchNodeV7 escapedNode = rallyNodes[t];
            for (auto const& node : candidates) {
                if (node.x > team.focusX + kEscapeMarginV12) {
                    if (!escaped || nodeBetterV7(node, escapedNode)) {
                        escaped = true;
                        escapedNode = node;
                    }
                }
            }

            if (escaped) {
                rallyNodes[t] = escapedNode;
                team.safeNode = escapedNode;
                team.frontier = {escapedNode};
                team.deadEnd = false;
                team.deadEndLevel = 0;
                team.rollbackDistance = 0;
                team.recoveryLayersAtDepth = 0;
                team.stallLayers = 0;
                team.focusX = 0.f;
                sameWallRounds[t] = 0;
                if (team.precisionLevel > 2)
                    --team.precisionLevel;
                mergeArchiveV9(team.archive, {escapedNode});
                trimTeamStateV11(team);
                reseedTeamVisitedV11(team);
                continue;
            }

            if (rallyAdvanced)
                rallyNodes[t] = newRally;

            bool sameWall =
                cluster.count >= 3 &&
                std::abs(cluster.x - team.focusX) <= kSameWallToleranceV12;
            sameWallRounds[t] = sameWall ? sameWallRounds[t] + 1 : 0;

            // Every repeated failure at the same spot increases the rollback
            // depth. That prevents rollback -> fake checkpoint -> same death.
            bool deepen =
                candidates.empty() ||
                batch.dead >= 7 ||
                sameWallRounds[t] >= 1 ||
                team.recoveryLayersAtDepth >= 2;

            if (deepen) {
                if (sameWall)
                    team.focusX = team.focusX * 0.8f + cluster.x * 0.2f;
                rollbackTeamV11(team, initial, true);
                rallyNodes[t] = team.frontier.empty() ? team.safeNode : team.frontier.front();
                sameWallRounds[t] = 0;
            } else if (!candidates.empty()) {
                team.frontier = {rallyNodes[t]};
                mergeArchiveV9(team.archive, {rallyNodes[t]});
                trimTeamStateV11(team);
            }
        }

        if (winningTeam >= 0)
            break;

        emit(
            1,
            "10 teams made separate route decisions",
            "Each team rallied around its own best living child; teams repeatedly dying at one wall reverted farther while the others kept training"
        );
    }

    int bestIndex = winningTeam >= 0 ? winningTeam : bestTeamIndexV11(teams);
    TeamStateV11 const& best = teams[bestIndex];
    SearchNodeV7 const& outputNode = winningTeam >= 0 ? best.solutionNode : best.safeNode;

    PathfinderResult result;
    result.inputs = routeToInputsV7(outputNode.route);
    result.macro = inputsToMacroV7(result.inputs);
    result.complete = winningTeam >= 0;
    result.progress = progressFor(
        result.complete ? root.length : best.safeNode.x,
        startX,
        root.length,
        result.complete
    );

    std::ostringstream diagnostics;
    diagnostics
        << "solver=100-helpers-10-rally-teams-v12"
        << " progress=" << result.progress
        << " winningTeam=" << (winningTeam >= 0 ? winningTeam + 1 : 0)
        << " bestTeam=" << bestIndex + 1
        << " bestSafeX=" << best.safeNode.x
        << " bestRallyX=" << rallyNodes[bestIndex].x
        << " generations=" << generation
        << " totalTrials=" << totalTrials
        << " teams=" << kTrainerTeamsV12
        << " helpersPerTeam=" << kHelpersPerTeamV12
        << " logicalHelpers=" << kLogicalHelpersV12
        << " physicalThreads=" << kPhysicalThreadsV12
        << " prunedNoTouch=" << prunedNoTouch
        << " inputs=" << result.inputs.size()
        << " complete=" << (result.complete ? 1 : 0)
        << " stopped=" << (stop.load() ? 1 : 0);

    for (int i = 0; i < kTrainerTeamsV12; ++i) {
        diagnostics
            << " team" << (i + 1) << "SafeX=" << teams[i].safeNode.x
            << " team" << (i + 1) << "RallyX=" << rallyNodes[i].x
            << " team" << (i + 1) << "DeadEnd=" << (teams[i].deadEnd ? 1 : 0)
            << " team" << (i + 1) << "Rollbacks=" << teams[i].recoveryCount;
    }
    result.diagnostics = diagnostics.str();

    emit(
        result.complete ? 4 : 2,
        result.complete
            ? fmt::format("Team {} found a candidate clear", winningTeam + 1)
            : "Stopped on the best verified team route",
        result.complete
            ? "The winning team's route still must reproduce the finish in a fresh replay"
            : "Rollback origins and speculative rally points were not accepted as earned progress"
    );

    return result;
}

} // namespace

PathfinderResult pathfind_v12(
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
                telemetry.decision = "Validating winning team clear";
                telemetry.recoveryReason = "100% stays locked until the winning route reproduces the finish";
            }
            callback(telemetry);
        };
    }

    PathfinderResult result = runRallyTrainerV12(
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
