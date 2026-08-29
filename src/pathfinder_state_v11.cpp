// V11 turns the solver into ten genuinely independent trainer teams.
// Each team owns ten helpers, its own frontier/archive/failure memory, its own
// checkpoint history, and its own dead-end rollback depth. A bad route learned
// by one team cannot drag the other nine into the same basin. The only shared
// pieces are the immutable level simulator and the final race to a verified clear.
#define pathfind_v10 pathfind_v10_base_v11
#include "pathfinder_state_v10.cpp"
#undef pathfind_v10

namespace {

constexpr int kTrainerTeamsV11 = 10;
constexpr int kHelpersPerTeamV11 = 10;
constexpr int kLogicalHelpersV11 = kTrainerTeamsV11 * kHelpersPerTeamV11;
constexpr int kPhysicalThreadsV11 = 30;
constexpr int kTeamFrontierLimitV11 = 24;
constexpr int kTeamArchiveLimitV11 = 180;
constexpr float kEscapeMarginV11 = 165.f;
constexpr float kSameWallToleranceV11 = 84.f;

static_assert(kLogicalHelpersV11 == 100);

struct TeamStateV11 {
    int id = 0;
    std::vector<SearchNodeV7> frontier;
    std::vector<SearchNodeV7> archive;
    SearchNodeV7 safeNode;
    SearchNodeV7 seenNode;
    SearchNodeV7 solutionNode;

    std::unordered_map<StateKeyV7, float, StateKeyHashV7> visited;
    std::unordered_map<uint64_t, int> failureMemory;

    float speculativeX = 0.f;
    float focusX = 0.f;
    float lastDeathX = 0.f;
    float recoveryOriginSafeX = 0.f;
    float rollbackAnchorX = 0.f;

    uint64_t trials = 0;
    int precisionLevel = 0;
    int stallLayers = 0;
    int recoveryCount = 0;
    int deadEndLevel = 0;
    int rollbackDistance = 0;
    int recoveryLayersAtDepth = 0;
    int lastProduced = 0;
    int lastUnique = 0;
    int lastDead = 0;
    int lastDuplicate = 0;
    int lastClusterCount = 0;

    bool deadEnd = false;
    bool complete = false;
};

struct TaskV11 {
    int team = 0;
    int helper = 0;
    size_t parent = 0;
    MacroActionV7 action;
    uint64_t failureKey = 0;
};

struct TeamBatchV11 {
    std::vector<SearchNodeV7> produced;
    std::vector<SearchNodeV7> provenParents;
    std::vector<float> deaths;
    std::vector<uint64_t> failedKeys;
    uint64_t taskCount = 0;
};

void trimTeamStateV11(TeamStateV11& team) {
    if (team.frontier.size() > kTeamFrontierLimitV11)
        team.frontier.resize(kTeamFrontierLimitV11);
    if (team.archive.size() > kTeamArchiveLimitV11)
        team.archive.resize(kTeamArchiveLimitV11);
}

void reseedTeamVisitedV11(TeamStateV11& team) {
    team.visited.clear();
    for (auto const& node : team.frontier)
        team.visited[stateKeyV7(node.snapshot, team.precisionLevel)] = node.score;
}

std::vector<SearchNodeV7> teamParentPoolV11(
    TeamStateV11 const& team,
    SearchNodeV7 const& initial
) {
    if (team.deadEnd) {
        auto rolled = rollbackFrontierV9(
            team.archive,
            team.frontier,
            initial,
            team.focusX,
            team.rollbackDistance
        );
        if (!rolled.empty()) {
            if (rolled.size() > kTeamFrontierLimitV11)
                rolled.resize(kTeamFrontierLimitV11);
            return rolled;
        }
    }

    std::vector<SearchNodeV7> pool = team.frontier;
    pool.insert(pool.end(), team.archive.begin(), team.archive.end());
    if (pool.empty())
        pool.push_back(initial);
    std::sort(pool.begin(), pool.end(), nodeBetterV7);

    std::unordered_set<CoarseKeyV7, CoarseKeyHashV7> seen;
    std::vector<SearchNodeV7> unique;
    unique.reserve(std::min<size_t>(pool.size(), 96));
    for (auto const& node : pool) {
        CoarseKeyV7 key = coarseKeyV7(node);
        if (!seen.insert(key).second)
            continue;
        unique.push_back(node);
        if (unique.size() >= 96)
            break;
    }
    if (unique.empty())
        unique.push_back(initial);
    return unique;
}

std::vector<MacroActionV7> actionsForTeamV11(
    Level2& probe,
    SearchNodeV7 const& parent,
    TeamStateV11 const& team,
    int helper,
    uint32_t seed
) {
    int precision = team.precisionLevel;

    // A recovering team changes its own local strategy only. The other teams
    // keep training on their unrelated routes and never inherit this rollback.
    if (team.deadEnd) {
        if (helper < 6)
            return rescueActionsV9(probe, parent.snapshot, std::max(4, precision));
        return explorerActionsV7(
            parent.snapshot,
            std::max(4, precision),
            seed ^ static_cast<uint32_t>((helper + 1) * 2654435761u)
        );
    }

    // Each team has a stable training personality. Helpers inside that team
    // vote by survival: the team's next frontier is selected only from its own
    // ten results. No team copies another team's current best prefix.
    switch (team.id) {
        case 0: // conservative guided route
            return helper < 8
                ? guidedActionsV7(probe, parent.snapshot, precision)
                : explorerActionsV7(parent.snapshot, precision, seed);
        case 1: // earlier timing / rescue-biased route
            return helper < 7
                ? rescueActionsV9(probe, parent.snapshot, precision + 1)
                : explorerActionsV7(parent.snapshot, precision + 1, seed);
        case 2: // precision-guided route
            return helper < 8
                ? guidedActionsV7(probe, parent.snapshot, precision + 2)
                : rescueActionsV9(probe, parent.snapshot, precision + 2);
        case 3: // exploration-heavy route
            return helper < 3
                ? guidedActionsV7(probe, parent.snapshot, precision)
                : explorerActionsV7(parent.snapshot, precision + 1, seed);
        case 4: // high-variance explorer route
            return helper < 2
                ? rescueActionsV9(probe, parent.snapshot, precision + 1)
                : explorerActionsV7(parent.snapshot, precision + 2, seed ^ 0x9e3779b9u);
        case 5: // balanced route
            return (helper % 2 == 0)
                ? guidedActionsV7(probe, parent.snapshot, precision + 1)
                : explorerActionsV7(parent.snapshot, precision + 1, seed);
        case 6: // timing-search route
            return helper < 6
                ? rescueActionsV9(probe, parent.snapshot, precision + 2)
                : guidedActionsV7(probe, parent.snapshot, precision + 1);
        case 7: // archive-diversity explorer route
            return helper < 4
                ? guidedActionsV7(probe, parent.snapshot, precision)
                : explorerActionsV7(parent.snapshot, precision + (helper & 1), seed ^ 0x85ebca6bu);
        case 8: // rescue + exploration hybrid
            return helper < 5
                ? rescueActionsV9(probe, parent.snapshot, precision + 1)
                : explorerActionsV7(parent.snapshot, precision + 1, seed ^ 0xc2b2ae35u);
        default: // intentionally mixed wildcard route
            if (helper < 3)
                return guidedActionsV7(probe, parent.snapshot, precision + 1);
            if (helper < 6)
                return rescueActionsV9(probe, parent.snapshot, precision + 1);
            return explorerActionsV7(parent.snapshot, precision + 2, seed ^ 0x27d4eb2fu);
    }
}

std::vector<TaskV11> buildTeamTasksV11(
    Level2& probe,
    TeamStateV11 const& team,
    std::vector<SearchNodeV7> const& parents,
    uint32_t teamSeed,
    int generation
) {
    std::vector<TaskV11> tasks;
    tasks.reserve(kHelpersPerTeamV11);
    if (parents.empty())
        return tasks;

    std::unordered_set<uint64_t> used;
    used.reserve(40);

    for (int helper = 0; helper < kHelpersPerTeamV11; ++helper) {
        size_t parentIndex = static_cast<size_t>(
            (helper * 7 + team.id * 13 + generation * (team.id + 3)) %
            static_cast<int>(parents.size())
        );
        SearchNodeV7 const& parent = parents[parentIndex];

        uint32_t seed = teamSeed ^ static_cast<uint32_t>(generation * 104729u) ^
            static_cast<uint32_t>((helper + 1) * 3266489917u);
        auto actions = actionsForTeamV11(probe, parent, team, helper, seed);

        bool assigned = false;
        if (!actions.empty()) {
            size_t start = static_cast<size_t>(
                (helper * 11 + team.id * 17 + generation * 5) %
                static_cast<int>(actions.size())
            );
            for (size_t attempt = 0; attempt < actions.size(); ++attempt) {
                MacroActionV7 const& action = actions[(start + attempt) % actions.size()];
                uint64_t failureKey = coarseFailureKeyV9(parent, action);
                int failures = 0;
                if (auto it = team.failureMemory.find(failureKey); it != team.failureMemory.end())
                    failures = it->second;
                int banThreshold = helper >= 6 ? 5 : 3;
                if (failures >= banThreshold)
                    continue;

                uint64_t assignment = assignmentKeyV10(parent, action);
                if (!used.insert(assignment).second)
                    continue;
                tasks.push_back({team.id, helper, parentIndex, action, failureKey});
                assigned = true;
                break;
            }
        }

        // Never collapse a team from ten helpers to five because its catalogue
        // was exhausted. Synthesize a fresh timing lane for that specific team.
        for (int salt = 0; !assigned && salt < 64; ++salt) {
            int logicalHelper = team.id * kHelpersPerTeamV11 + helper;
            MacroActionV7 action = fallbackActionV10(
                parent,
                logicalHelper,
                team.precisionLevel,
                generation * 97 + team.id * 31 + salt
            );
            uint64_t failureKey = coarseFailureKeyV9(parent, action);
            int failures = 0;
            if (auto it = team.failureMemory.find(failureKey); it != team.failureMemory.end())
                failures = it->second;
            if (failures >= 6)
                continue;
            uint64_t assignment = assignmentKeyV10(parent, action);
            if (!used.insert(assignment).second)
                continue;
            tasks.push_back({team.id, helper, parentIndex, std::move(action), failureKey});
            assigned = true;
        }

        if (!assigned) {
            int logicalHelper = team.id * kHelpersPerTeamV11 + helper;
            MacroActionV7 action = fallbackActionV10(
                parent,
                logicalHelper,
                team.precisionLevel,
                generation * 997 + helper
            );
            tasks.push_back({
                team.id,
                helper,
                parentIndex,
                action,
                coarseFailureKeyV9(parent, action)
            });
        }
    }

    return tasks;
}

void rollbackTeamV11(
    TeamStateV11& team,
    SearchNodeV7 const& initial,
    bool deepen
) {
    if (!team.deadEnd) {
        team.deadEnd = true;
        team.deadEndLevel = 0;
        team.recoveryOriginSafeX = team.safeNode.x;
    } else if (deepen) {
        ++team.deadEndLevel;
    }

    team.rollbackDistance = rollbackDistanceV9(team.deadEndLevel);
    ++team.recoveryCount;
    ++team.precisionLevel;
    team.precisionLevel = std::clamp(team.precisionLevel, 4, 6);
    team.recoveryLayersAtDepth = 0;

    quarantineDeadEndV9(team.archive, team.focusX);
    SearchNodeV7 anchor = rollbackAnchorV9(
        team.archive,
        team.frontier,
        initial,
        team.focusX,
        team.rollbackDistance
    );
    team.rollbackAnchorX = anchor.x;

    // The rollback anchor is only this team's new search base. It is never
    // proof of progress. Repeated failure deepens this team's rollback while
    // all other teams continue from their own independent histories.
    team.safeNode = anchor;
    team.frontier = rollbackFrontierV9(
        team.archive,
        team.frontier,
        initial,
        team.focusX,
        team.rollbackDistance
    );
    if (team.frontier.empty())
        team.frontier.push_back(anchor);
    mergeArchiveV9(team.archive, {anchor});
    trimTeamStateV11(team);
    reseedTeamVisitedV11(team);
}

int bestTeamIndexV11(std::array<TeamStateV11, kTrainerTeamsV11> const& teams) {
    int best = 0;
    for (int i = 1; i < kTrainerTeamsV11; ++i) {
        if (teams[i].complete && !teams[best].complete) {
            best = i;
            continue;
        }
        if (teams[i].complete == teams[best].complete) {
            if (teams[i].safeNode.x > teams[best].safeNode.x + 0.5f ||
                (std::abs(teams[i].safeNode.x - teams[best].safeNode.x) <= 0.5f &&
                 nodeBetterV7(teams[i].safeNode, teams[best].safeNode))) {
                best = i;
            }
        }
    }
    return best;
}

PathfinderResult runTrainerTeamsV11(
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
        root.lengthSource = "trusted-gd-v11";
    }

    SearchNodeV7 initial;
    initial.snapshot = captureSnapshotV7(root);
    initial.x = root.latestState().pos.x;
    initial.y = root.latestState().pos.y;
    initial.velocity = root.latestState().velocity;
    initial.minClearance = hazardClearance(root, root.latestState());
    initial.score = scoreStateV7(root, initial.snapshot, initial.minClearance, 0, startX);

    std::array<TeamStateV11, kTrainerTeamsV11> teams;
    std::array<uint32_t, kTrainerTeamsV11> teamSeeds {};
    std::random_device rd;
    uint32_t baseSeed = rd() ^ static_cast<uint32_t>(std::hash<std::string>{}(lvlString));

    for (int i = 0; i < kTrainerTeamsV11; ++i) {
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
        team.failureMemory.reserve(2600);
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
        telemetry.candidateCount = kLogicalHelpersV11;
        telemetry.workerCount = kLogicalHelpersV11;
        telemetry.physicalThreadCount = kPhysicalThreadsV11;
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
        telemetry.progressLocked = recovering == kTrainerTeamsV11;
        telemetry.totalTrials = totalTrials;
        telemetry.mode = "ten-independent-trainer-teams-10x10-v11";
        telemetry.decision = std::move(decision);
        telemetry.recoveryReason = fmt::format(
            "{} | leading team {} at safe X {:.0f}; {} of 10 teams currently recovering",
            reason,
            bestIndex + 1,
            best.safeNode.x,
            recovering
        );
        publishPathfinderTelemetryV8(telemetry);
        if (callback)
            callback(telemetry);
    };

    emit(
        0,
        "10 independent trainer teams launched",
        "Each team owns 10 helpers, a private route history, private failure memory, and private rollback state"
    );

    while (!stop.load() && winningTeam < 0) {
        ++generation;

        std::array<std::vector<SearchNodeV7>, kTrainerTeamsV11> parentsByTeam;
        std::vector<TaskV11> tasks;
        tasks.reserve(kLogicalHelpersV11);
        Level2 actionProbe = compactWorkerV7(root);

        for (int teamIndex = 0; teamIndex < kTrainerTeamsV11; ++teamIndex) {
            TeamStateV11& team = teams[teamIndex];
            if (team.complete)
                continue;

            parentsByTeam[teamIndex] = teamParentPoolV11(team, initial);
            if (parentsByTeam[teamIndex].empty())
                parentsByTeam[teamIndex].push_back(initial);

            auto teamTasks = buildTeamTasksV11(
                actionProbe,
                team,
                parentsByTeam[teamIndex],
                teamSeeds[teamIndex],
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
            "10 teams training on separate pathways",
            "A team's deaths and rollback decisions stay inside that team; the other nine continue their own searches"
        );

        std::atomic<size_t> nextTask {0};
        std::mutex resultMutex;
        std::array<TeamBatchV11, kTrainerTeamsV11> batches;

        int threadCount = std::min<int>(kPhysicalThreadsV11, static_cast<int>(tasks.size()));
        std::vector<std::thread> threads;
        threads.reserve(threadCount);

        for (int threadIndex = 0; threadIndex < threadCount; ++threadIndex) {
            threads.emplace_back([&, threadIndex] {
                Level2 worker = compactWorkerV7(root);
                std::array<TeamBatchV11, kTrainerTeamsV11> local;

                while (!stop.load()) {
                    size_t taskIndex = nextTask.fetch_add(1, std::memory_order_relaxed);
                    if (taskIndex >= tasks.size())
                        break;

                    TaskV11 const& task = tasks[taskIndex];
                    SearchNodeV7 const& parent = parentsByTeam[task.team][task.parent];
                    SimResultV7 sim = simulateActionV7(worker, parent, task.action, startX);
                    TeamBatchV11& batch = local[task.team];
                    ++batch.taskCount;

                    if (sim.dead) {
                        batch.deaths.push_back(sim.deathX);
                        batch.failedKeys.push_back(task.failureKey);
                    } else {
                        float proofDistance = flightMode(parent.snapshot.p1.vehicle.type) ? 18.f : 24.f;
                        if (sim.node.x >= parent.x + proofDistance)
                            batch.provenParents.push_back(parent);
                        batch.produced.push_back(std::move(sim.node));
                    }
                }

                std::lock_guard<std::mutex> guard(resultMutex);
                for (int i = 0; i < kTrainerTeamsV11; ++i) {
                    TeamBatchV11& dst = batches[i];
                    TeamBatchV11& src = local[i];
                    dst.taskCount += src.taskCount;
                    dst.produced.insert(
                        dst.produced.end(),
                        std::make_move_iterator(src.produced.begin()),
                        std::make_move_iterator(src.produced.end())
                    );
                    dst.provenParents.insert(
                        dst.provenParents.end(),
                        std::make_move_iterator(src.provenParents.begin()),
                        std::make_move_iterator(src.provenParents.end())
                    );
                    dst.deaths.insert(dst.deaths.end(), src.deaths.begin(), src.deaths.end());
                    dst.failedKeys.insert(dst.failedKeys.end(), src.failedKeys.begin(), src.failedKeys.end());
                }
            });
        }

        for (auto& thread : threads)
            thread.join();

        for (int teamIndex = 0; teamIndex < kTrainerTeamsV11; ++teamIndex) {
            TeamStateV11& team = teams[teamIndex];
            TeamBatchV11& batch = batches[teamIndex];
            if (batch.taskCount == 0)
                continue;

            team.trials += batch.taskCount;
            totalTrials += batch.taskCount;
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
            int visitedRejected = 0;
            for (auto& node : batch.produced) {
                StateKeyV7 key = stateKeyV7(node.snapshot, team.precisionLevel);
                auto visitedIt = team.visited.find(key);
                if (visitedIt != team.visited.end() && visitedIt->second >= node.score - 0.01f) {
                    ++visitedRejected;
                    continue;
                }
                team.visited[key] = node.score;
                auto it = unique.find(key);
                if (it == unique.end() || nodeBetterV7(node, it->second))
                    unique[key] = std::move(node);
            }

            std::vector<SearchNodeV7> candidates;
            candidates.reserve(unique.size());
            for (auto& [_, node] : unique)
                candidates.push_back(std::move(node));

            team.lastProduced = static_cast<int>(batch.produced.size());
            team.lastUnique = static_cast<int>(candidates.size());
            team.lastDead = static_cast<int>(batch.deaths.size());
            team.lastDuplicate = std::max(
                0,
                team.lastProduced - team.lastUnique + visitedRejected
            );

            SearchNodeV7 escapedNode;
            bool escaped = false;
            for (auto const& node : candidates) {
                if (node.complete) {
                    if (!team.complete || nodeBetterV7(node, team.solutionNode))
                        team.solutionNode = node;
                    team.complete = true;
                }
                if (node.x > team.seenNode.x + 0.5f || nodeBetterV7(node, team.seenNode))
                    team.seenNode = node;
                team.speculativeX = std::max(team.speculativeX, node.x);

                if (team.deadEnd && node.x > team.focusX + kEscapeMarginV11) {
                    if (!escaped || nodeBetterV7(node, escapedNode)) {
                        escapedNode = node;
                        escaped = true;
                    }
                }
            }

            if (team.complete) {
                if (winningTeam < 0 ||
                    nodeBetterV7(team.solutionNode, teams[winningTeam].solutionNode)) {
                    winningTeam = teamIndex;
                }
                continue;
            }

            bool safeAdvanced = false;
            if (!team.deadEnd) {
                SearchNodeV7 proven = team.safeNode;
                bool foundProven = false;
                for (auto const& parent : batch.provenParents) {
                    if (cluster.count > 0 && parent.x > cluster.x - 105.f)
                        continue;
                    if (parent.x <= team.safeNode.x + 1.f)
                        continue;
                    if (!foundProven || nodeBetterV7(parent, proven)) {
                        proven = parent;
                        foundProven = true;
                    }
                }
                if (foundProven) {
                    safeAdvanced = proven.x > team.safeNode.x + 1.f;
                    team.safeNode = proven;
                }
            }

            bool concentratedFailure =
                cluster.count >= 3 &&
                cluster.total > 0 &&
                cluster.count * 2 >= cluster.total &&
                cluster.x >= team.safeNode.x - 90.f &&
                cluster.x <= team.speculativeX + 280.f;

            if (!team.deadEnd) {
                team.stallLayers = safeAdvanced ? 0 : team.stallLayers + 1;
                team.frontier = selectFrontierV7(std::move(candidates));
                if (!team.frontier.empty())
                    mergeArchiveV9(team.archive, team.frontier);
                trimTeamStateV11(team);

                bool enterDeadEnd =
                    team.frontier.empty() ||
                    concentratedFailure ||
                    (team.stallLayers >= 3 && cluster.count >= 2);

                if (enterDeadEnd) {
                    team.focusX = cluster.count > 0
                        ? cluster.x
                        : team.lastDeathX > 0.f ? team.lastDeathX : team.speculativeX;
                    rollbackTeamV11(team, initial, false);
                }
                continue;
            }

            // Team-local recovery. SAFE is frozen for this team only. Another
            // team may be making forward progress at the exact same time.
            ++team.recoveryLayersAtDepth;
            if (escaped) {
                team.safeNode = escapedNode;
                team.deadEnd = false;
                team.deadEndLevel = 0;
                team.rollbackDistance = 0;
                team.recoveryLayersAtDepth = 0;
                team.stallLayers = 0;
                team.focusX = 0.f;
                if (team.precisionLevel > 2)
                    --team.precisionLevel;

                team.frontier = selectFrontierV7(std::move(candidates));
                if (team.frontier.empty())
                    team.frontier.push_back(team.safeNode);
                mergeArchiveV9(team.archive, team.frontier);
                trimTeamStateV11(team);
                reseedTeamVisitedV11(team);
                continue;
            }

            team.frontier = selectFrontierV7(std::move(candidates));
            if (!team.frontier.empty())
                mergeArchiveV9(team.archive, team.frontier);
            trimTeamStateV11(team);

            bool sameWall =
                cluster.count >= 2 &&
                std::abs(cluster.x - team.focusX) <= kSameWallToleranceV11;
            bool wallDominates =
                sameWall &&
                cluster.total > 0 &&
                cluster.count * 2 >= cluster.total;

            if (team.frontier.empty() || wallDominates || team.recoveryLayersAtDepth >= 2) {
                if (sameWall)
                    team.focusX = team.focusX * 0.8f + cluster.x * 0.2f;
                rollbackTeamV11(team, initial, true);
            }
        }

        if (winningTeam >= 0)
            break;

        emit(
            1,
            "Teams made independent group decisions",
            "Each ten-helper team selected its own survivors; only teams that hit a dead end rolled themselves back"
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
        << "solver=ten-independent-trainer-teams-10x10-v11"
        << " progress=" << result.progress
        << " winningTeam=" << (winningTeam >= 0 ? winningTeam + 1 : 0)
        << " bestTeam=" << bestIndex + 1
        << " bestSafeX=" << best.safeNode.x
        << " bestSpeculativeX=" << best.speculativeX
        << " bestFocusX=" << best.focusX
        << " bestRollbackDistance=" << best.rollbackDistance
        << " generations=" << generation
        << " totalTrials=" << totalTrials
        << " teams=" << kTrainerTeamsV11
        << " helpersPerTeam=" << kHelpersPerTeamV11
        << " logicalHelpers=" << kLogicalHelpersV11
        << " physicalThreads=" << kPhysicalThreadsV11
        << " prunedNoTouch=" << prunedNoTouch
        << " inputs=" << result.inputs.size()
        << " complete=" << (result.complete ? 1 : 0)
        << " stopped=" << (stop.load() ? 1 : 0);

    for (int i = 0; i < kTrainerTeamsV11; ++i) {
        diagnostics
            << " team" << (i + 1) << "SafeX=" << teams[i].safeNode.x
            << " team" << (i + 1) << "DeadEnd=" << (teams[i].deadEnd ? 1 : 0)
            << " team" << (i + 1) << "Rollbacks=" << teams[i].recoveryCount;
    }
    result.diagnostics = diagnostics.str();

    emit(
        result.complete ? 4 : 2,
        result.complete
            ? fmt::format("Team {} found a candidate clear", winningTeam + 1)
            : "Stopped on the best independently verified team route",
        result.complete
            ? "The winning team's route now has to reproduce the finish in a fresh replay"
            : "No team's rollback-only position was accepted as earned progress"
    );

    return result;
}

} // namespace

PathfinderResult pathfind_v11(
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

    PathfinderResult result = runTrainerTeamsV11(
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
