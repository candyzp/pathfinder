// V13 fixes the V12 recovery state machine.
// 100 logical helpers remain 10 trainer teams x 10 helpers, but recovery can no
// longer erase forward recovery progress or increase rollback depth without an
// actually earlier anchor. SAFE progress is frozen during recovery instead of
// being replaced by the rollback anchor, and Stop returns the best living
// partial route instead of throwing it away.
#define pathfind_v12 pathfind_v12_base_v13
#include "pathfinder_state_v12.cpp"
#undef pathfind_v12

namespace {

constexpr int kTrainerTeamsV13 = 10;
constexpr int kHelpersPerTeamV13 = 10;
constexpr int kLogicalHelpersV13 = 100;
constexpr int kPhysicalThreadsV13 = 30;
constexpr float kEscapeMarginV13 = 165.f;
constexpr float kSameWallToleranceV13 = 84.f;
constexpr float kMinimumAnchorMoveV13 = 24.f;
constexpr int kMaxRollbackLevelV13 = 9;
constexpr int kMaxAnchorRetriesV13 = 10;
constexpr int kGlobalStallLimitV13 = 700;

static_assert(kTrainerTeamsV13 * kHelpersPerTeamV13 == kLogicalHelpersV13);

struct TeamRuntimeV13 {
    TeamStateV11 team;
    SearchNodeV7 rally;
    SearchNodeV7 rollbackAnchor;
    SearchNodeV7 bestUsable;
    float lastWallX = 0.f;
    int wallStrikes = 0;
    int noProgressRounds = 0;
    int anchorRetries = 0;
    bool exhausted = false;
};

struct TeamBatchV13 {
    std::vector<SearchNodeV7> produced;
    std::vector<float> deaths;
    std::vector<uint64_t> failedKeys;
    float maxAliveX = -std::numeric_limits<float>::infinity();
    int alive = 0;
    int dead = 0;
};

SearchNodeV7 chooseForwardWinnerV13(
    SearchNodeV7 const& previous,
    std::vector<SearchNodeV7> const& candidates
) {
    SearchNodeV7 winner = previous;
    for (auto const& node : candidates) {
        if (node.x > winner.x + 0.5f ||
            (std::abs(node.x - winner.x) <= 0.5f && nodeBetterV7(node, winner))) {
            winner = node;
        }
    }
    return winner;
}

void rememberUsableV13(TeamRuntimeV13& runtime, SearchNodeV7 const& node) {
    if (node.x > runtime.bestUsable.x + 0.5f ||
        (std::abs(node.x - runtime.bestUsable.x) <= 0.5f &&
         nodeBetterV7(node, runtime.bestUsable))) {
        runtime.bestUsable = node;
    }
}

int requestedRollbackDistanceV13(
    int level,
    float focusX,
    float startX
) {
    static constexpr std::array<int, 10> distances = {
        120, 220, 360, 540, 780, 1080, 1450, 1900, 2500, 3300
    };
    int index = std::clamp(level, 0, static_cast<int>(distances.size()) - 1);
    int available = std::max(0, static_cast<int>(std::floor(focusX - startX)) - 8);
    return std::min(distances[index], available);
}

SearchNodeV7 rollbackAnchorForV13(
    TeamRuntimeV13 const& runtime,
    SearchNodeV7 const& initial,
    int rollbackLevel,
    float startX
) {
    int distance = requestedRollbackDistanceV13(
        rollbackLevel,
        runtime.team.focusX,
        startX
    );
    std::vector<SearchNodeV7> current {runtime.rally};
    return rollbackAnchorV9(
        runtime.team.archive,
        current,
        initial,
        runtime.team.focusX,
        distance
    );
}

void installRecoveryAnchorV13(
    TeamRuntimeV13& runtime,
    SearchNodeV7 const& anchor,
    int rollbackLevel
) {
    runtime.team.deadEnd = true;
    runtime.team.deadEndLevel = rollbackLevel;
    runtime.rollbackAnchor = anchor;
    runtime.team.rollbackAnchorX = anchor.x;
    runtime.team.rollbackDistance = std::max(
        0,
        static_cast<int>(std::lround(runtime.team.focusX - anchor.x))
    );
    runtime.rally = anchor;
    runtime.team.frontier = {anchor};
    runtime.team.recoveryLayersAtDepth = 0;
    runtime.noProgressRounds = 0;
    runtime.wallStrikes = 0;
    ++runtime.team.recoveryCount;
    mergeArchiveV9(runtime.team.archive, {anchor});
    trimTeamStateV11(runtime.team);
    reseedTeamVisitedV11(runtime.team);
}

void enterRecoveryV13(
    TeamRuntimeV13& runtime,
    SearchNodeV7 const& initial,
    float wallX,
    float startX
) {
    runtime.team.deadEnd = true;
    runtime.team.focusX = wallX;
    runtime.team.recoveryOriginSafeX = runtime.team.safeNode.x;
    runtime.team.precisionLevel = std::clamp(
        std::max(runtime.team.precisionLevel, 4),
        4,
        6
    );
    runtime.anchorRetries = 0;
    runtime.exhausted = false;

    SearchNodeV7 anchor = rollbackAnchorForV13(runtime, initial, 0, startX);
    installRecoveryAnchorV13(runtime, anchor, 0);
}

bool deepenRecoveryV13(
    TeamRuntimeV13& runtime,
    SearchNodeV7 const& initial,
    float startX
) {
    if (runtime.team.deadEndLevel >= kMaxRollbackLevelV13) {
        ++runtime.anchorRetries;
        runtime.rally = runtime.rollbackAnchor;
        runtime.team.frontier = {runtime.rollbackAnchor};
        runtime.noProgressRounds = 0;
        runtime.wallStrikes = 0;
        return false;
    }

    int proposedLevel = runtime.team.deadEndLevel + 1;
    SearchNodeV7 candidate = rollbackAnchorForV13(
        runtime,
        initial,
        proposedLevel,
        startX
    );

    // This is the anti-runaway rule. A larger numeric depth is meaningless if
    // it resolves to the same physical checkpoint. Do not increment it.
    if (candidate.x >= runtime.rollbackAnchor.x - kMinimumAnchorMoveV13) {
        ++runtime.anchorRetries;
        runtime.rally = runtime.rollbackAnchor;
        runtime.team.frontier = {runtime.rollbackAnchor};
        runtime.noProgressRounds = 0;
        runtime.wallStrikes = 0;
        return false;
    }

    runtime.anchorRetries = 0;
    installRecoveryAnchorV13(runtime, candidate, proposedLevel);
    return true;
}

int bestSafeTeamV13(std::array<TeamRuntimeV13, kTrainerTeamsV13> const& runtimes) {
    int best = 0;
    for (int i = 1; i < kTrainerTeamsV13; ++i) {
        auto const& lhs = runtimes[i].team.safeNode;
        auto const& rhs = runtimes[best].team.safeNode;
        if (lhs.x > rhs.x + 0.5f ||
            (std::abs(lhs.x - rhs.x) <= 0.5f && nodeBetterV7(lhs, rhs))) {
            best = i;
        }
    }
    return best;
}

int bestUsableTeamV13(std::array<TeamRuntimeV13, kTrainerTeamsV13> const& runtimes) {
    int best = 0;
    for (int i = 1; i < kTrainerTeamsV13; ++i) {
        auto const& lhs = runtimes[i].bestUsable;
        auto const& rhs = runtimes[best].bestUsable;
        if (lhs.x > rhs.x + 0.5f ||
            (std::abs(lhs.x - rhs.x) <= 0.5f && nodeBetterV7(lhs, rhs))) {
            best = i;
        }
    }
    return best;
}

PathfinderResult runTrainerV13(
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
        root.lengthSource = "trusted-gd-v13";
    }

    SearchNodeV7 initial;
    initial.snapshot = captureSnapshotV7(root);
    initial.x = root.latestState().pos.x;
    initial.y = root.latestState().pos.y;
    initial.velocity = root.latestState().velocity;
    initial.minClearance = hazardClearance(root, root.latestState());
    initial.score = scoreStateV7(root, initial.snapshot, initial.minClearance, 0, startX);

    std::array<TeamRuntimeV13, kTrainerTeamsV13> runtimes;
    std::array<uint32_t, kTrainerTeamsV13> teamSeeds {};
    std::random_device rd;
    uint32_t baseSeed = rd() ^ static_cast<uint32_t>(std::hash<std::string>{}(lvlString));

    for (int i = 0; i < kTrainerTeamsV13; ++i) {
        TeamRuntimeV13& runtime = runtimes[i];
        TeamStateV11& team = runtime.team;
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
        team.failureMemory.reserve(3200);
        runtime.rally = initial;
        runtime.rollbackAnchor = initial;
        runtime.bestUsable = initial;
        teamSeeds[i] = baseSeed ^ static_cast<uint32_t>((i + 1) * 0x9e3779b9u);
    }

    uint64_t totalTrials = 0;
    int generation = 0;
    int winningTeam = -1;
    int globalStall = 0;
    float globalBestUsableX = initial.x;

    auto emit = [&](int phase, std::string decision, std::string reason) {
        int bestIndex = bestSafeTeamV13(runtimes);
        TeamRuntimeV13 const& bestRuntime = runtimes[bestIndex];
        TeamStateV11 const& best = bestRuntime.team;

        int recovering = 0;
        int exhausted = 0;
        int totalArchive = 0;
        int totalProduced = 0;
        int totalUnique = 0;
        int totalDead = 0;
        int totalDuplicate = 0;
        int totalRecoveries = 0;
        float seen = initial.x;
        for (auto const& runtime : runtimes) {
            recovering += runtime.team.deadEnd ? 1 : 0;
            exhausted += runtime.exhausted ? 1 : 0;
            totalArchive += static_cast<int>(runtime.team.archive.size());
            totalProduced += runtime.team.lastProduced;
            totalUnique += runtime.team.lastUnique;
            totalDead += runtime.team.lastDead;
            totalDuplicate += runtime.team.lastDuplicate;
            totalRecoveries += runtime.team.recoveryCount;
            seen = std::max(seen, runtime.bestUsable.x);
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
        telemetry.furthestX = seen;
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
            bestRuntime.rally.snapshot.p1.vehicle.type,
            best.precisionLevel,
            false
        );
        telemetry.candidateCount = kLogicalHelpersV13;
        telemetry.workerCount = kLogicalHelpersV13;
        telemetry.physicalThreadCount = kPhysicalThreadsV13;
        telemetry.phase = phase;
        telemetry.frontierCount = kTrainerTeamsV13;
        telemetry.guidedCount = 50;
        telemetry.explorerCount = 50;
        telemetry.archiveCount = totalArchive;
        telemetry.producedCount = totalProduced;
        telemetry.uniqueCount = totalUnique;
        telemetry.deadCount = totalDead;
        telemetry.duplicateCount = totalDuplicate;
        telemetry.stallLayers = globalStall;
        telemetry.recoveryCount = totalRecoveries;
        telemetry.deathClusterCount = best.lastClusterCount;
        telemetry.rollbackDistance = best.rollbackDistance;
        telemetry.deadEndLevel = best.deadEndLevel;
        telemetry.stallRescue = recovering > 0;
        telemetry.progressLocked = recovering == kTrainerTeamsV13;
        telemetry.totalTrials = totalTrials;
        telemetry.mode = "trainer-teams-v13-no-runaway-rollback";
        telemetry.decision = std::move(decision);
        telemetry.recoveryReason = fmt::format(
            "{} | team {} SAFE {:.0f}, rally {:.0f}; {} recovering, {} exhausted",
            reason,
            bestIndex + 1,
            best.safeNode.x,
            bestRuntime.rally.x,
            recovering,
            exhausted
        );
        publishPathfinderTelemetryV8(telemetry);
        if (callback)
            callback(telemetry);
    };

    emit(
        0,
        "10 trainer teams ready",
        "Rollback anchors are search bases only; forward recovery progress is never erased by the same generation"
    );

    while (!stop.load() && winningTeam < 0) {
        ++generation;
        Level2 actionProbe = compactWorkerV7(root);
        std::vector<RallyTaskV12> tasks;
        tasks.reserve(kLogicalHelpersV13);

        for (int t = 0; t < kTrainerTeamsV13; ++t) {
            TeamRuntimeV13& runtime = runtimes[t];
            if (runtime.exhausted || runtime.team.complete)
                continue;

            auto teamTasks = buildRallyTasksV12(
                actionProbe,
                runtime.team,
                runtime.rally,
                teamSeeds[t],
                generation + runtime.anchorRetries * 101
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
            "10 teams training separate rally routes",
            "Each team follows its own best living route; only a stalled team is allowed to backtrack"
        );

        std::atomic<size_t> nextTask {0};
        std::mutex resultMutex;
        std::array<TeamBatchV13, kTrainerTeamsV13> batches;

        int threadCount = std::min<int>(kPhysicalThreadsV13, static_cast<int>(tasks.size()));
        std::vector<std::thread> threads;
        threads.reserve(threadCount);

        for (int threadIndex = 0; threadIndex < threadCount; ++threadIndex) {
            threads.emplace_back([&, threadIndex] {
                Level2 worker = compactWorkerV7(root);
                std::array<TeamBatchV13, kTrainerTeamsV13> local;

                while (!stop.load()) {
                    size_t index = nextTask.fetch_add(1, std::memory_order_relaxed);
                    if (index >= tasks.size())
                        break;

                    RallyTaskV12 const& task = tasks[index];
                    SearchNodeV7 const& parent = runtimes[task.team].rally;
                    SimResultV7 sim = simulateActionV7(worker, parent, task.action, startX);
                    TeamBatchV13& batch = local[task.team];

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
                for (int t = 0; t < kTrainerTeamsV13; ++t) {
                    TeamBatchV13& dst = batches[t];
                    TeamBatchV13& src = local[t];
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

        bool anyUsableAdvanced = false;

        for (int t = 0; t < kTrainerTeamsV13; ++t) {
            TeamRuntimeV13& runtime = runtimes[t];
            TeamStateV11& team = runtime.team;
            TeamBatchV13& batch = batches[t];
            if (runtime.exhausted || (batch.alive + batch.dead) == 0)
                continue;

            totalTrials += static_cast<uint64_t>(batch.alive + batch.dead);
            team.trials += static_cast<uint64_t>(batch.alive + batch.dead);
            for (uint64_t key : batch.failedKeys)
                ++team.failureMemory[key];

            float previousWallX = runtime.lastWallX;
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
            int duplicates = 0;
            for (auto& node : batch.produced) {
                StateKeyV7 key = stateKeyV7(node.snapshot, team.precisionLevel);
                auto it = unique.find(key);
                if (it == unique.end() || nodeBetterV7(node, it->second))
                    unique[key] = std::move(node);
                else
                    ++duplicates;
            }

            std::vector<SearchNodeV7> candidates;
            candidates.reserve(unique.size());
            for (auto& [_, node] : unique)
                candidates.push_back(std::move(node));

            team.lastProduced = static_cast<int>(batch.produced.size());
            team.lastUnique = static_cast<int>(candidates.size());
            team.lastDead = batch.dead;
            team.lastDuplicate = duplicates;

            SearchNodeV7 previousRally = runtime.rally;
            SearchNodeV7 winner = chooseForwardWinnerV13(previousRally, candidates);
            bool rallyAdvanced = winner.x > previousRally.x + 0.5f;

            for (auto const& node : candidates) {
                rememberUsableV13(runtime, node);
                team.speculativeX = std::max(team.speculativeX, node.x);
                if (node.x > team.seenNode.x + 0.5f || nodeBetterV7(node, team.seenNode))
                    team.seenNode = node;
                if (node.complete) {
                    team.solutionNode = node;
                    team.complete = true;
                    winningTeam = t;
                    break;
                }
            }
            if (winningTeam >= 0)
                break;

            if (runtime.bestUsable.x > globalBestUsableX + 0.5f) {
                globalBestUsableX = runtime.bestUsable.x;
                anyUsableAdvanced = true;
            }

            bool sameRecentWall =
                cluster.count >= 3 &&
                previousWallX > 0.f &&
                std::abs(cluster.x - previousWallX) <= kSameWallToleranceV13;
            if (cluster.count > 0)
                runtime.lastWallX = cluster.x;

            if (!team.deadEnd) {
                // The current rally becomes verified only after one of its ten
                // children survives far enough beyond it.
                float proofDistance = flightMode(previousRally.snapshot.p1.vehicle.type)
                    ? 18.f : 24.f;
                if (batch.alive > 0 && batch.maxAliveX >= previousRally.x + proofDistance) {
                    if (previousRally.x > team.safeNode.x + 1.f)
                        team.safeNode = previousRally;
                }

                // CRITICAL: forward movement wins. Never declare a dead end in
                // the same generation that found a farther living child.
                if (rallyAdvanced) {
                    runtime.rally = winner;
                    team.frontier = {winner};
                    mergeArchiveV9(team.archive, {winner, team.safeNode});
                    trimTeamStateV11(team);
                    team.stallLayers = 0;
                    runtime.noProgressRounds = 0;
                    runtime.wallStrikes = 0;
                    continue;
                }

                if (winner.x >= previousRally.x - 0.5f && nodeBetterV7(winner, previousRally))
                    runtime.rally = winner;

                ++team.stallLayers;
                runtime.wallStrikes = sameRecentWall
                    ? runtime.wallStrikes + 1
                    : (cluster.count > 0 ? 1 : 0);

                bool mostlyDead = batch.dead >= 7;
                bool provenWall = mostlyDead && runtime.wallStrikes >= 2;
                if (candidates.empty() || provenWall || team.stallLayers >= 4) {
                    float wallX = cluster.count > 0
                        ? cluster.x
                        : team.lastDeathX > 0.f ? team.lastDeathX : previousRally.x;
                    enterRecoveryV13(runtime, initial, wallX, startX);
                }
                continue;
            }

            ++team.recoveryLayersAtDepth;

            // Recovery also obeys forward-first. This is what V12 violated.
            // A team walking back toward its wall is allowed to KEEP walking.
            if (rallyAdvanced) {
                runtime.rally = winner;
                team.frontier = {winner};
                mergeArchiveV9(team.archive, {winner});
                trimTeamStateV11(team);
                runtime.noProgressRounds = 0;
                runtime.wallStrikes = 0;

                if (winner.x > team.focusX + kEscapeMarginV13) {
                    team.safeNode = winner;
                    team.deadEnd = false;
                    team.deadEndLevel = 0;
                    team.rollbackDistance = 0;
                    team.recoveryLayersAtDepth = 0;
                    team.stallLayers = 0;
                    team.focusX = 0.f;
                    runtime.anchorRetries = 0;
                    runtime.lastWallX = 0.f;
                    if (team.precisionLevel > 2)
                        --team.precisionLevel;
                }
                continue;
            }

            if (winner.x >= previousRally.x - 0.5f && nodeBetterV7(winner, previousRally))
                runtime.rally = winner;

            ++runtime.noProgressRounds;
            bool sameRecoveryWall =
                cluster.count >= 3 &&
                std::abs(cluster.x - team.focusX) <= kSameWallToleranceV13;
            runtime.wallStrikes = sameRecoveryWall
                ? runtime.wallStrikes + 1
                : std::max(0, runtime.wallStrikes - 1);

            bool shouldBackUp =
                candidates.empty() ||
                runtime.wallStrikes >= 2 ||
                runtime.noProgressRounds >= 3;

            if (shouldBackUp) {
                deepenRecoveryV13(runtime, initial, startX);
                if (runtime.anchorRetries >= kMaxAnchorRetriesV13) {
                    runtime.exhausted = true;
                }
            }
        }

        if (winningTeam >= 0)
            break;

        globalStall = anyUsableAdvanced ? 0 : globalStall + 1;
        int exhaustedCount = 0;
        int recoveringCount = 0;
        for (auto const& runtime : runtimes) {
            exhaustedCount += runtime.exhausted ? 1 : 0;
            recoveringCount += runtime.team.deadEnd ? 1 : 0;
        }

        emit(
            1,
            "Teams committed their own route decisions",
            "Living forward progress is preserved; stalled teams back up only when a genuinely earlier anchor exists"
        );

        if (exhaustedCount == kTrainerTeamsV13)
            break;
        if (globalStall >= kGlobalStallLimitV13 &&
            recoveringCount + exhaustedCount == kTrainerTeamsV13) {
            break;
        }
    }

    int outputTeam = winningTeam >= 0
        ? winningTeam
        : bestUsableTeamV13(runtimes);
    TeamRuntimeV13 const& outputRuntime = runtimes[outputTeam];
    SearchNodeV7 const& outputNode = winningTeam >= 0
        ? outputRuntime.team.solutionNode
        : outputRuntime.bestUsable;

    PathfinderResult result;
    result.inputs = routeToInputsV7(outputNode.route);
    result.macro = inputsToMacroV7(result.inputs);
    result.complete = winningTeam >= 0;
    result.progress = progressFor(
        result.complete ? root.length : outputNode.x,
        startX,
        root.length,
        result.complete
    );

    std::ostringstream diagnostics;
    diagnostics
        << "solver=trainer-teams-v13-no-runaway-rollback"
        << " progress=" << result.progress
        << " outputTeam=" << outputTeam + 1
        << " winningTeam=" << (winningTeam >= 0 ? winningTeam + 1 : 0)
        << " outputX=" << outputNode.x
        << " generations=" << generation
        << " globalStall=" << globalStall
        << " totalTrials=" << totalTrials
        << " logicalHelpers=" << kLogicalHelpersV13
        << " teams=" << kTrainerTeamsV13
        << " helpersPerTeam=" << kHelpersPerTeamV13
        << " physicalThreads=" << kPhysicalThreadsV13
        << " prunedNoTouch=" << prunedNoTouch
        << " inputs=" << result.inputs.size()
        << " complete=" << (result.complete ? 1 : 0)
        << " stopped=" << (stop.load() ? 1 : 0);

    for (int i = 0; i < kTrainerTeamsV13; ++i) {
        auto const& runtime = runtimes[i];
        diagnostics
            << " team" << (i + 1) << "SafeX=" << runtime.team.safeNode.x
            << " team" << (i + 1) << "RallyX=" << runtime.rally.x
            << " team" << (i + 1) << "UsableX=" << runtime.bestUsable.x
            << " team" << (i + 1) << "Depth=" << runtime.team.deadEndLevel
            << " team" << (i + 1) << "Rollback=" << runtime.team.rollbackDistance
            << " team" << (i + 1) << "Exhausted=" << (runtime.exhausted ? 1 : 0);
    }
    result.diagnostics = diagnostics.str();

    emit(
        result.complete ? 4 : 2,
        result.complete
            ? fmt::format("Team {} found a candidate clear", winningTeam + 1)
            : stop.load()
                ? fmt::format("Stopped: keeping Team {} best living partial route", outputTeam + 1)
                : fmt::format("Search saturated: keeping Team {} best living partial route", outputTeam + 1),
        result.complete
            ? "The candidate clear still has to reproduce in a fresh replay"
            : "The best living route is preserved even if its team was still recovering"
    );

    return result;
}

} // namespace

PathfinderResult pathfind_v13(
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
                telemetry.recoveryReason = "100% stays locked until a fresh replay reproduces the finish";
            }
            callback(telemetry);
        };
    }

    PathfinderResult result = runTrainerV13(
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
