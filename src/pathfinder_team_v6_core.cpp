#include "pathfinder_team_v4.cpp"

namespace {

// Orb specialists used to generate short pulses that could begin well before the
// estimated orb contact and release before the cube actually touched the orb.
// Every early v6 pulse is forced to remain held through the contact window.
void appendSpanningOrbCandidatesV6(
    Level2 const& base,
    int horizonFrames,
    std::set<SearchInput> const& seedLeader,
    UpcomingOrb const& orb,
    std::vector<std::set<SearchInput>>& candidates
) {
    if (!orb.found || orb.frameOffset < 0)
        return;

    static constexpr std::array<int, 21> deltas = {
        -24, -18, -14, -10, -8, -6, -4, -3, -2, -1,
        0, 1, 2, 3, 4, 6, 8, 10, 14, 18, 24
    };

    int contactTail = orb.type == OrbType::Blue ? 5 : 3;
    auto addFamily = [&](int delta, int requestedWidth, bool useSeed) {
        int pressAt = orb.frameOffset + delta;
        if (pressAt < 0 || pressAt >= horizonFrames - 1)
            return;

        // If the press begins early, never release it before the estimated orb
        // contact plus a few coyote frames. This is the direct fix for the
        // "click early, arrive with no click left" failure seen around 23%.
        int minimumWidth = std::max(1, orb.frameOffset + contactTail - pressAt);
        int width = std::max(requestedWidth, minimumWidth);
        width = std::min(width, horizonFrames - pressAt - 1);
        if (width <= 0)
            return;

        std::set<SearchInput> route = useSeed
            ? windowedRoute(seedLeader, base.currentFrame(), horizonFrames)
            : std::set<SearchInput>{};
        forceFreshPulse(route, base, horizonFrames, pressAt, width);
        candidates.push_back(std::move(route));
    };

    for (int delta : deltas) {
        if (orb.type == OrbType::Blue) {
            for (int width : {1, 2, 4, 8, 16, 28, 48})
                addFamily(delta, width, true);
        } else {
            for (int width : {1, 2, 4, 8, 20, 36})
                addFamily(delta, width, true);
        }
    }

    // A few clean-slate orb routes avoid inheriting an already-poisoned local
    // click pattern from the current champion.
    for (int delta : {-8, -4, -2, -1, 0, 1, 2, 4, 8}) {
        for (int width : {2, 8, 20})
            addFamily(delta, width, false);
    }
}

TeamBatchResult evaluateTeamBatch100V6(
    Level2 const& base,
    std::vector<std::set<SearchInput>> const& specialists,
    std::set<SearchInput> const& seedLeader,
    int cooperativeHorizonFrames,
    int rogueHorizonFrames,
    int searchLevel,
    int focusOffset,
    bool focused,
    uint32_t baseSeed,
    std::atomic_bool& stop,
    LiveBatchCallback liveCallback
) {
    // The original 50-helper crew remains intact. It still uses 30 physical
    // threads and shares live leaders exactly as before.
    TeamBatchResult cooperative;
    std::atomic<uint64_t> cooperativeTrials {0};
    std::atomic<uint64_t> rogueTrials {0};
    std::atomic<float> bestLiveX {base.latestState().pos.x};
    std::mutex callbackMutex;

    auto reportCombined = [&](uint64_t coopTrials, float liveX) {
        cooperativeTrials.store(coopTrials, std::memory_order_relaxed);
        float current = bestLiveX.load(std::memory_order_relaxed);
        while (liveX > current &&
               !bestLiveX.compare_exchange_weak(
                   current,
                   liveX,
                   std::memory_order_relaxed
               )) {}
        if (liveCallback) {
            std::lock_guard<std::mutex> guard(callbackMutex);
            liveCallback(
                cooperativeTrials.load(std::memory_order_relaxed) +
                    rogueTrials.load(std::memory_order_relaxed),
                bestLiveX.load(std::memory_order_relaxed)
            );
        }
    };

    std::thread cooperativeRunner([&] {
        cooperative = evaluateTeamBatch(
            base,
            specialists,
            seedLeader,
            cooperativeHorizonFrames,
            searchLevel,
            focusOffset,
            focused,
            baseSeed,
            stop,
            reportCombined
        );
        cooperativeTrials.store(cooperative.trials, std::memory_order_relaxed);
    });

    // The other 50 helpers are intentionally isolated from the current champion.
    // They never read seedLeader/liveLeaders, never use focusOffset, and keep the
    // broad horizon even when the cooperative crew zooms into a death/orb section.
    constexpr int kRoguePhysicalThreads = 10;
    constexpr int kRogueLogicalWorkers = 50;
    int rogueAttempts = focused
        ? std::clamp(24 + searchLevel * 2, 24, 44)
        : std::clamp(18 + searchLevel * 2, 18, 36);

    Level2 compactBase = compactWorkerBase(base);
    std::atomic_bool rogueSolved = false;
    std::mutex rogueBestMutex;
    LocalBest rogueBest;
    std::atomic<uint64_t> nextRogueReport {24};

    auto mergeRogue = [&](LocalBest&& local) {
        if (!local.have)
            return;
        std::lock_guard<std::mutex> guard(rogueBestMutex);
        if (teamBetterTrial(
                local.trial,
                local.toggles,
                rogueBest.have,
                rogueBest.trial,
                rogueBest.toggles,
                base.latestState().vehicle.type
            )) {
            rogueBest = std::move(local);
        }
    };

    std::vector<std::thread> rogueThreads;
    rogueThreads.reserve(kRoguePhysicalThreads);
    for (int lane = 0; lane < kRoguePhysicalThreads; ++lane) {
        rogueThreads.emplace_back([&, lane] {
            Level2 worker = compactBase;
            prepareWorker(worker);
            WorkerBaseline baseline = captureWorkerBaseline(worker);
            LocalBest local;
            std::mt19937 rng(
                baseSeed ^ 0x9e3779b9u ^
                static_cast<uint32_t>((base.currentFrame() + 31) * 2246822519u) ^
                static_cast<uint32_t>((lane + 101) * 3266489917u)
            );

            for (int attempt = 0;
                 attempt < rogueAttempts &&
                 !stop.load() &&
                 !rogueSolved.load(std::memory_order_acquire);
                 ++attempt) {
                // Five independent logical identities ride each physical rogue
                // thread, giving exactly 50 independent global-search helpers.
                int logicalLane = 50 + lane + (attempt % 5) * 10;
                auto candidate = makeBruteCandidate(
                    base,
                    rogueHorizonFrames,
                    logicalLane,
                    attempt,
                    -1,
                    rng
                );
                TrialResult trial = tryInputsTeam(
                    worker,
                    baseline,
                    candidate,
                    rogueHorizonFrames
                );
                considerLocal(
                    local,
                    candidate,
                    trial,
                    TeamRole::BruteForce,
                    base.latestState().vehicle.type
                );

                uint64_t n = rogueTrials.fetch_add(1, std::memory_order_relaxed) + 1;
                float current = bestLiveX.load(std::memory_order_relaxed);
                while (trial.x > current &&
                       !bestLiveX.compare_exchange_weak(
                           current,
                           trial.x,
                           std::memory_order_relaxed
                       )) {}

                uint64_t target = nextRogueReport.load(std::memory_order_relaxed);
                if (liveCallback && n >= target &&
                    nextRogueReport.compare_exchange_strong(
                        target,
                        n + 24,
                        std::memory_order_relaxed
                    )) {
                    std::lock_guard<std::mutex> guard(callbackMutex);
                    liveCallback(
                        cooperativeTrials.load(std::memory_order_relaxed) +
                            rogueTrials.load(std::memory_order_relaxed),
                        bestLiveX.load(std::memory_order_relaxed)
                    );
                }

                if (trial.complete)
                    rogueSolved.store(true, std::memory_order_release);
            }
            mergeRogue(std::move(local));
        });
    }

    for (auto& thread : rogueThreads)
        thread.join();
    cooperativeRunner.join();

    TeamBatchResult out = cooperative;
    out.physicalWorkers = 40;
    out.logicalWorkers = 100;
    out.trials = cooperativeTrials.load(std::memory_order_relaxed) +
        rogueTrials.load(std::memory_order_relaxed);
    out.candidateCount = cooperative.candidateCount +
        rogueAttempts * kRogueLogicalWorkers;

    // Rogue deaths deliberately do not move lastDeathX/deathClusterX. Their job is
    // to disagree with the focused theory, not teach it a new random death target.
    if (rogueBest.have && teamBetterTrial(
            rogueBest.trial,
            rogueBest.toggles,
            out.haveBest,
            out.bestTrial,
            out.bestInputs.size(),
            base.latestState().vehicle.type
        )) {
        out.bestInputs = std::move(rogueBest.inputs);
        out.bestTrial = rogueBest.trial;
        out.haveBest = true;
        out.bestRole = TeamRole::BruteForce;
    }

    if (liveCallback) {
        std::lock_guard<std::mutex> guard(callbackMutex);
        liveCallback(out.trials, bestLiveX.load(std::memory_order_relaxed));
    }
    return out;
}

} // namespace

PathfinderResult pathfind_v6_core(
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
    int hardAvoidRounds = 0;
    int debugSearchLevel = 0;
    int debugHorizon = 0;
    int debugCandidateCount = 0;
    int debugHelpers = 100;
    int debugPhysicalWorkers = 40;
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
        telemetry.workerCount = debugHelpers;
        telemetry.phase = phase;
        telemetry.totalTrials = totalTrials;
        telemetry.mode = "split100-50coop-50rogue-v6";
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

    auto observeFailure = [&](float x, VehicleType mode) {
        if (x <= 0.f)
            return;
        lastDeathX = x;
        float band = flightMode(mode) ? 90.f : 140.f;
        if (deathRepeat > 0 && std::abs(x - deathClusterX) <= band) {
            ++deathRepeat;
            deathClusterX = deathClusterX * 0.75f + x * 0.25f;
        } else {
            deathRepeat = 1;
            deathClusterX = x;
        }
    };

    while (!reachedGoal(lvl) && !stop.load()) {
        try {
            if (lvl.latestState().dead) {
                observeFailure(lvl.latestState().pos.x, lvl.latestState().vehicle.type);
                recoverFromBest(std::min(3200, 260 + recoveryCount * 220));
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
            int broadHorizon = horizonFrames;

            UpcomingOrb orb = findUpcomingOrb(lvl, horizonFrames);
            float failureTargetX = deathRepeat >= 2 ? deathClusterX : lastDeathX;
            int failureOffset = estimateFrameOffsetForX(lvl, failureTargetX, horizonFrames);
            int focusOffset = -1;
            bool hardFailure = false;

            if (orb.found && orb.frameOffset >= 0 && orb.frameOffset <= 360)
                focusOffset = orb.frameOffset;

            if (deathRepeat >= 2 &&
                failureOffset >= 0 &&
                failureOffset < horizonFrames - 12) {
                hardFailure = true;
                int preRoll = flightMode(mode)
                    ? 150
                    : mode == VehicleType::Robot ? 190
                    : mode == VehicleType::Spider ? 130
                    : 240;
                int preDeathFocus = std::max(0, failureOffset - preRoll);
                if (focusOffset < 0 || preDeathFocus < focusOffset)
                    focusOffset = preDeathFocus;
            }

            bool focused = focusOffset >= 0;
            if (focused) {
                ++focusedRounds;
                if (hardFailure)
                    ++hardAvoidRounds;

                int postFailure = flightMode(mode) ? 260 : 150;
                int wanted = hardFailure && failureOffset >= 0
                    ? failureOffset + postFailure
                    : focusOffset + (flightMode(mode) ? 320 : 210);
                int minimum = flightMode(mode) ? 420 : 280;
                int maximum = flightMode(mode) ? 1050 : 980;
                horizonFrames = std::clamp(
                    wanted,
                    minimum,
                    std::min(maximum, broadHorizon)
                );
                orb = findUpcomingOrb(lvl, horizonFrames);
                if (hardFailure)
                    failureOffset = estimateFrameOffsetForX(lvl, deathClusterX, horizonFrames);
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
            // V6 replaces the old orb family instead of stacking both. This keeps
            // obviously expired pre-orb pulses out of the specialist pool.
            appendSpanningOrbCandidatesV6(
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
                failureTargetX,
                deathRepeat,
                specialists
            );
            appendHardFailureCandidatesV4(
                lvl,
                horizonFrames,
                persistentLeader,
                failureOffset,
                deathRepeat,
                specialists
            );
            dedupeCandidates(specialists);

            int generatedAttempts = focused
                ? std::clamp(18 + searchLevel * 2, 18, 36)
                : std::clamp(12 + searchLevel * 2, 12, 30);
            int rogueAttempts = focused
                ? std::clamp(24 + searchLevel * 2, 24, 44)
                : std::clamp(18 + searchLevel * 2, 18, 36);
            debugVehicleType = static_cast<int>(mode);
            debugSearchLevel = searchLevel;
            debugHorizon = horizonFrames;
            debugHelpers = 100;
            debugPhysicalWorkers = 40;
            debugCandidateCount = static_cast<int>(
                specialists.size() + generatedAttempts * 20 + rogueAttempts * 50
            );
            debugClearance = 0.f;
            emitTelemetry(
                hardFailure ? 2 : 0,
                hardFailure ? "hard-negative+rogue-search" :
                    focused ? "focused+rogue-search" : "split100-search"
            );

            auto liveReport = [&](uint64_t batchTrials, float liveX) {
                if (!callback)
                    return;
                PathfinderTelemetry telemetry = makeTelemetry(
                    hardFailure ? 2 : 1,
                    hardFailure ? "live-hard-negative+rogue" :
                        focused ? "live-focused+rogue" : "live-split100"
                );
                telemetry.currentX = std::max(telemetry.currentX, liveX);
                telemetry.totalTrials = totalTrials + batchTrials;
                callback(telemetry);
            };

            TeamBatchResult team = evaluateTeamBatch100V6(
                lvl,
                specialists,
                persistentLeader,
                horizonFrames,
                broadHorizon,
                searchLevel,
                focusOffset,
                focused,
                baseSeed ^
                    static_cast<uint32_t>(recoveryCount * 7919u) ^
                    static_cast<uint32_t>(frame * 104729u) ^
                    static_cast<uint32_t>(deathRepeat * 65537u),
                stop,
                liveReport
            );

            totalTrials += team.trials;
            debugHelpers = team.logicalWorkers;
            debugPhysicalWorkers = team.physicalWorkers;
            debugCandidateCount = team.candidateCount;
            if (team.lastDeathX > 0.f)
                lastDeathX = team.lastDeathX;

            if (stop.load())
                break;

            std::set<SearchInput> bestInputs = team.bestInputs;
            TrialResult bestTrial = team.bestTrial;
            bool haveBest = team.haveBest;
            float oldClusterX = deathClusterX;
            int oldDeathRepeat = deathRepeat;

            if (haveBest && bestTrial.dead && bestTrial.deathX > 0.f)
                observeFailure(bestTrial.deathX, mode);

            bool escapedKnownFailure = haveBest && bestTrial.dead &&
                oldDeathRepeat >= 2 &&
                bestTrial.deathX > oldClusterX + (flightMode(mode) ? 100.f : 160.f);

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
                hardFailure ? 2 : 1,
                haveBest ? roleReason(team.bestRole) : "team-no-candidate"
            );

            if (oldDeathRepeat >= 2 &&
                (!haveBest || (bestTrial.dead && !escapedKnownFailure))) {
                ++deadCandidatesRejected;
                int retreat = std::min(
                    std::max(1, bestPlayable.frame() - 1),
                    180 + std::min(oldDeathRepeat, 10) * 70 +
                        std::min(recoveryCount, 8) * 90
                );
                recoverFromBest(retreat);
                stagnantRounds = 0;
                emitTelemetry(2, "reject-known-death");
                continue;
            }

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

            bool routeWasDead = bestTrial.dead;
            int applyUntil = frame;
            if (bestTrial.complete) {
                applyUntil = bestTrial.frame;
            } else if (!bestTrial.dead) {
                int advance = bestTrial.frame - frame;
                int safetyDivisor = focused ? 10 :
                    flightMode(mode) ||
                    mode == VehicleType::Robot ||
                    mode == VehicleType::Spider ? 4 : 5;
                int safetyTail = focused
                    ? std::max(4, advance / safetyDivisor)
                    : std::max(12, advance / safetyDivisor);
                applyUntil = std::max(frame + 1, bestTrial.frame - safetyTail);
            } else {
                int deathBuffer = std::min(340, 150 + searchLevel * 14);
                int safeEnd = std::max(frame, bestTrial.frame - deathBuffer);
                int safeAdvance = safeEnd - frame;
                applyUntil = frame + safeAdvance / 3;
                ++deadCandidatesRejected;
                persistentLeader.clear();
            }

            if (applyUntil <= frame) {
                recoverFromBest(std::min(3200, 240 + recoveryCount * 220));
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
                observeFailure(lvl.latestState().pos.x, mode);
                recoverFromBest(std::min(3400, 320 + recoveryCount * 220));
                emitTelemetry(2, "recover-applied-death");
                continue;
            }

            bool advancedX = lvl.latestState().pos.x > furthestX + 1.f;
            if (!routeWasDead && (advancedX || reachedGoal(lvl))) {
                furthestX = std::max(furthestX, lvl.latestState().pos.x);
                bestPlayable.capture(lvl);
                trueBestFrame = std::max(trueBestFrame, lvl.currentFrame());
                fail = 0;
                numAway = 1000;
                stagnantRounds = 0;
                recoveryCount = std::max(0, recoveryCount - 2);

                if (deathRepeat >= 2 &&
                    lvl.latestState().pos.x > deathClusterX +
                        (flightMode(mode) ? 120.f : 190.f)) {
                    deathRepeat = 0;
                    deathClusterX = 0.f;
                }
                emitTelemetry(3, reachedGoal(lvl) ? "complete" : "trusted-advance");
            } else if (routeWasDead) {
                ++stagnantRounds;
                emitTelemetry(2, "untrusted-dead-prefix");
            } else if (lvl.latestState().direction < 0) {
                stagnantRounds = 0;
            } else {
                ++stagnantRounds;
            }

            int stagnationLimit = deathRepeat >= 2 ? 3 : focused ? 4 : 7;
            if (stagnantRounds >= stagnationLimit &&
                bestPlayable.frame() > 2 &&
                lvl.latestState().direction >= 0) {
                int retreat = std::min(
                    bestPlayable.frame() - 1,
                    260 + std::min(deathRepeat, 10) * 100 +
                        std::min(recoveryCount, 10) * 160
                );
                recoverFromBest(retreat);
                stagnantRounds = 0;
                fail = 1;
                numAway = std::min(8000, 1000 + recoveryCount * 500);
                emitTelemetry(2, "hard-self-correct");
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
                static_cast<uint32_t>(p1.frame), false, p1.button, 1
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
                    static_cast<uint32_t>(p2.frame), true, p2.button, 1
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
        << "solver=split100-50coop-50rogue-v6"
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
        << " hardAvoidRounds=" << hardAvoidRounds
        << " deathRepeat=" << deathRepeat
        << " deathClusterX=" << deathClusterX
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
