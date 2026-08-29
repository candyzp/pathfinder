#include "pathfinder_team_v3.cpp"

namespace {

void appendHardFailureCandidatesV4(
    Level2 const& base,
    int horizonFrames,
    std::set<SearchInput> const& seedLeader,
    int failureOffset,
    int repeatCount,
    std::vector<std::set<SearchInput>>& candidates
) {
    if (repeatCount < 2 || failureOffset < 0 || failureOffset >= horizonFrames)
        return;

    int frame = base.currentFrame();
    VehicleType mode = base.latestState().vehicle.type;

    // A death marks the collision frame, not necessarily the frame where the
    // corrective input belongs. Search progressively farther *before* the death.
    // This is the important difference from v3's narrow +/-36 frame sweep.
    static constexpr std::array<int, 22> groundLeads = {
        320, 280, 240, 210, 185, 165, 145, 128, 112, 96, 82,
        70, 58, 48, 38, 30, 24, 18, 12, 8, 4, 0
    };
    static constexpr std::array<int, 13> flightLeads = {
        300, 240, 200, 170, 140, 115, 90, 70, 52, 36, 24, 12, 0
    };

    if (flightMode(mode)) {
        for (int lead : flightLeads) {
            int start = failureOffset - lead;
            if (start < 0 || start >= horizonFrames)
                continue;
            for (int width : {4, 8, 12, 18, 24, 32, 44, 60, 84}) {
                if (start + width >= horizonFrames)
                    continue;
                auto route = windowedRoute(seedLeader, frame, horizonFrames);
                eraseP1Window(
                    route,
                    frame,
                    std::max(0, start - 8),
                    std::min(horizonFrames - 1, failureOffset + 24)
                );
                addToggle(route, frame, horizonFrames, start, false);
                addToggle(route, frame, horizonFrames, start + width, false);
                candidates.push_back(std::move(route));
            }
        }
        return;
    }

    std::array<int, 10> widths = mode == VehicleType::Robot
        ? std::array<int, 10>{1, 2, 4, 7, 10, 14, 20, 28, 36, 48}
        : mode == VehicleType::Spider
            ? std::array<int, 10>{1, 2, 3, 4, 6, 12, 24, 48, 72, 108}
            : std::array<int, 10>{1, 2, 4, 7, 10, 14, 20, 28, 40, 56};

    int leadIndex = 0;
    for (int lead : groundLeads) {
        int pressAt = failureOffset - lead;
        if (pressAt < 0 || pressAt >= horizonFrames)
            continue;

        for (int width : widths) {
            auto route = windowedRoute(seedLeader, frame, horizonFrames);

            // Remove the old local decision around the dangerous section first.
            // Otherwise a new jump can be layered on top of the exact lethal input
            // and appear to be a different route while behaving identically.
            eraseP1Window(
                route,
                frame,
                std::max(0, pressAt - 10),
                std::min(horizonFrames - 1, failureOffset + 28)
            );
            forceFreshPulse(route, base, horizonFrames, pressAt, width);
            candidates.push_back(std::move(route));
        }

        // Every few lead positions also try a clean-slate local route. This prevents
        // all recovery candidates from inheriting a poisoned champion prefix.
        if ((leadIndex++ % 3) == 0) {
            for (int width : {1, 4, 12, 28}) {
                std::set<SearchInput> clean;
                forceFreshPulse(clean, base, horizonFrames, pressAt, width);
                candidates.push_back(std::move(clean));
            }
        }
    }

    // Explicitly test "do nothing near the failure" as well. Sometimes the
    // learned bad behavior is an unnecessary click into a spike/orb.
    auto noLocalInput = windowedRoute(seedLeader, frame, horizonFrames);
    eraseP1Window(
        noLocalInput,
        frame,
        std::max(0, failureOffset - 340),
        std::min(horizonFrames - 1, failureOffset + 36)
    );
    candidates.push_back(std::move(noLocalInput));
}

} // namespace

PathfinderResult pathfind_v4(
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
        telemetry.workerCount = debugHelpers;
        telemetry.phase = phase;
        telemetry.totalTrials = totalTrials;
        telemetry.mode = "hard-negative-team50-30thread-v4";
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

            // V3 only focused if the repeated death was <=420 frames away. That is
            // why the screenshot could sit at horizon 1660 forever. Any repeated
            // reachable death now forces a focused batch.
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

                // Keep the collision itself inside the focused horizon. V3 could
                // focus around a far death and then cap the horizon before reaching it.
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
            debugVehicleType = static_cast<int>(mode);
            debugSearchLevel = searchLevel;
            debugHorizon = horizonFrames;
            debugHelpers = 50;
            debugPhysicalWorkers = 30;
            debugCandidateCount = static_cast<int>(
                specialists.size() + generatedAttempts * 20
            );
            debugClearance = 0.f;
            emitTelemetry(
                hardFailure ? 2 : 0,
                hardFailure ? "hard-negative-search" :
                    focused ? "focused-search" : "adaptive-search"
            );

            auto liveReport = [&](uint64_t batchTrials, float liveX) {
                if (!callback)
                    return;
                PathfinderTelemetry telemetry = makeTelemetry(
                    hardFailure ? 2 : 1,
                    hardFailure ? "live-hard-negative" :
                        focused ? "live-focused" : "live-search"
                );
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

            // Only the batch winner teaches the death cluster. In v3, *any* dead
            // worker could move lastDeathX and destabilize the focus target.
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

            // Repeated fatal routes are hard negatives. Do not inch the checkpoint
            // toward a spike just because the first part of a doomed route survived.
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
                // First-time/advanced deaths may still contribute a conservative
                // prefix, but that prefix is NOT promoted to the trusted checkpoint.
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

            // Never promote a checkpoint produced by a route already known to die.
            // This is the core fix for the Theory of Everything / Clutterfunk loop.
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
        << "solver=hard-negative-team50-30thread-v4"
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
