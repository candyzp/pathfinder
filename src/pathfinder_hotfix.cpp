// beta.246 recovery controller
//
// Keep the beam-search machinery in pathfinder.cpp, but replace its top-level
// controller. The original camila314 Pathfinder never treated one repeated
// local state as proof that the whole level was impossible: it progressively
// backed up farther and kept searching. Replay bots such as xdBot/ReplayBot/
// PosBot also preserve and restore much more player state around pads, rings,
// portals, and checkpoints instead of using X progress as the only truth.
//
// This translation unit reuses beta.246's simulator/beam helpers while giving
// the outer search loop those safer recovery semantics.

#define pathfind pathfind_beta246_impl
#include "pathfinder.cpp"
#undef pathfind

namespace {

std::vector<PathfinderInput> collapseTransitionPairs(
    std::vector<PathfinderInput> inputs,
    bool firstDown,
    bool secondDown,
    uint32_t maxGap
) {
    bool changed = true;
    while (changed) {
        changed = false;
        std::vector<bool> remove(inputs.size(), false);

        for (int player = 0; player < 2; ++player) {
            size_t previous = inputs.size();
            for (size_t i = 0; i < inputs.size(); ++i) {
                if (inputs[i].player2 != (player != 0))
                    continue;

                if (previous != inputs.size() &&
                    inputs[previous].down == firstDown &&
                    inputs[i].down == secondDown &&
                    inputs[i].frame >= inputs[previous].frame &&
                    inputs[i].frame - inputs[previous].frame <= maxGap) {
                    remove[previous] = true;
                    remove[i] = true;
                    previous = inputs.size();
                    changed = true;
                } else {
                    previous = i;
                }
            }
        }

        if (changed) {
            std::vector<PathfinderInput> compact;
            compact.reserve(inputs.size());
            for (size_t i = 0; i < inputs.size(); ++i) {
                if (!remove[i])
                    compact.push_back(inputs[i]);
            }
            inputs = std::move(compact);
        }
    }
    return inputs;
}

bool validatesCompleteRoute(
    std::string const& lvlString,
    float trustedEndX,
    std::vector<PathfinderInput> const& inputs
) {
    Level2 lvl(lvlString);
    float solveStartX = lvl.latestState().pos.x;
    if (std::isfinite(trustedEndX) && trustedEndX > solveStartX + 30.f) {
        lvl.length = trustedEndX;
        lvl.lengthSource = "trusted-gd";
    }

    bool press1 = false;
    bool press2 = false;
    size_t cursor = 0;
    uint32_t lastInputFrame = inputs.empty() ? 0u : inputs.back().frame;
    int maxFrame = std::max<int>(static_cast<int>(lastInputFrame) + 4800, 240 * 180);

    while (lvl.currentFrame() < maxFrame &&
           !lvl.latestState().dead &&
           !reachedGoal(lvl)) {
        uint32_t frame = static_cast<uint32_t>(lvl.currentFrame());
        while (cursor < inputs.size() && inputs[cursor].frame <= frame) {
            if (inputs[cursor].player2)
                press2 = inputs[cursor].down;
            else
                press1 = inputs[cursor].down;
            ++cursor;
        }

        float beforeX1 = lvl.latestState().pos.x;
        float beforeX2 = lvl.latestState2().pos.x;
        lvl.runFrame(press1, press2, 1.f / 240.f);

        bool credible = crediblePlayerStep(beforeX1, lvl.latestState(), lvl);
        if (lvl.latestState().dualActive)
            credible = credible && crediblePlayerStep(beforeX2, lvl.latestState2(), lvl);
        if (!credible)
            return false;
    }

    return !lvl.latestState().dead && reachedGoal(lvl);
}

void rebuildMacro(PathfinderResult& result) {
    Replay2 replay;
    for (auto const& input : result.inputs)
        replay.inputs.push_back(gdr::Input(input.frame, 1, input.player2, input.down));
    result.macro = replay.exportData().unwrapOr({});
}

void simplifyCompletedInputs(
    std::string const& lvlString,
    float trustedEndX,
    PathfinderResult& result
) {
    if (!result.complete || result.inputs.size() < 4)
        return;

    auto original = result.inputs;

    // Prefer cleaner holds when they are simulator-equivalent. The original
    // Pathfinder used input count as a tie-breaker; beta.246 intentionally
    // removed that preference and can therefore emit pointless tap trains.
    for (uint32_t gap : {2u, 1u}) {
        auto candidate = collapseTransitionPairs(original, false, true, gap);
        candidate = collapseTransitionPairs(std::move(candidate), true, false, 1u);
        if (candidate.size() < original.size() &&
            validatesCompleteRoute(lvlString, trustedEndX, candidate)) {
            result.inputs = std::move(candidate);
            rebuildMacro(result);
            return;
        }
    }

    auto candidate = collapseTransitionPairs(original, true, false, 1u);
    if (candidate.size() < original.size() &&
        validatesCompleteRoute(lvlString, trustedEndX, candidate)) {
        result.inputs = std::move(candidate);
        rebuildMacro(result);
    }
}

size_t cleanerEquivalentCandidate(SearchBatch const& batch, VehicleType mode) {
    if (batch.candidates.empty())
        return 0;

    size_t chosen = 0;
    int frameSlack = mode == VehicleType::Wave ? 2 : continuousMode(mode) ? 5 : 10;
    float xSlack = mode == VehicleType::Wave ? 4.f : continuousMode(mode) ? 8.f : 14.f;

    for (size_t i = 1; i < batch.candidates.size(); ++i) {
        auto const& best = batch.candidates[chosen];
        auto const& candidate = batch.candidates[i];

        if (candidate.trial.complete != best.trial.complete ||
            candidate.trial.validProgress != best.trial.validProgress ||
            candidate.trial.dead != best.trial.dead ||
            candidate.trial.survivedHorizon != best.trial.survivedHorizon) {
            continue;
        }
        if (candidate.trial.frame + frameSlack < best.trial.frame)
            continue;
        if (candidate.trial.x + xSlack < best.trial.x)
            continue;
        if (candidate.trial.clearance + 2.f < best.trial.clearance)
            continue;

        // Only trade a tiny amount of equivalent progress for a meaningful
        // reduction in toggles. Wave keeps a much smaller bias because rapid
        // switching can genuinely be required there.
        size_t requiredSaving = mode == VehicleType::Wave ? 4 : 2;
        if (candidate.inputs.size() + requiredSaving <= best.inputs.size())
            chosen = i;
    }
    return chosen;
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
    float simulatorInferredLength = lvl.inferredLength;
    bool hasTrustedEnd = std::isfinite(trustedEndX) && trustedEndX > solveStartX + 30.f;
    if (hasTrustedEnd) {
        lvl.length = trustedEndX;
        lvl.lengthSource = "trusted-gd";
    }

    auto progressFor = [&](float x, bool complete) -> double {
        if (complete)
            return 100.0;
        double span = static_cast<double>(lvl.length - solveStartX);
        if (span <= 1.0)
            return 0.0;
        return std::clamp(
            ((static_cast<double>(x) - solveStartX) / span) * 100.0,
            0.0,
            99.99
        );
    };

    uint32_t seed = static_cast<uint32_t>(std::hash<std::string>{}(lvlString));
    seed ^= static_cast<uint32_t>(std::llround(std::max(0.f, trustedEndX)));
    std::random_device randomDevice;
    seed ^= randomDevice();
    std::mt19937 rng(seed);

    BestTimeline best(lvl);
    std::deque<AlternativeBranch> alternatives;
    int bestGeneration = 0;
    bool currentExtendsBest = true;
    float furthestX = solveStartX;
    float lastDeathX = solveStartX;
    int recoveryLevel = 0;
    int repeatedDeathZone = 0;
    int stagnantRounds = 0;
    int deepBacktracks = 0;
    int fullRestarts = 0;
    int maxRootRetries = 0;
    std::string lastRecoveryReason = "start";
    std::unordered_map<uint64_t, int> failedRoots;
    bool solverExhausted = false;
    SearchStats totals;

    auto mergeStats = [&totals](SearchStats const& stats) {
        totals.expandedStates += stats.expandedStates;
        totals.simulatedFrames += stats.simulatedFrames;
        totals.deduplicatedStates += stats.deduplicatedStates;
        totals.rejectedJumps += stats.rejectedJumps;
        totals.horizon = stats.horizon;
        totals.beamWidth = stats.beamWidth;
    };

    auto emitTelemetry = [&](std::string reason, float deathX = 0.f) {
        if (!callback)
            return;
        PathfinderTelemetry telemetry;
        telemetry.progress = progressFor(furthestX, reachedGoal(lvl));
        telemetry.startX = solveStartX;
        telemetry.currentX = lvl.latestState().pos.x;
        telemetry.furthestX = furthestX;
        telemetry.trustedEndX = hasTrustedEnd ? trustedEndX : 0.f;
        telemetry.inferredLength = simulatorInferredLength;
        telemetry.frame = lvl.currentFrame();
        telemetry.mode = vehicleName(lvl.latestState().vehicle.type);
        telemetry.checkpointFrame = best.frame();
        telemetry.checkpointX = best.x();
        telemetry.deathX = deathX;
        // main.cpp only dumps verbose state logs for reasons containing ':'.
        // Keep the UI informed without flooding Geode's log during recovery.
        telemetry.recoveryReason = reason.find(':') == std::string::npos
            ? std::move(reason)
            : std::string("recovering");
        callback(telemetry);
    };

    auto restartFromBeginning = [&](std::string const& reason) {
        best.restore(lvl);
        lvl.rollback(1);
        lvl.syncPresses();
        currentExtendsBest = false;
        alternatives.clear();
        failedRoots.clear();
        recoveryLevel = 0;
        repeatedDeathZone = 0;
        stagnantRounds = 0;
        ++fullRestarts;
        lastRecoveryReason = reason;
        rng.seed(seed ^ static_cast<uint32_t>(fullRestarts * 0x9e3779b9u));
        emitTelemetry("recovering", lastDeathX);
    };

    auto recover = [&](std::string reason, int extraRetreat, bool forceRetreat) {
        // Alternate branches are useful until a pad/portal collapses many input
        // histories into the same post-effect state. Once a root repeats, skip
        // sibling cycling and deliberately approach the effect from farther back.
        if (!forceRetreat) {
            while (!alternatives.empty()) {
                AlternativeBranch alternative = std::move(alternatives.back());
                alternatives.pop_back();
                if (alternative.generation != bestGeneration ||
                    alternative.baseFrame >= best.frame() ||
                    alternative.applyUntil <= alternative.baseFrame) {
                    continue;
                }

                best.restore(lvl);
                lvl.rollback(alternative.baseFrame);
                lvl.syncPresses();
                auto applied = applyInputsUntil(lvl, alternative.inputs, alternative.applyUntil);
                if (applied.valid && !lvl.latestState().dead &&
                    lvl.currentFrame() > alternative.baseFrame + 8) {
                    currentExtendsBest = false;
                    recoveryLevel = std::min(12, recoveryLevel + 1);
                    repeatedDeathZone = 0;
                    stagnantRounds = 0;
                    lastRecoveryReason = reason + ":alternate-branch";
                    emitTelemetry("recovering", lastDeathX);
                    return;
                }
            }
        }

        best.restore(lvl);
        int available = std::max(0, best.frame() - 1);
        int retreat = std::min(
            available,
            180 + recoveryLevel * 160 + stagnantRounds * 80 + extraRetreat
        );
        lvl.rollback(std::max(1, best.frame() - retreat));
        lvl.syncPresses();
        currentExtendsBest = lvl.currentFrame() == best.frame();
        recoveryLevel = std::min(12, recoveryLevel + 1);
        repeatedDeathZone = 0;
        stagnantRounds = 0;
        if (forceRetreat)
            ++deepBacktracks;
        lastRecoveryReason = reason + (forceRetreat ? ":deep-retreat" : ":retreat");
        rng.seed(seed ^ static_cast<uint32_t>(recoveryLevel * 0x45d9f3bu) ^
                 static_cast<uint32_t>(lvl.currentFrame() * 7919) ^
                 static_cast<uint32_t>(deepBacktracks * 104729));
        emitTelemetry("recovering", lastDeathX);
    };

    while (!reachedGoal(lvl) && !stop.load()) {
        if (lvl.latestState().dead) {
            recover("committed-death", 300, false);
            continue;
        }

        int frame = lvl.currentFrame();
        float rootX = lvl.latestState().pos.x;
        VehicleType mode = lvl.latestState().vehicle.type;
        bool rootIsBest = currentExtendsBest && frame == best.frame();
        uint64_t rootHash = hashSnapshot(captureSnapshot(lvl));
        int previousRootFailures = failedRoots.contains(rootHash) ? failedRoots[rootHash] : 0;
        int difficulty = std::min(7,
            recoveryLevel / 2 + repeatedDeathZone + stagnantRounds + previousRootFailures / 2
        );

        // Camila's original solver always looked roughly 1000 frames ahead. The
        // beta.246 beam planner is normally shorter for speed, but when a pad or
        // portal repeatedly funnels us into the same state, widen the horizon so
        // the search can judge the landing/next obstacle rather than only the bounce.
        int maximumHorizon = continuousMode(mode) ? 700 : 600;
        maximumHorizon += difficulty * (continuousMode(mode) ? 55 : 45);
        maximumHorizon += std::min(700, previousRootFailures * 100);
        maximumHorizon = std::min(1500, maximumHorizon);

        auto batch = searchBestInputs(lvl, stop, rng, maximumHorizon, difficulty);
        mergeStats(batch.stats);
        if (stop.load())
            break;

        size_t chosenIndex = cleanerEquivalentCandidate(batch, mode);
        auto const& chosen = batch.candidates[chosenIndex];

        // Keep the endpoint-corruption guard. This is a different failure class
        // from local pad convergence: if almost every very first step is an
        // impossible X teleport, continuing cannot produce trustworthy physics.
        if (best.frame() <= 1 && frame <= 1 && batch.stats.expandedStates > 50 &&
            batch.stats.rejectedJumps * 3 > batch.stats.expandedStates) {
            lastRecoveryReason = "unrecoverable-invalid-x-jump";
            solverExhausted = true;
            break;
        }

        if (rootIsBest) {
            for (size_t i = 0; i < batch.candidates.size() && i < 7; ++i) {
                if (i == chosenIndex)
                    continue;
                auto const& candidate = batch.candidates[i];
                int safeFrames = safePrefixFrames(candidate, frame, mode);
                if (safeFrames < 18)
                    continue;

                AlternativeBranch alternative;
                alternative.generation = bestGeneration;
                alternative.baseFrame = frame;
                alternative.baseX = rootX;
                alternative.applyUntil = frame + std::min(safeFrames, 360);
                alternative.inputs = candidate.inputs;
                alternative.inputHash = hashInputs(alternative.inputs);

                bool duplicate = std::any_of(
                    alternatives.begin(), alternatives.end(),
                    [&](AlternativeBranch const& existing) {
                        return existing.generation == alternative.generation &&
                               existing.baseFrame == alternative.baseFrame &&
                               existing.inputHash == alternative.inputHash;
                    }
                );
                if (!duplicate)
                    alternatives.push_back(std::move(alternative));
            }
            while (alternatives.size() > 28)
                alternatives.pop_front();
        }

        int commitFrames = 0;
        if (chosen.trial.complete) {
            commitFrames = std::max(0, chosen.trial.frame - frame);
        } else if (chosen.trial.dead) {
            if (std::abs(chosen.trial.x - lastDeathX) <= 40.f)
                ++repeatedDeathZone;
            else
                repeatedDeathZone = 1;
            lastDeathX = chosen.trial.x;

            commitFrames = safePrefixFrames(chosen, frame, mode);
            float deathGain = chosen.trial.x - rootX;
            if (commitFrames < 18 || (deathGain < 20.f && lvl.latestState().direction >= 0)) {
                int failures = ++failedRoots[rootHash];
                maxRootRetries = std::max(maxRootRetries, failures);

                // Do not declare the level done/failed here. Pads and portals can
                // legitimately collapse many different input histories into one
                // state. Escalate exactly like the old Pathfinder: retry, back up
                // farther, then eventually restart the approach from frame 1.
                if (failures >= 14) {
                    restartFromBeginning("repeated-state-full-restart");
                    continue;
                }

                int extraRetreat = 160 + std::min(failures, 12) * 260;
                bool forceRetreat = failures >= 2;
                recover(
                    repeatedDeathZone >= 2 ? "repeated-death-zone" : "no-safe-prefix",
                    extraRetreat,
                    forceRetreat
                );
                continue;
            }
        } else {
            int span = std::max(0, chosen.trial.frame - frame);
            commitFrames = safePrefixFrames(chosen, frame, mode);
            if (chosen.trial.survivedHorizon && chosen.inputs.empty() && !continuousMode(mode))
                commitFrames = span * 78 / 100;
            if (commitFrames <= 0) {
                int failures = ++failedRoots[rootHash];
                maxRootRetries = std::max(maxRootRetries, failures);
                if (failures >= 14) {
                    restartFromBeginning("zero-advance-full-restart");
                    continue;
                }
                recover("zero-advance", 220 + failures * 180, failures >= 2);
                continue;
            }
        }

        int cap = mode == VehicleType::Wave ? 190 : continuousMode(mode) ? 250 : 360;
        if (!chosen.trial.complete)
            commitFrames = std::min(commitFrames, cap);

        auto applied = applyInputsUntil(lvl, chosen.inputs, frame + commitFrames);
        if (!applied.valid) {
            ++totals.rejectedJumps;
            lastDeathX = applied.deathX;
            int failures = ++failedRoots[rootHash];
            maxRootRetries = std::max(maxRootRetries, failures);
            recover("invalid-x-jump", 400 + failures * 180, failures >= 2);
            continue;
        }
        if (lvl.currentFrame() <= frame) {
            ++stagnantRounds;
            int failures = ++failedRoots[rootHash];
            maxRootRetries = std::max(maxRootRetries, failures);
            recover("no-frame-advance", 260 + failures * 160, failures >= 2);
            continue;
        }
        if (lvl.latestState().dead) {
            lastDeathX = lvl.latestState().pos.x;
            int failures = ++failedRoots[rootHash];
            maxRootRetries = std::max(maxRootRetries, failures);
            recover("prefix-died", 320 + failures * 180, failures >= 2);
            continue;
        }

        float previousFurthest = furthestX;
        furthestX = std::max(furthestX, lvl.latestState().pos.x);
        bool newFurthest = furthestX > previousFurthest + 1.f;

        if (currentExtendsBest) {
            best.appendFrom(lvl);
        } else if (newFurthest || reachedGoal(lvl)) {
            best.capture(lvl);
            currentExtendsBest = true;
            ++bestGeneration;
            alternatives.clear();
        }

        if (newFurthest) {
            // Successful forward progress means the local trap was not globally
            // impossible. Forget old root-failure evidence just as the original
            // solver reset its fail counter after extending trueBest.
            failedRoots.clear();
            stagnantRounds = 0;
            repeatedDeathZone = 0;
            recoveryLevel = std::max(0, recoveryLevel - 1);
            lastRecoveryReason = "advance";
        } else if (lvl.latestState().direction < 0) {
            stagnantRounds = 0;
            lastRecoveryReason = "reverse-traversal";
        } else {
            ++stagnantRounds;
            lastRecoveryReason = "replaying-checkpoint";
        }

        int stallLimit = currentExtendsBest ? 5 : 10;
        if (stagnantRounds >= stallLimit && best.frame() > 1) {
            int extra = 360 + std::min(stagnantRounds, 10) * 180;
            recover("x-stagnation", extra, stagnantRounds >= 7);
            continue;
        }

        emitTelemetry(lastRecoveryReason, chosen.trial.dead ? chosen.trial.x : 0.f);
    }

    if (!lvl.latestState().dead && reachedGoal(lvl)) {
        best.capture(lvl);
        currentExtendsBest = true;
        furthestX = std::max(furthestX, lvl.latestState().pos.x);
    } else if (currentExtendsBest && lvl.currentFrame() > best.frame()) {
        best.appendFrom(lvl);
    }

    PathfinderResult result;
    Replay2 output;
    for (size_t i = 1; i < best.p1.size(); ++i) {
        auto const& p1 = best.p1[i];
        auto const& previousP1 = best.p1[i - 1];
        if (p1.frame > 1 && p1.button != previousP1.button) {
            output.inputs.push_back(gdr::Input(p1.frame, 1, false, p1.button));
            result.inputs.push_back({static_cast<uint32_t>(p1.frame), false, p1.button});
        }

        if (i < best.p2.size()) {
            auto const& p2 = best.p2[i];
            auto const& previousP2 = best.p2[i - 1];
            if (p2.dualActive && p2.frame > 1 && p2.button != previousP2.button) {
                output.inputs.push_back(gdr::Input(p2.frame, 1, true, p2.button));
                result.inputs.push_back({static_cast<uint32_t>(p2.frame), true, p2.button});
            }
        }
    }

    std::sort(result.inputs.begin(), result.inputs.end(), [](auto const& a, auto const& b) {
        if (a.frame != b.frame)
            return a.frame < b.frame;
        return a.player2 < b.player2;
    });

    result.macro = output.exportData().unwrapOr({});
    result.complete = !best.p1.empty() && !best.p1.back().dead && best.p1.back().completed;
    result.progress = progressFor(furthestX, result.complete);

    std::map<int, size_t> unsupportedGameplay;
    for (auto const& object : lvl.unsupportedObjects) {
        if (unsupportedMechanicName(object.objectID))
            ++unsupportedGameplay[object.objectID];
    }

    std::ostringstream diagnostics;
    diagnostics << std::fixed << std::setprecision(2)
                << "startX=" << solveStartX
                << " currentX=" << (best.p1.empty() ? solveStartX : best.p1.back().pos.x)
                << " furthestX=" << furthestX
                << " trustedEndX=" << (hasTrustedEnd ? trustedEndX : 0.f)
                << " inferredLength=" << simulatorInferredLength
                << " endpointSource=" << lvl.lengthSource
                << " frame=" << (best.p1.empty() ? 0 : best.p1.back().frame)
                << " mode=" << (best.p1.empty() ? "Unknown" : vehicleName(best.p1.back().vehicle.type))
                << " checkpointFrame=" << best.frame()
                << " checkpointX=" << best.x()
                << " deathX=" << lastDeathX
                << " recovery=" << lastRecoveryReason
                << " exhausted=" << (solverExhausted ? 1 : 0)
                << " deepBacktracks=" << deepBacktracks
                << " fullRestarts=" << fullRestarts
                << " maxRootRetries=" << maxRootRetries
                << " expanded=" << totals.expandedStates
                << " simulatedFrames=" << totals.simulatedFrames
                << " deduplicated=" << totals.deduplicatedStates
                << " rejectedJumps=" << totals.rejectedJumps
                << " lastHorizon=" << totals.horizon
                << " lastBeam=" << totals.beamWidth;

    if (!unsupportedGameplay.empty()) {
        diagnostics << " unsupportedGameplay=";
        bool first = true;
        for (auto const& [id, count] : unsupportedGameplay) {
            if (!first)
                diagnostics << ',';
            first = false;
            diagnostics << unsupportedMechanicName(id) << '(' << id << ")x" << count;
        }
    }
    result.diagnostics = diagnostics.str();

    // Never arm a partial route. It can still be reported as "Best path" in the
    // UI, but autoplay must only receive a simulator-confirmed completion.
    if (!result.complete) {
        result.inputs.clear();
        result.macro.clear();
    } else {
        simplifyCompletedInputs(lvlString, trustedEndX, result);
    }

    return result;
}
