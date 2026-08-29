#include <mutex>
#include <utility>

// Keep the simulator, dynamic-geometry model, candidate generators and replay
// export code in one implementation. Rename the old controller in this
// translation unit so the public controller below can orchestrate three
// cooperating worker groups without duplicating the simulator itself.
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
    int candidateCount = 0;
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

    int maxP1 = maxToggleBudget(mode) + 4 + (lane % 5) * 2;
    int maxP2 = dual
        ? maxToggleBudget(base.latestState2().vehicle.type) + 2 + (lane % 4) * 2
        : 0;

    // Every brute-force lane uses a different time scale. Some hammer the next
    // few moments while others deliberately throw inputs across the full horizon.
    int focusWindow = std::min(
        horizonFrames,
        90 + lane * std::max(30, horizonFrames / 12)
    );
    focusWindow = std::max(1, focusWindow);

    std::uniform_int_distribution<int> fullFrame(0, std::max(0, horizonFrames - 1));
    std::uniform_int_distribution<int> focusFrame(0, focusWindow - 1);
    std::uniform_int_distribution<int> p1Budget(0, std::max(0, maxP1));
    std::uniform_int_distribution<int> p2Budget(0, std::max(0, maxP2));

    int p1Count = p1Budget(rng);
    int p2Count = p2Budget(rng);

    auto randomOffset = [&](int index) {
        bool focused = ((index + attempt + lane) % 4) != 0;
        return focused ? focusFrame(rng) : fullFrame(rng);
    };

    for (int i = 0; i < p1Count; ++i)
        addToggle(inputs, frame, horizonFrames, randomOffset(i), false);
    for (int i = 0; i < p2Count; ++i)
        addToggle(inputs, frame, horizonFrames, randomOffset(i + p1Count), true);

    // A few brute lanes intentionally create complete press/release windows.
    // This gives Robot, Spider and dash-orb sections a chance even inside the
    // otherwise unconstrained brute-force group.
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

    // Before another worker has published a live leader, refinement workers are
    // still useful: each one scouts a deterministic timing lane instead of idling.
    if (route.empty()) {
        int usable = std::max(1, std::min(horizonFrames, 480));
        int offset = (lane * 37 + attempt * 17) % usable;

        if (dashing) {
            // If the current state is already dashing, keeping the button held is
            // the baseline. Search different release points instead of cancelling it.
            int releaseAt = std::min(horizonFrames - 1, 36 + lane * 24 + attempt * 3);
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
        // Fine timing nudge around one decision.
        static constexpr int deltas[] = {-24, -12, -6, -3, -1, 1, 3, 6, 12, 24};
        int delta = deltas[(lane * 3 + attempt) % (sizeof(deltas) / sizeof(deltas[0]))];
        int shifted = std::clamp(chosenFrame + delta, frame, frame + horizonFrames - 1);
        route.erase(chosen);
        route.insert(inputKey(static_cast<uint32_t>(shifted), chosenP2));
    } else if (strategy == 1) {
        // Test whether one of the leader's inputs is unnecessary or harmful.
        route.erase(chosen);
    } else if (strategy == 2) {
        // Add a correction near the front of the current horizon.
        int localWindow = std::max(1, std::min(horizonFrames, 360));
        std::uniform_int_distribution<int> dist(0, localWindow - 1);
        addToggle(route, frame, horizonFrames, dist(rng), false);
    } else if (strategy == 3) {
        // Add a press/release pair while leaving the rest of the leader intact.
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
        // Shift the whole surviving idea a few frames, useful for portals and
        // corridors where the route shape is right but phase is wrong.
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
        // Stretch or shorten an existing hold by moving its release edge.
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
        // Dual sections get a cross-player experiment; otherwise add one sparse
        // late correction so this lane still explores a distinct mutation family.
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

    constexpr int kRoleWorkers = 10;
    constexpr int kTotalWorkers = 30;
    int generatedAttempts = std::clamp(24 + searchLevel * 2, 24, 48);
    out.workerCount = kTotalWorkers;
    out.candidateCount = static_cast<int>(
        specialists.size() + generatedAttempts * kRoleWorkers * 2
    );

    std::atomic_bool batchSolved = false;
    std::atomic<uint64_t> trials = 0;
    std::mutex sharedMutex;

    size_t bestToggleCount = std::numeric_limits<size_t>::max();
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

    auto publish = [&](std::set<SearchInput> const& candidate, TrialResult const& trial, TeamRole role) {
        trials.fetch_add(1, std::memory_order_relaxed);

        {
            std::lock_guard<std::mutex> guard(sharedMutex);
            if (trial.dead)
                out.lastDeathX = trial.deathX;

            if (betterTrial(
                    trial,
                    candidate.size(),
                    out.haveBest,
                    out.bestTrial,
                    bestToggleCount,
                    mode
                )) {
                out.bestInputs = candidate;
                out.bestTrial = trial;
                out.haveBest = true;
                out.bestRole = role;
                bestToggleCount = candidate.size();
            }

            // Refinement workers only imitate routes that are still alive. A route
            // the simulator already killed is evidence, not a leader.
            if (!trial.dead && betterTrial(
                    trial,
                    candidate.size(),
                    haveLiveLeader,
                    liveLeaderTrial,
                    liveLeaderToggleCount,
                    mode
                )) {
                liveLeader = candidate;
                liveLeaderTrial = trial;
                liveLeaderToggleCount = candidate.size();
                haveLiveLeader = true;
            }
        }

        if (trial.complete)
            batchSolved.store(true, std::memory_order_release);
    };

    std::vector<std::thread> threads;
    threads.reserve(kTotalWorkers);

    // Workers 1-10: structured specialists. Every worker takes a different stride
    // through the mode-aware candidate library, so no two workers own the same route.
    for (int lane = 0; lane < kRoleWorkers; ++lane) {
        threads.emplace_back([&, lane] {
            Level2 worker = base;
            worker.fixStatePointers();

            for (size_t index = static_cast<size_t>(lane);
                 index < specialists.size() &&
                 !stop.load() &&
                 !batchSolved.load(std::memory_order_acquire);
                 index += kRoleWorkers) {
                TrialResult trial = tryInputs(worker, specialists[index], horizonFrames);
                publish(specialists[index], trial, TeamRole::Specialist);
            }
        });
    }

    // Workers 11-20: brute force. The ten lanes use different timing windows and
    // toggle budgets so this is ten independent searches rather than duplicate RNG.
    for (int lane = 0; lane < kRoleWorkers; ++lane) {
        threads.emplace_back([&, lane] {
            Level2 worker = base;
            worker.fixStatePointers();
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
                auto candidate = makeBruteCandidate(
                    base,
                    horizonFrames,
                    lane,
                    attempt,
                    localRng
                );
                TrialResult trial = tryInputs(worker, candidate, horizonFrames);
                publish(candidate, trial, TeamRole::BruteForce);
            }
        });
    }

    // Workers 21-30: cooperative refinement. They continuously copy the best live
    // route found by either of the other teams, then mutate a different part of it.
    // If nobody has published one yet, they start from the previous search winner or
    // from a structured scout route, so all ten workers stay useful immediately.
    for (int lane = 0; lane < kRoleWorkers; ++lane) {
        threads.emplace_back([&, lane] {
            Level2 worker = base;
            worker.fixStatePointers();
            std::mt19937 localRng(
                baseSeed ^
                static_cast<uint32_t>((frame + 17) * 2246822519u) ^
                static_cast<uint32_t>((lane + 21) * 3266489917u)
            );

            for (int attempt = 0;
                 attempt < generatedAttempts &&
                 !stop.load() &&
                 !batchSolved.load(std::memory_order_acquire);
                 ++attempt) {
                std::set<SearchInput> leader;
                {
                    std::lock_guard<std::mutex> guard(sharedMutex);
                    if (haveLiveLeader)
                        leader = liveLeader;
                    else
                        leader = seedLeader;
                }

                if (leader.empty() && !specialists.empty()) {
                    size_t seedIndex = static_cast<size_t>(
                        (lane + attempt * kRoleWorkers) % specialists.size()
                    );
                    leader = specialists[seedIndex];
                }

                auto candidate = mutateLeaderCandidate(
                    base,
                    leader,
                    horizonFrames,
                    lane,
                    attempt,
                    localRng
                );
                TrialResult trial = tryInputs(worker, candidate, horizonFrames);
                publish(candidate, trial, TeamRole::Refinement);
            }
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
        telemetry.mode = "team30-cooperative";
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

            int generatedAttempts = std::clamp(24 + searchLevel * 2, 24, 48);
            debugVehicleType = static_cast<int>(mode);
            debugSearchLevel = searchLevel;
            debugHorizon = horizonFrames;
            debugWorkers = 30;
            debugCandidateCount = static_cast<int>(
                specialists.size() + generatedAttempts * 20
            );
            debugClearance = 0.f;
            emitTelemetry(0, "team30-search");

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
        << "solver=team30-cooperative"
        << " progress=" << result.progress
        << " frame=" << bestPlayable.frame()
        << " routeX=" << bestPlayable.x()
        << " furthestX=" << furthestX
        << " endX=" << lvl.length
        << " recoveryCount=" << recoveryCount
        << " fullRestarts=" << fullRestarts
        << " hardestSearch=" << hardestSearchLevel
        << " totalTrials=" << totalTrials
        << " workers=" << debugWorkers
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
