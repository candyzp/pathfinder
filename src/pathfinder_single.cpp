// Single-controller Pathfinder V15 with local failure learning.
//
// There is still exactly one decision-making solver. Parallel threads only
// evaluate candidates. V15 adds persistent per-region timing memory, dense
// early dash-release candidates, repeated-death-basin detection, and bounded
// strategy resets so the solver cannot repeat one losing hold forever.
#define pathfind pathfind_plain_base_v15
#include "pathfinder.cpp"
#undef pathfind

#include "solver_dashboard.hpp"

#include <chrono>

namespace {

constexpr int kMaxStrategyEpochsV15 = 4;
constexpr uint64_t kEpochStallTrialsV15 = 70000;
constexpr auto kEpochSoftStallTimeV15 = std::chrono::seconds(4);
constexpr auto kEpochHardStallTimeV15 = std::chrono::seconds(12);
constexpr float kMeaningfulAdvanceV15 = 8.f;
constexpr float kDeathBucketSizeV15 = 24.f;
constexpr int kNormalBanThresholdV15 = 3;
constexpr int kDashBanThresholdV15 = 2;
constexpr int kMinCandidatesAfterPruneV15 = 16;

struct LearningMemoryV15 {
    std::unordered_map<uint64_t, int> failures;
    int lastDeathBucket = std::numeric_limits<int>::min();
    int sameBasinRounds = 0;
    uint64_t learnedDeaths = 0;
    uint64_t prunedCandidates = 0;
};

uint64_t mixV15(uint64_t h, uint64_t value) {
    h ^= value + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
    h *= 1099511628211ull;
    return h;
}

uint64_t candidateKeyV15(
    std::set<SearchInput> const& inputs,
    int frame,
    float x,
    VehicleType mode,
    bool dashing
) {
    uint64_t h = 1469598103934665603ull;
    int region = static_cast<int>(std::floor(x / 48.f));
    h = mixV15(h, static_cast<uint32_t>(region));
    h = mixV15(h, static_cast<uint32_t>(static_cast<int>(mode) + 1));
    h = mixV15(h, dashing ? 0xd45u : 0x51u);
    h = mixV15(h, static_cast<uint32_t>(inputs.size()));

    int count = 0;
    for (SearchInput key : inputs) {
        uint32_t inputFrame = key >> 1;
        bool player2 = (key & 1u) != 0;
        int offset = static_cast<int>(inputFrame) - frame;
        uint64_t packed =
            (static_cast<uint64_t>(static_cast<uint32_t>(offset + 4096)) << 2) |
            (player2 ? 2ull : 1ull);
        h = mixV15(h, packed);
        if (++count >= 40)
            break;
    }
    return h;
}

int failureCountV15(
    LearningMemoryV15 const& memory,
    std::set<SearchInput> const& inputs,
    int frame,
    float x,
    VehicleType mode,
    bool dashing
) {
    uint64_t key = candidateKeyV15(inputs, frame, x, mode, dashing);
    auto it = memory.failures.find(key);
    return it == memory.failures.end() ? 0 : it->second;
}

void appendDashLearningCandidatesV15(
    std::vector<std::set<SearchInput>>& candidates,
    int frame,
    int horizonFrames,
    bool heldP1,
    bool dashingP1
) {
    if (!dashingP1 || !heldP1)
        return;

    // The old dash search started deliberate releases at +24f. These dense
    // early releases are the missing "let go sooner" choices.
    static constexpr int releasePoints[] = {
        1, 2, 3, 4, 5, 6, 8, 10, 12, 14, 16, 18, 20, 22,
        24, 28, 32, 36, 40, 48, 60, 72, 84, 96, 120, 144, 192
    };
    static constexpr int repressGaps[] = {1, 2, 3, 4, 6, 8, 12, 16, 24, 36};

    for (int releaseAt : releasePoints) {
        if (releaseAt >= horizonFrames)
            continue;

        std::set<SearchInput> releaseOnly;
        addToggle(releaseOnly, frame, horizonFrames, releaseAt, false);
        candidates.push_back(releaseOnly);

        for (int gap : repressGaps) {
            if (releaseAt + gap + 1 >= horizonFrames)
                continue;
            auto releaseThenTap = releaseOnly;
            addToggle(releaseThenTap, frame, horizonFrames, releaseAt + gap, false);
            addToggle(releaseThenTap, frame, horizonFrames, releaseAt + gap + 1, false);
            candidates.push_back(std::move(releaseThenTap));
        }
    }
}

std::vector<std::set<SearchInput>> pruneLearnedFailuresV15(
    std::vector<std::set<SearchInput>> const& source,
    LearningMemoryV15& memory,
    int frame,
    float x,
    VehicleType mode,
    bool dashing
) {
    int threshold = dashing ? kDashBanThresholdV15 : kNormalBanThresholdV15;
    std::vector<std::set<SearchInput>> kept;
    kept.reserve(source.size());

    std::vector<std::pair<int, size_t>> rejected;
    rejected.reserve(source.size());

    for (size_t i = 0; i < source.size(); ++i) {
        int failures = failureCountV15(memory, source[i], frame, x, mode, dashing);
        if (failures < threshold) {
            kept.push_back(source[i]);
        } else {
            rejected.push_back({failures, i});
            ++memory.prunedCandidates;
        }
    }

    // Never let learning paint itself into a corner. If almost everything in a
    // region has failed, retry the least-bad old ideas occasionally.
    if (kept.size() < kMinCandidatesAfterPruneV15 && !rejected.empty()) {
        std::sort(rejected.begin(), rejected.end(), [](auto const& a, auto const& b) {
            if (a.first != b.first)
                return a.first < b.first;
            return a.second < b.second;
        });
        for (auto const& [_, index] : rejected) {
            kept.push_back(source[index]);
            if (kept.size() >= kMinCandidatesAfterPruneV15)
                break;
        }
    }

    return kept;
}

bool betterTrialLearnedV15(
    TrialResult const& trial,
    size_t toggleCount,
    int knownFailures,
    bool haveBest,
    TrialResult const& best,
    size_t bestToggleCount,
    int bestKnownFailures,
    VehicleType mode
) {
    if (!haveBest)
        return true;

    if (trial.complete != best.complete)
        return trial.complete;
    if (trial.dead != best.dead)
        return !trial.dead;

    // This is the learning rule the plain solver lacked. Among two dead ideas,
    // a repeatedly failed timing family loses before "died 8 px farther" is
    // considered.
    if (trial.dead && best.dead && knownFailures != bestKnownFailures)
        return knownFailures < bestKnownFailures;

    return betterTrial(
        trial,
        toggleCount,
        haveBest,
        best,
        bestToggleCount,
        mode
    );
}

void publishV15(
    PathfinderTelemetry telemetry,
    std::function<void(PathfinderTelemetry const&)> const& callback,
    int evaluatorThreads,
    int epoch,
    LearningMemoryV15 const& memory
) {
    telemetry.mode = "single-learning-v15";
    telemetry.workerCount = 1;
    telemetry.physicalThreadCount = std::max(1, evaluatorThreads);
    telemetry.guidedCount = 0;
    telemetry.explorerCount = 0;
    telemetry.frontierCount = 0;
    telemetry.archiveCount = 0;
    telemetry.progressLocked = false;
    telemetry.deadEndLevel = 0;
    telemetry.rollbackDistance = 0;

    std::ostringstream detail;
    detail << "epoch " << (epoch + 1) << "/" << kMaxStrategyEpochsV15
           << " | remembered failures " << memory.failures.size()
           << " | same-basin rounds " << memory.sameBasinRounds
           << " | evaluator threads " << std::max(1, evaluatorThreads);
    if (!telemetry.recoveryReason.empty())
        detail << " | " << telemetry.recoveryReason;
    telemetry.recoveryReason = detail.str();

    if (memory.sameBasinRounds >= 2) {
        telemetry.decision =
            "Learning from repeated deaths: abandoning familiar timing";
        telemetry.stallRescue = true;
    } else if (telemetry.decision.empty()) {
        telemetry.decision = "Single Pathfinder testing learned timing choices";
    }

    publishPathfinderTelemetryV8(telemetry);
    if (callback)
        callback(telemetry);
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
    bool hasTrustedEnd =
        std::isfinite(trustedEndX) && trustedEndX > solveStartX + 30.f;
    if (hasTrustedEnd) {
        lvl.length = trustedEndX;
        lvl.lengthSource = "trusted-gd-v15";
    }

    std::random_device rd;
    uint32_t baseSeed =
        rd() ^ static_cast<uint32_t>(std::hash<std::string>{}(lvlString));
    std::mt19937 rng(baseSeed);

    LearningMemoryV15 memory;
    memory.failures.reserve(12000);

    int trueBestFrame = lvl.currentFrame();
    int fail = 1;
    int numAway = 1000;
    int stagnantRounds = 0;
    int recoveryCount = 0;
    int hardestSearchLevel = 0;
    int recoveredExceptions = 0;
    int deadCandidatesRejected = 0;
    int randomFallbacks = 0;
    int structuredWins = 0;
    int debugSearchLevel = 0;
    int debugHorizon = 0;
    int debugCandidateCount = 0;
    int debugWorkers = 1;
    int debugVehicleType = static_cast<int>(lvl.latestState().vehicle.type);
    uint64_t totalTrials = 0;
    int epoch = 0;

    float furthestX = lvl.latestState().pos.x;
    float lastDeathX = 0.f;
    float debugClearance = 0.f;
    Timeline bestPlayable(lvl);

    uint64_t lastRealAdvanceTrial = 0;
    auto lastRealAdvanceTime = std::chrono::steady_clock::now();

    auto emitTelemetry = [&](int phase, std::string reason) {
        PathfinderTelemetry telemetry;
        telemetry.progress =
            progressFor(furthestX, solveStartX, lvl.length, reachedGoal(lvl));
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
        telemetry.workerCount = 1;
        telemetry.physicalThreadCount = debugWorkers;
        telemetry.phase = phase;
        telemetry.totalTrials = totalTrials;
        telemetry.recoveryReason = std::move(reason);
        publishV15(telemetry, callback, debugWorkers, epoch, memory);
    };

    auto recoverFromBest = [&](int retreatFrames) {
        bestPlayable.restore(lvl);
        int target = std::max(1, bestPlayable.frame() - retreatFrames);
        lvl.rollback(target);
        lvl.syncPresses();
        ++recoveryCount;
        rng.seed(
            baseSeed ^
            static_cast<uint32_t>((epoch + 1) * 104729u) ^
            static_cast<uint32_t>(recoveryCount * 7919u) ^
            static_cast<uint32_t>(lvl.currentFrame())
        );
    };

    auto resetStrategyEpoch = [&]() -> bool {
        if (epoch + 1 >= kMaxStrategyEpochsV15)
            return false;

        ++epoch;
        int retreat = std::min(
            std::max(0, bestPlayable.frame() - 1),
            720 * (epoch + 1)
        );
        bestPlayable.restore(lvl);
        if (retreat > 0)
            lvl.rollback(std::max(1, bestPlayable.frame() - retreat));
        lvl.syncPresses();

        fail = 1;
        numAway = std::min(9000, 1400 + epoch * 1200);
        stagnantRounds = 0;
        recoveryCount = std::max(recoveryCount, epoch * 2);
        memory.sameBasinRounds = 0;
        memory.lastDeathBucket = std::numeric_limits<int>::min();
        lastRealAdvanceTrial = totalTrials;
        lastRealAdvanceTime = std::chrono::steady_clock::now();

        rng.seed(
            baseSeed ^
            static_cast<uint32_t>((epoch + 1) * 2246822519u) ^
            static_cast<uint32_t>(memory.learnedDeaths)
        );

        emitTelemetry(
            2,
            "strategy reset: kept failure memory, backed up for a different approach"
        );
        return true;
    };

    emitTelemetry(0, "learning search started");

    while (!reachedGoal(lvl) && !stop.load()) {
        try {
            auto now = std::chrono::steady_clock::now();
            uint64_t noProgressTrials =
                totalTrials >= lastRealAdvanceTrial
                    ? totalTrials - lastRealAdvanceTrial
                    : 0;
            auto noProgressTime = now - lastRealAdvanceTime;

            bool saturated =
                (noProgressTrials >= kEpochStallTrialsV15 &&
                 noProgressTime >= kEpochSoftStallTimeV15) ||
                noProgressTime >= kEpochHardStallTimeV15;

            if (saturated) {
                if (resetStrategyEpoch())
                    continue;
                emitTelemetry(
                    2,
                    "search saturated after learned strategy resets; returning best route"
                );
                break;
            }

            if (lvl.latestState().dead) {
                recoverFromBest(std::min(3000, 300 + recoveryCount * 220));
                emitTelemetry(2, "recover-dead-state");
                continue;
            }

            int frame = lvl.currentFrame();
            float currentX = lvl.latestState().pos.x;
            VehicleType mode = lvl.latestState().vehicle.type;
            bool dashing = lvl.latestState().dashing;
            bool dual = lvl.latestState().dualActive;

            int searchLevel = std::min(
                12,
                recoveryCount + stagnantRounds / 3 + memory.sameBasinRounds / 2
            );
            hardestSearchLevel = std::max(hardestSearchLevel, searchLevel);

            int horizonFrames = 720 + searchLevel * 80;
            if (flightMode(mode))
                horizonFrames += 240;
            else if (mode == VehicleType::Robot)
                horizonFrames += 120;
            else if (mode == VehicleType::Spider)
                horizonFrames = std::min(horizonFrames, 1200);
            horizonFrames = std::min(horizonFrames, 1920);

            auto rawCandidates = structuredCandidates(
                frame,
                horizonFrames,
                mode,
                dual,
                lvl.press1,
                dashing
            );
            appendDashLearningCandidatesV15(
                rawCandidates,
                frame,
                horizonFrames,
                lvl.press1,
                dashing
            );

            auto candidates = pruneLearnedFailuresV15(
                rawCandidates,
                memory,
                frame,
                currentX,
                mode,
                dashing
            );

            debugVehicleType = static_cast<int>(mode);
            debugSearchLevel = searchLevel;
            debugHorizon = horizonFrames;
            debugCandidateCount = static_cast<int>(candidates.size());
            debugClearance = 0.f;

            emitTelemetry(
                dashing ? 2 : 0,
                dashing
                    ? "dash search: dense early releases + learned failure bans"
                    : "structured search with learned failure bans"
            );

            int workersUsed = 1;
            auto results = evaluateCandidatesParallel(
                lvl,
                candidates,
                horizonFrames,
                stop,
                workersUsed
            );
            debugWorkers = workersUsed;
            totalTrials += results.size();

            std::set<SearchInput> bestInputs;
            TrialResult bestTrial {
                frame, currentX, 0.f, 0.f, false, false
            };
            bool haveBest = false;
            size_t bestToggleCount = std::numeric_limits<size_t>::max();
            int bestKnownFailures = std::numeric_limits<int>::max();

            std::unordered_map<int, int> roundDeathBuckets;
            int roundDead = 0;

            auto consumeBatch = [&](auto const& batchCandidates, auto const& batchResults) {
                size_t count = std::min(batchCandidates.size(), batchResults.size());
                for (size_t i = 0; i < count; ++i) {
                    auto const& trial = batchResults[i];
                    auto const& inputs = batchCandidates[i];

                    uint64_t key =
                        candidateKeyV15(inputs, frame, currentX, mode, dashing);
                    int priorFailures = 0;
                    if (auto it = memory.failures.find(key); it != memory.failures.end())
                        priorFailures = it->second;

                    if (trial.dead) {
                        int& value = memory.failures[key];
                        value = std::min(20, value + 1);
                        ++memory.learnedDeaths;
                        ++roundDead;

                        int bucket = static_cast<int>(
                            std::floor(trial.deathX / kDeathBucketSizeV15)
                        );
                        ++roundDeathBuckets[bucket];
                    } else {
                        auto it = memory.failures.find(key);
                        if (it != memory.failures.end()) {
                            if (it->second <= 1)
                                memory.failures.erase(it);
                            else
                                --it->second;
                        }
                    }

                    if (trial.dead)
                        lastDeathX = trial.deathX;

                    if (!betterTrialLearnedV15(
                            trial,
                            inputs.size(),
                            priorFailures,
                            haveBest,
                            bestTrial,
                            bestToggleCount,
                            bestKnownFailures,
                            mode
                        )) {
                        if (trial.dead && haveBest && !bestTrial.dead)
                            ++deadCandidatesRejected;
                        continue;
                    }

                    bestTrial = trial;
                    bestToggleCount = inputs.size();
                    bestKnownFailures = priorFailures;
                    bestInputs = inputs;
                    haveBest = true;
                }
            };

            consumeBatch(candidates, results);

            float strongClearance = flightMode(mode)
                ? 5.5f
                : mode == VehicleType::Spider ? 3.0f
                : 2.5f;
            bool structuredStrong =
                haveBest &&
                !bestTrial.dead &&
                bestTrial.frame >= frame + horizonFrames - 2 &&
                (bestTrial.complete || bestTrial.minClearance >= strongClearance);

            if (structuredStrong) {
                ++structuredWins;
            } else if (!stop.load()) {
                ++randomFallbacks;
                int randomCount = std::min(
                    dashing ? 260 : 620,
                    (dashing ? 120 : 90) + searchLevel * (dashing ? 20 : 45)
                );

                std::vector<std::set<SearchInput>> randomCandidates;
                randomCandidates.reserve(static_cast<size_t>(randomCount));

                int nearWindow = std::min(
                    horizonFrames,
                    dashing ? 96 : 160 + searchLevel * 45
                );
                std::uniform_int_distribution<int> farFrame(0, horizonFrames - 1);
                std::uniform_int_distribution<int> nearFrame(
                    0,
                    std::max(0, nearWindow - 1)
                );

                int extraToggles = searchLevel * 2;
                int maxP1 = maxToggleBudget(mode) + extraToggles;
                int maxP2 = dual
                    ? maxToggleBudget(lvl.latestState2().vehicle.type) + extraToggles
                    : 0;
                std::uniform_int_distribution<int> p1Budget(0, maxP1);
                std::uniform_int_distribution<int> p2Budget(0, maxP2);

                for (int attempt = 0; attempt < randomCount; ++attempt) {
                    std::set<SearchInput> inputs;

                    if (dashing && lvl.press1) {
                        // Random dash exploration must also include genuinely
                        // early releases rather than mostly far-away toggles.
                        int earlyWindow = std::min(48, horizonFrames - 1);
                        if (earlyWindow > 0) {
                            std::uniform_int_distribution<int> earlyRelease(
                                1,
                                earlyWindow
                            );
                            addToggle(
                                inputs,
                                frame,
                                horizonFrames,
                                earlyRelease(rng),
                                false
                            );
                        }
                    }

                    int p1Candidates = p1Budget(rng);
                    int p2Candidates = p2Budget(rng);

                    auto candidateFrame = [&](int index) -> uint32_t {
                        bool useNear =
                            dashing ||
                            (searchLevel > 0 && ((attempt + index) % 5 != 0));
                        int offset = useNear ? nearFrame(rng) : farFrame(rng);
                        return static_cast<uint32_t>(frame + offset);
                    };

                    for (int i = 0; i < p1Candidates; ++i)
                        inputs.insert(inputKey(candidateFrame(i), false));
                    for (int i = 0; i < p2Candidates; ++i)
                        inputs.insert(
                            inputKey(candidateFrame(i + p1Candidates), true)
                        );

                    randomCandidates.push_back(std::move(inputs));
                }

                auto filteredRandom = pruneLearnedFailuresV15(
                    randomCandidates,
                    memory,
                    frame,
                    currentX,
                    mode,
                    dashing
                );

                debugCandidateCount = static_cast<int>(
                    candidates.size() + filteredRandom.size()
                );
                emitTelemetry(1, "random fallback avoiding learned failures");

                int randomWorkers = 1;
                auto randomResults = evaluateCandidatesParallel(
                    lvl,
                    filteredRandom,
                    horizonFrames,
                    stop,
                    randomWorkers
                );
                debugWorkers = std::max(debugWorkers, randomWorkers);
                totalTrials += randomResults.size();
                consumeBatch(filteredRandom, randomResults);
            }

            if (stop.load())
                break;

            int dominantBucket = std::numeric_limits<int>::min();
            int dominantCount = 0;
            for (auto const& [bucket, count] : roundDeathBuckets) {
                if (count > dominantCount) {
                    dominantBucket = bucket;
                    dominantCount = count;
                }
            }

            bool meaningfulBasin =
                dominantCount >= 3 &&
                roundDead > 0 &&
                dominantCount * 3 >= roundDead;

            if (meaningfulBasin) {
                if (dominantBucket == memory.lastDeathBucket)
                    ++memory.sameBasinRounds;
                else
                    memory.sameBasinRounds = 1;
                memory.lastDeathBucket = dominantBucket;
                lastDeathX =
                    (static_cast<float>(dominantBucket) + 0.5f) *
                    kDeathBucketSizeV15;
            } else {
                memory.sameBasinRounds =
                    std::max(0, memory.sameBasinRounds - 1);
            }

            debugClearance = haveBest ? bestTrial.minClearance : 0.f;
            emitTelemetry(
                1,
                memory.sameBasinRounds >= 2
                    ? "same death basin recognized; changing timing family"
                    : "candidate selected"
            );

            if (!haveBest || bestTrial.frame <= frame) {
                ++recoveryCount;
                int preferred = std::max(frame - fail, trueBestFrame - numAway);
                int target = std::clamp(
                    preferred,
                    1,
                    std::max(1, frame - 1)
                );
                lvl.rollback(target);
                lvl.syncPresses();

                fail += 5;
                if (fail > numAway + 1000) {
                    numAway = std::min(9000, numAway + 1000);
                    fail = 1;
                } else if (fail > 100) {
                    fail += 50;
                }

                emitTelemetry(2, "rollback-no-advance");
                continue;
            }

            // If the same wall has already proven this candidate family bad,
            // do not commit another dead prefix just because it dies slightly
            // farther. Back up and make the next round use different memory.
            if (bestTrial.dead && memory.sameBasinRounds >= 2) {
                int retreat = std::min(
                    3200,
                    360 + memory.sameBasinRounds * 240
                );
                recoverFromBest(retreat);
                emitTelemetry(
                    2,
                    dashing
                        ? "repeated dash death: released timing family rejected"
                        : "repeated death basin: candidate family rejected"
                );

                if (memory.sameBasinRounds >= 7) {
                    if (!resetStrategyEpoch()) {
                        emitTelemetry(
                            2,
                            "same death survived all strategy resets; returning best route"
                        );
                        break;
                    }
                }
                continue;
            }

            int applyUntil = frame;
            if (bestTrial.complete) {
                applyUntil = bestTrial.frame;
            } else if (!bestTrial.dead) {
                int advance = bestTrial.frame - frame;
                int safetyDivisor =
                    flightMode(mode) ||
                    mode == VehicleType::Robot ||
                    mode == VehicleType::Spider ||
                    dashing
                        ? 4
                        : 5;
                int safetyTail = std::max(
                    dashing ? 8 : 12,
                    advance / safetyDivisor
                );
                applyUntil = std::max(frame + 1, bestTrial.frame - safetyTail);
            } else {
                int deathBuffer = std::min(
                    340,
                    130 + searchLevel * 14 +
                    memory.sameBasinRounds * 20
                );
                int safeEnd = std::max(frame, bestTrial.frame - deathBuffer);
                int safeAdvance = safeEnd - frame;
                applyUntil = frame + safeAdvance / 2;
                ++deadCandidatesRejected;
            }

            if (applyUntil <= frame) {
                recoverFromBest(
                    std::min(3000, 300 + recoveryCount * 220)
                );
                emitTelemetry(2, "recover-zero-prefix");
                continue;
            }

            while (
                lvl.currentFrame() < applyUntil &&
                !lvl.latestState().dead &&
                !reachedGoal(lvl)
            ) {
                uint32_t current =
                    static_cast<uint32_t>(lvl.currentFrame());

                if (bestInputs.contains(inputKey(current, false)))
                    lvl.press1 = !lvl.press1;
                if (bestInputs.contains(inputKey(current, true)))
                    lvl.press2 = !lvl.press2;

                lvl.runFrame(lvl.press1, lvl.press2, 1.f / 240.f);
            }

            if (lvl.latestState().dead) {
                lastDeathX = lvl.latestState().pos.x;

                uint64_t appliedKey = candidateKeyV15(
                    bestInputs,
                    frame,
                    currentX,
                    mode,
                    dashing
                );
                int& appliedFailures = memory.failures[appliedKey];
                appliedFailures = std::min(20, appliedFailures + 2);
                memory.learnedDeaths += 2;

                recoverFromBest(
                    std::min(3200, 480 + recoveryCount * 240)
                );
                emitTelemetry(
                    2,
                    "applied candidate died; timing was strongly penalized"
                );
                continue;
            }

            if (lvl.currentFrame() > trueBestFrame) {
                trueBestFrame = lvl.currentFrame();
                fail = 0;
                numAway = 1000;
            }

            bool advancedX =
                lvl.latestState().pos.x > furthestX + kMeaningfulAdvanceV15;

            if (advancedX || reachedGoal(lvl)) {
                furthestX = std::max(
                    furthestX,
                    lvl.latestState().pos.x
                );
                bestPlayable.capture(lvl);
                stagnantRounds = 0;
                recoveryCount = std::max(0, recoveryCount - 2);
                memory.sameBasinRounds = 0;
                memory.lastDeathBucket = std::numeric_limits<int>::min();
                lastRealAdvanceTrial = totalTrials;
                lastRealAdvanceTime = std::chrono::steady_clock::now();
                emitTelemetry(
                    3,
                    reachedGoal(lvl)
                        ? "complete"
                        : "real X advance: learned route promoted"
                );
            } else if (lvl.latestState().direction < 0) {
                stagnantRounds = 0;
            } else {
                ++stagnantRounds;
            }

            if (
                stagnantRounds >= 8 &&
                bestPlayable.frame() > 2 &&
                lvl.latestState().direction >= 0
            ) {
                int retreat = std::min(
                    bestPlayable.frame() - 1,
                    360 * (1 + std::min(recoveryCount, 8))
                );
                recoverFromBest(retreat);
                stagnantRounds = 0;
                fail = 1;
                numAway = std::min(
                    9000,
                    1200 + recoveryCount * 550
                );
                emitTelemetry(
                    2,
                    "stagnation recovery: backing up without forgetting failures"
                );
            }
        } catch (std::exception const&) {
            ++recoveredExceptions;
            recoverFromBest(
                std::min(3600, 480 + recoveredExceptions * 240)
            );
            emitTelemetry(2, "recover-exception");
        } catch (...) {
            ++recoveredExceptions;
            recoverFromBest(
                std::min(3600, 480 + recoveredExceptions * 240)
            );
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
            output.inputs.push_back(
                gdr::Input(p1.frame, 1, false, p1.button)
            );
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

            if (
                p2.dualActive &&
                p2.frame > 1 &&
                p2.button != previousP2.button
            ) {
                output.inputs.push_back(
                    gdr::Input(p2.frame, 1, true, p2.button)
                );
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

    // A no-button partial route is still a usable route. Give the UI a harmless
    // release event so Stop does not report "no usable path" after valid coasting.
    if (
        result.inputs.empty() &&
        bestPlayable.frame() > 1 &&
        bestPlayable.x() > solveStartX + 1.f
    ) {
        output.inputs.push_back(gdr::Input(1, 1, false, false));
        result.inputs.push_back({1u, false, false, 1u});
    }

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
        << "solver=single-learning-v15"
        << " progress=" << result.progress
        << " frame=" << bestPlayable.frame()
        << " routeX=" << bestPlayable.x()
        << " furthestX=" << furthestX
        << " endX=" << lvl.length
        << " epoch=" << (epoch + 1)
        << " recoveryCount=" << recoveryCount
        << " hardestSearch=" << hardestSearchLevel
        << " totalTrials=" << totalTrials
        << " evaluatorThreads=" << debugWorkers
        << " rememberedFailures=" << memory.failures.size()
        << " learnedDeaths=" << memory.learnedDeaths
        << " prunedCandidates=" << memory.prunedCandidates
        << " sameBasinRounds=" << memory.sameBasinRounds
        << " structuredWins=" << structuredWins
        << " randomFallbacks=" << randomFallbacks
        << " deadCandidatesRejected=" << deadCandidatesRejected
        << " lastDeathX=" << lastDeathX
        << " bestClearance=" << debugClearance
        << " recoveredExceptions=" << recoveredExceptions
        << " moveTriggers=" << lvl.supportedMoveTriggers
        << " unsupportedMoves=" << lvl.unsupportedMoveTriggers
        << " movingObjects=" << lvl.movingObjectIDs.size()
        << " inputs=" << result.inputs.size()
        << " complete=" << (result.complete ? 1 : 0)
        << " stopped=" << (stop.load() ? 1 : 0)
        << " helpers=0";

    result.diagnostics = diagnostics.str();
    return result;
}
