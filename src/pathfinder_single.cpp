// Clean single-controller Pathfinder.
//
// The V7-V13 helper/team experiments are intentionally not part of this
// controller. There is exactly one search brain. It reuses the mature plain
// Pathfinder implementation in pathfinder.cpp (all game modes, dynamic geometry,
// structured candidates, partial-route preservation), while this thin wrapper
// adds a hard no-progress watchdog and a few clean fresh-search retries.
//
// Parallel work inside pathfinder.cpp is only candidate evaluation. Those
// threads do not own routes, checkpoints, rollback state, or independent AI
// decisions.
#define pathfind pathfind_plain_base_v14
#include "pathfinder.cpp"
#undef pathfind

#include "solver_dashboard.hpp"

#include <chrono>
#include <thread>

namespace {

constexpr int kMaxFreshAttemptsV14 = 4;
constexpr uint64_t kNoProgressTrialBudgetV14 = 90000;
constexpr auto kNoProgressTimeBudgetV14 = std::chrono::seconds(14);
constexpr float kMeaningfulXAdvanceV14 = 12.f;
constexpr double kMeaningfulPercentAdvanceV14 = 0.08;

bool betterPartialV14(PathfinderResult const& candidate, PathfinderResult const& current) {
    if (candidate.complete != current.complete)
        return candidate.complete;
    if (candidate.progress > current.progress + 0.001)
        return true;
    if (current.progress > candidate.progress + 0.001)
        return false;
    if (candidate.inputs.empty() != current.inputs.empty())
        return !candidate.inputs.empty();
    return candidate.inputs.size() < current.inputs.size();
}

std::string attemptStatusV14(
    int attempt,
    std::string const& baseReason,
    int evaluatorThreads,
    double bestProgress
) {
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(2);
    out << "attempt " << attempt << "/" << kMaxFreshAttemptsV14
        << " | " << (baseReason.empty() ? "searching" : baseReason)
        << " | evaluator threads " << evaluatorThreads
        << " | best " << bestProgress << "%";
    return out.str();
}

void publishAndForwardV14(
    PathfinderTelemetry const& telemetry,
    std::function<void(PathfinderTelemetry const&)> const& callback
) {
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
    PathfinderResult bestResult;
    bool haveResult = false;
    double globalBestProgress = 0.0;
    float globalBestX = 0.f;
    int attemptsUsed = 0;
    int stalledAttempts = 0;

    for (int attempt = 0;
         attempt < kMaxFreshAttemptsV14 && !stop.load();
         ++attempt) {
        ++attemptsUsed;
        std::atomic_bool attemptStop {false};
        std::atomic_bool monitorDone {false};
        bool stalled = false;

        uint64_t lastAdvanceTrial = 0;
        float attemptBestX = -std::numeric_limits<float>::infinity();
        double attemptBestProgress = 0.0;
        auto lastAdvanceTime = std::chrono::steady_clock::now();

        // Keep the existing Stop button responsive even while the base solver
        // is in a large parallel candidate batch.
        std::thread stopWatcher([&] {
            while (!monitorDone.load(std::memory_order_relaxed) &&
                   !attemptStop.load(std::memory_order_relaxed)) {
                if (stop.load(std::memory_order_relaxed)) {
                    attemptStop.store(true, std::memory_order_relaxed);
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(8));
            }
        });

        auto bridge = [&](PathfinderTelemetry const& incoming) {
            if (stop.load(std::memory_order_relaxed)) {
                attemptStop.store(true, std::memory_order_relaxed);
                return;
            }

            bool meaningfulAdvance =
                incoming.furthestX > attemptBestX + kMeaningfulXAdvanceV14 ||
                incoming.progress > attemptBestProgress + kMeaningfulPercentAdvanceV14;

            if (meaningfulAdvance || !std::isfinite(attemptBestX)) {
                attemptBestX = std::max(attemptBestX, incoming.furthestX);
                attemptBestProgress = std::max(attemptBestProgress, incoming.progress);
                lastAdvanceTrial = incoming.totalTrials;
                lastAdvanceTime = std::chrono::steady_clock::now();
            } else {
                attemptBestX = std::max(attemptBestX, incoming.furthestX);
                attemptBestProgress = std::max(attemptBestProgress, incoming.progress);
            }

            globalBestX = std::max(globalBestX, incoming.furthestX);
            globalBestProgress = std::max(globalBestProgress, incoming.progress);

            uint64_t stagnantTrials = incoming.totalTrials >= lastAdvanceTrial
                ? incoming.totalTrials - lastAdvanceTrial
                : 0;
            auto stagnantFor = std::chrono::steady_clock::now() - lastAdvanceTime;

            // A real level-X advance resets this clock. If neither broader
            // searching nor rollback can move X for a long time, end only this
            // attempt and start one clean search with a fresh RNG seed.
            if (stagnantTrials >= kNoProgressTrialBudgetV14 ||
                stagnantFor >= kNoProgressTimeBudgetV14) {
                stalled = true;
                attemptStop.store(true, std::memory_order_relaxed);
            }

            PathfinderTelemetry telemetry = incoming;
            telemetry.mode = "single-pathfinder-v14";
            telemetry.workerCount = 1; // one decision-making solver
            telemetry.physicalThreadCount = std::max(1, incoming.workerCount);
            telemetry.guidedCount = 0;
            telemetry.explorerCount = 0;
            telemetry.frontierCount = 0;
            telemetry.archiveCount = 0;
            telemetry.progressLocked = false;
            telemetry.stallRescue = incoming.phase == 2;
            telemetry.rollbackDistance = 0;
            telemetry.deadEndLevel = 0;
            telemetry.decision = stalled
                ? "No real X progress: ending this search attempt cleanly"
                : "Single Pathfinder choosing one route";
            telemetry.recoveryReason = attemptStatusV14(
                attempt + 1,
                incoming.recoveryReason,
                std::max(1, incoming.workerCount),
                globalBestProgress
            );
            publishAndForwardV14(telemetry, callback);
        };

        PathfinderResult result = pathfind_plain_base_v14(
            lvlString,
            attemptStop,
            bridge,
            trustedEndX
        );

        monitorDone.store(true, std::memory_order_relaxed);
        attemptStop.store(true, std::memory_order_relaxed);
        if (stopWatcher.joinable())
            stopWatcher.join();

        if (!haveResult || betterPartialV14(result, bestResult)) {
            bestResult = std::move(result);
            haveResult = true;
        }

        if (haveResult && bestResult.complete)
            break;
        if (stop.load(std::memory_order_relaxed))
            break;

        if (!stalled)
            break;

        ++stalledAttempts;

        if (attempt + 1 < kMaxFreshAttemptsV14) {
            PathfinderTelemetry telemetry;
            telemetry.progress = globalBestProgress;
            telemetry.currentX = globalBestX;
            telemetry.furthestX = globalBestX;
            telemetry.checkpointX = globalBestX;
            telemetry.workerCount = 1;
            telemetry.physicalThreadCount = 1;
            telemetry.phase = 2;
            telemetry.stallRescue = true;
            telemetry.totalTrials = 0;
            telemetry.mode = "single-pathfinder-v14";
            telemetry.decision = "Restarting one clean Pathfinder search";
            std::ostringstream reason;
            reason << "attempt " << (attempt + 1)
                   << " saturated without real X progress; keeping the best partial route and retrying with a fresh seed";
            telemetry.recoveryReason = reason.str();
            publishAndForwardV14(telemetry, callback);
        }
    }

    if (!haveResult)
        return PathfinderResult {};

    bestResult.diagnostics +=
        " controller=single-pathfinder-v14" +
        std::string(" attempts=") + std::to_string(attemptsUsed) +
        " stalledAttempts=" + std::to_string(stalledAttempts) +
        " helpers=0";

    return bestResult;
}
