// beta.246 stability guard
//
// Keep the large beam-search implementation in pathfinder.cpp intact, but wrap it
// so a local search dead-end cannot instantly become a 3% "finished" autoplay.
// This translation unit deliberately includes the implementation under a private
// symbol, then exposes the normal pathfind() entry point with retry, telemetry
// quieting, incomplete-result suppression, and simulator-verified input cleanup.

#define pathfind pathfind_beta246_impl
#include "pathfinder.cpp"
#undef pathfind

namespace {

bool betterPublicResult(PathfinderResult const& a, PathfinderResult const& b) {
    if (a.complete != b.complete)
        return a.complete;
    if (std::abs(a.progress - b.progress) > 0.01)
        return a.progress > b.progress;
    return a.inputs.size() < b.inputs.size();
}

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
    int maxFrame = std::max<int>(
        static_cast<int>(lastInputFrame) + 4800,
        240 * 180
    );

    while (lvl.currentFrame() < maxFrame &&
           !lvl.latestState().dead &&
           !reachedGoal(lvl)) {
        uint32_t nextStateFrame = static_cast<uint32_t>(lvl.currentFrame() + 1);
        while (cursor < inputs.size() && inputs[cursor].frame <= nextStateFrame) {
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

    // First merge tiny release/re-press gaps. This attacks the beta.246
    // tap-spam pattern by turning OFF/ON/OFF/ON bursts into a held input when
    // the simulator confirms the route still completes.
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

    // If merging holds was too aggressive, try only deleting one-frame pulses.
    auto candidate = collapseTransitionPairs(original, true, false, 1u);
    if (candidate.size() < original.size() &&
        validatesCompleteRoute(lvlString, trustedEndX, candidate)) {
        result.inputs = std::move(candidate);
        rebuildMacro(result);
    }
}

} // namespace

PathfinderResult pathfind(
    std::string const& lvlString,
    std::atomic_bool& stop,
    std::function<void(PathfinderTelemetry const&)> callback,
    float trustedEndX
) {
    constexpr int kMaxAttempts = 8;

    PathfinderResult best;
    bool haveBest = false;
    int attempts = 0;
    int stagnantAttempts = 0;
    double bestProgress = -1.0;

    // The UI used to print every alternate-branch/retreat transition. Preserve
    // progress telemetry, but sanitize those internal recovery reasons so they
    // do not flood the log while the solver is thrashing around a hard section.
    auto quietCallback = [&](PathfinderTelemetry const& telemetry) {
        if (!callback)
            return;
        auto quiet = telemetry;
        if (quiet.recoveryReason.find(':') != std::string::npos)
            quiet.recoveryReason = "recovering";
        callback(quiet);
    };

    while (!stop.load() && attempts < kMaxAttempts) {
        auto candidate = pathfind_beta246_impl(
            lvlString,
            stop,
            quietCallback,
            trustedEndX
        );
        ++attempts;

        if (candidate.complete)
            simplifyCompletedInputs(lvlString, trustedEndX, candidate);

        double candidateProgress = candidate.progress;
        if (!haveBest || betterPublicResult(candidate, best)) {
            best = std::move(candidate);
            haveBest = true;
        }

        if (best.complete)
            break;
        if (stop.load())
            break;

        // beta.246's internal repeated-state guard can return after only a few
        // local failures. Retry the full search with a fresh randomized seed
        // instead of letting that tiny local dead-end finalize the whole run.
        if (candidateProgress > bestProgress + 0.10) {
            bestProgress = candidateProgress;
            stagnantAttempts = 0;
        } else {
            ++stagnantAttempts;
        }

        if (attempts >= 5 && stagnantAttempts >= 4)
            break;
    }

    if (!haveBest)
        return {};

    size_t retainedInputs = best.inputs.size();

    // Never arm an unfinished route. The old behavior could "finish" at 3%,
    // autoplay a partial macro, then leave the player in a held-input state.
    if (!best.complete) {
        best.inputs.clear();
        best.macro.clear();
    }

    std::ostringstream wrapperDiagnostics;
    if (!best.diagnostics.empty())
        wrapperDiagnostics << best.diagnostics << ' ';
    wrapperDiagnostics << "guardAttempts=" << attempts
                       << " incompleteInputsSuppressed=" << (!best.complete && retainedInputs > 0 ? 1 : 0)
                       << " retainedInputs=" << retainedInputs
                       << " finalInputs=" << best.inputs.size();
    best.diagnostics = wrapperDiagnostics.str();
    return best;
}
