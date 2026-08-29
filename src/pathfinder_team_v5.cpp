#include "pathfinder_team_v4.cpp"

namespace {

struct ReplayValidationV5 {
    bool complete = false;
    bool dead = false;
    float furthestX = 0.f;
    float endX = 0.f;
    int frame = 0;
};

ReplayValidationV5 validateExportedReplayV5(
    std::string const& lvlString,
    std::vector<PathfinderInput> const& inputs,
    float trustedEndX
) {
    Level2 check(lvlString);
    pruneNoTouchPhysics(check, lvlString);

    float startX = check.latestState().pos.x;
    bool hasTrustedEnd = std::isfinite(trustedEndX) &&
                         trustedEndX > startX + 30.f;
    if (hasTrustedEnd) {
        check.length = trustedEndX;
        check.lengthSource = "trusted-gd-validation";
    }

    std::vector<PathfinderInput> ordered = inputs;
    std::sort(
        ordered.begin(),
        ordered.end(),
        [](PathfinderInput const& a, PathfinderInput const& b) {
            if (a.frame != b.frame)
                return a.frame < b.frame;
            if (a.player2 != b.player2)
                return a.player2 < b.player2;
            return a.button < b.button;
        }
    );

    uint32_t lastInputFrame = ordered.empty() ? 0u : ordered.back().frame;
    double distance = std::max(0.0, static_cast<double>(check.length - startX));

    // Validation only runs after a candidate has claimed completion. Give the
    // exported replay a generous amount of empty travel after its final input,
    // while still preventing a malformed route from simulating forever.
    int distanceBudget = static_cast<int>(std::ceil(distance * 2.4)) + 4800;
    int inputBudget = static_cast<int>(
        std::min<uint64_t>(
            static_cast<uint64_t>(lastInputFrame) + 24000ull,
            180000ull
        )
    );
    int maxFrame = std::clamp(
        std::max(distanceBudget, inputBudget),
        4800,
        180000
    );

    bool heldP1 = check.press1;
    bool heldP2 = check.press2;
    size_t cursor = 0;
    float furthestX = check.latestState().pos.x;

    while (check.currentFrame() < maxFrame &&
           !check.latestState().dead &&
           !reachedGoal(check)) {
        uint32_t current = static_cast<uint32_t>(check.currentFrame());
        while (cursor < ordered.size() && ordered[cursor].frame <= current) {
            auto const& input = ordered[cursor++];
            if (input.button != 1)
                continue;
            if (input.player2)
                heldP2 = input.down;
            else
                heldP1 = input.down;
        }

        check.press1 = heldP1;
        check.press2 = heldP2;
        check.runFrame(heldP1, heldP2, 1.f / 240.f);
        furthestX = std::max(furthestX, check.latestState().pos.x);
    }

    bool simulatorComplete = reachedGoal(check) && !check.latestState().dead;

    // A simulator End Trigger is not enough to claim 100% when Geometry Dash gave
    // us a trusted real endpoint. The replay must physically reach that endpoint.
    bool reachedTrustedEnd = !hasTrustedEnd ||
        furthestX >= trustedEndX - 30.f;

    ReplayValidationV5 validation;
    validation.complete = simulatorComplete && reachedTrustedEnd;
    validation.dead = check.latestState().dead;
    validation.furthestX = furthestX;
    validation.endX = check.length;
    validation.frame = check.currentFrame();
    return validation;
}

void addValidatedZeroInputNoopV5(PathfinderResult& result) {
    // A true zero-input completion is a legitimate route, but the current UI uses
    // an empty input vector as its "no path" sentinel. Represent the solved idle
    // route with a harmless release event so it cannot be mistaken for failure.
    result.inputs.push_back({1u, false, false, 1u});

    Replay2 output;
    output.inputs.push_back(gdr::Input(1, 1, false, false));
    result.macro = output.exportData().unwrapOr({});
}

} // namespace

PathfinderResult pathfind_v5(
    std::string const& lvlString,
    std::atomic_bool& stop,
    std::function<void(PathfinderTelemetry const&)> callback,
    float trustedEndX
) {
    // Do not expose an unvalidated 100% to the UI. v4 can believe a worker reached
    // completion before we know that the final exported replay reproduces it.
    std::function<void(PathfinderTelemetry const&)> guardedCallback;
    if (callback) {
        guardedCallback = [callback](PathfinderTelemetry const& incoming) {
            PathfinderTelemetry telemetry = incoming;
            if (telemetry.progress >= 100.0) {
                telemetry.progress = 99.99;
                telemetry.phase = 1;
                telemetry.recoveryReason = "awaiting-replay-validation";
            }
            callback(telemetry);
        };
    }

    PathfinderResult result = pathfind_v4(
        lvlString,
        stop,
        guardedCallback,
        trustedEndX
    );

    if (!result.complete) {
        result.diagnostics += " replayValidation=not-needed";
        return result;
    }

    bool claimedZeroInput = result.inputs.empty();
    ReplayValidationV5 validation = validateExportedReplayV5(
        lvlString,
        result.inputs,
        trustedEndX
    );

    if (!validation.complete) {
        result.complete = false;

        Level2 progressLevel(lvlString);
        float startX = progressLevel.latestState().pos.x;
        float endX = (std::isfinite(trustedEndX) && trustedEndX > startX + 30.f)
            ? trustedEndX
            : progressLevel.length;
        result.progress = progressFor(
            validation.furthestX,
            startX,
            endX,
            false
        );

        result.diagnostics +=
            " replayValidation=FAILED"
            " claimedComplete=1";
        result.diagnostics += " validatedFrame=" + std::to_string(validation.frame);
        result.diagnostics += " validatedX=" + std::to_string(validation.furthestX);
        result.diagnostics += " validationEndX=" + std::to_string(validation.endX);
        result.diagnostics += " validationDead=" + std::to_string(validation.dead ? 1 : 0);
        result.diagnostics += " zeroInputClaim=" + std::to_string(claimedZeroInput ? 1 : 0);
        return result;
    }

    result.complete = true;
    result.progress = 100.0;

    if (claimedZeroInput)
        addValidatedZeroInputNoopV5(result);

    result.diagnostics += " replayValidation=PASS";
    result.diagnostics += " validatedFrame=" + std::to_string(validation.frame);
    result.diagnostics += " validatedX=" + std::to_string(validation.furthestX);
    result.diagnostics += " validationEndX=" + std::to_string(validation.endX);
    result.diagnostics += " zeroInputClaim=" + std::to_string(claimedZeroInput ? 1 : 0);
    return result;
}
