// Universal Pathfinder V17: one frame-exact receding-horizon state-space solver
// for Cube, Ship, Ball, UFO, Wave, Robot, Spider, Swing, and dual mode.
//
// The old solver is included only for its Level2 simulation primitives and
// replay helpers. V17 does not call the old structured/random pathfinder.
#define pathfind pathfind_plain_unused_v17
#include "pathfinder.cpp"
#undef pathfind

#include "real_geometry.hpp"
#include "solver_dashboard.hpp"

#include <chrono>
#include <memory>
#include <unordered_map>

namespace {

struct MpcConfigV17 {
    int lookahead = 128;
    int beamWidth = 256;
    int commitFrames = 8;
    double clearanceWeight = 12.0;
    double togglePenalty = 0.12;
};

struct MpcTrialV17 {
    std::set<SearchInput> inputs;
    TrialResult result;
    float p1Y = 0.f;
    float p1V = 0.f;
    float p2Y = 0.f;
    float p2V = 0.f;
    float direction = 1.f;
    VehicleType mode1 = VehicleType::Cube;
    VehicleType mode2 = VehicleType::Cube;
    bool held1 = false;
    bool held2 = false;
    bool upside1 = false;
    bool upside2 = false;
    bool dash1 = false;
    bool dash2 = false;
    bool dual = false;
    int speed1 = 0;
    int speed2 = 0;
};

struct MpcPlanV17 {
    std::set<SearchInput> inputs;
    int commitFrames = 0;
    int lookaheadFrames = 0;
    int beamWidth = 0;
    int evaluatorThreads = 1;
    int produced = 0;
    int unique = 0;
    int dead = 0;
    uint64_t trials = 0;
    float minClearance = 0.f;
    float deathX = 0.f;
    bool found = false;
    bool complete = false;
    bool usedRealGeometry = false;
};

static char const* modeNameV17(VehicleType mode) {
    switch (mode) {
        case VehicleType::Cube: return "Cube";
        case VehicleType::Ship: return "Ship";
        case VehicleType::Ball: return "Ball";
        case VehicleType::Ufo: return "UFO";
        case VehicleType::Wave: return "Wave";
        case VehicleType::Robot: return "Robot";
        case VehicleType::Spider: return "Spider";
        case VehicleType::Swing: return "Swing";
    }
    return "Unknown";
}

static MpcConfigV17 configForV17(VehicleType mode, int searchLevel, bool dual) {
    MpcConfigV17 config;
    switch (mode) {
        case VehicleType::Cube:
            config = {160, 240, 10, 8.0, 0.16};
            break;
        case VehicleType::Ship:
            config = {144, 320, 6, 18.0, 0.05};
            break;
        case VehicleType::Ball:
            config = {160, 256, 8, 10.0, 0.10};
            break;
        case VehicleType::Ufo:
            config = {152, 288, 6, 14.0, 0.08};
            break;
        case VehicleType::Wave:
            config = {168, 384, 6, 22.0, 0.035};
            break;
        case VehicleType::Robot:
            config = {176, 288, 8, 10.0, 0.12};
            break;
        case VehicleType::Spider:
            config = {160, 256, 6, 12.0, 0.07};
            break;
        case VehicleType::Swing:
            config = {160, 336, 6, 18.0, 0.05};
            break;
    }

    config.lookahead = std::min(240, config.lookahead + searchLevel * 8);
    config.beamWidth = std::min(512, config.beamWidth + searchLevel * 24);

    // Four branches per tick in dual mode instead of two. Keep the search large,
    // but cap it enough that one plan cannot allocate the entire phone.
    if (dual) {
        config.lookahead = std::min(config.lookahead, 144);
        config.beamWidth = std::min(config.beamWidth, 288);
        config.commitFrames = std::min(config.commitFrames, 6);
    }
    return config;
}

static bool flightLikeV17(VehicleType mode) {
    return mode == VehicleType::Ship ||
           mode == VehicleType::Wave ||
           mode == VehicleType::Swing;
}

static float realGeometryClearanceV17(
    std::shared_ptr<PathfinderRealGeometry const> const& geometry,
    Player const& player
) {
    if (!geometry)
        return std::numeric_limits<float>::infinity();
    Entity hitbox = player.unrotatedHitbox();
    return pathfinderRealGeometryClearance(
        *geometry,
        hitbox.getLeft(),
        hitbox.getBottom(),
        hitbox.getRight(),
        hitbox.getTop()
    );
}

static MpcTrialV17 evaluateMpcCandidateV17(
    Level2 const& base,
    std::set<SearchInput> const& inputs,
    int depthFrames,
    std::shared_ptr<PathfinderRealGeometry const> const& geometry
) {
    Level2 worker = base;
    worker.fixStatePointers();

    int startFrame = worker.currentFrame();
    int endFrame = startFrame + depthFrames;
    float minClearance = std::numeric_limits<float>::infinity();
    float deathX = 0.f;

    while (
        worker.currentFrame() < endFrame &&
        !worker.latestState().dead &&
        !reachedGoal(worker)
    ) {
        uint32_t current = static_cast<uint32_t>(worker.currentFrame());
        if (inputs.contains(inputKey(current, false)))
            worker.press1 = !worker.press1;
        if (inputs.contains(inputKey(current, true)))
            worker.press2 = !worker.press2;

        worker.runFrame(worker.press1, worker.press2, 1.f / 240.f);
        auto const& p1 = worker.latestState();
        if (p1.dead) {
            deathX = p1.pos.x;
            break;
        }

        // Expensive on purpose: sample clearance on every single 240-TPS tick.
        float frameClearance = hazardClearance(worker, p1);

        // The real Show-Hitbox-style geometry is safest as a second edge sensor.
        // For grounded modes a solid overlap can be a legal floor contact, so
        // real geometry influences scoring most strongly for flight-like modes;
        // gd-sim remains the authoritative death/contact simulation for all modes.
        if (geometry && flightLikeV17(p1.vehicle.type)) {
            float real = realGeometryClearanceV17(geometry, p1);
            frameClearance = std::min(frameClearance, real);
        }

        if (p1.dualActive) {
            auto const& p2 = worker.latestState2();
            if (p2.dead) {
                deathX = std::max(p1.pos.x, p2.pos.x);
                break;
            }
            frameClearance = std::min(frameClearance, hazardClearance(worker, p2));
            if (geometry && flightLikeV17(p2.vehicle.type)) {
                float real2 = realGeometryClearanceV17(geometry, p2);
                frameClearance = std::min(frameClearance, real2);
            }
        }

        minClearance = std::min(minClearance, frameClearance);

        float y = p1.pos.y;
        if (y > std::max(1500.f, worker.highestY + 700.f) || y < -700.f) {
            deathX = p1.pos.x;
            break;
        }
    }

    auto const& p1 = worker.latestState();
    bool outOfBounds =
        p1.pos.y > std::max(1500.f, worker.highestY + 700.f) ||
        p1.pos.y < -700.f;
    bool p2Dead = p1.dualActive && worker.latestState2().dead;
    bool dead = p1.dead || p2Dead || outOfBounds;

    if (!std::isfinite(minClearance))
        minClearance = 10000.f;

    MpcTrialV17 out;
    out.inputs = inputs;
    out.result = {
        worker.currentFrame(),
        p1.pos.x,
        dead ? (deathX != 0.f ? deathX : p1.pos.x) : 0.f,
        minClearance,
        dead,
        !dead && reachedGoal(worker)
    };
    out.p1Y = p1.pos.y;
    out.p1V = static_cast<float>(p1.velocity);
    out.direction = static_cast<float>(p1.direction);
    out.mode1 = p1.vehicle.type;
    out.held1 = worker.press1;
    out.upside1 = p1.upsideDown;
    out.dash1 = p1.dashing;
    out.dual = p1.dualActive;
    out.speed1 = p1.speed;

    if (out.dual) {
        auto const& p2 = worker.latestState2();
        out.p2Y = p2.pos.y;
        out.p2V = static_cast<float>(p2.velocity);
        out.mode2 = p2.vehicle.type;
        out.held2 = worker.press2;
        out.upside2 = p2.upsideDown;
        out.dash2 = p2.dashing;
        out.speed2 = p2.speed;
    }
    return out;
}

static int mpcWorkerCountV17(size_t count) {
    if (count <= 1)
        return 1;
    constexpr size_t kMaxWorkers = 24;
    return static_cast<int>(std::min(count, kMaxWorkers));
}

static std::vector<MpcTrialV17> evaluateBatchV17(
    Level2 const& base,
    std::vector<std::set<SearchInput>> const& candidates,
    int depthFrames,
    std::shared_ptr<PathfinderRealGeometry const> const& geometry,
    std::atomic_bool& stop,
    int& workersUsed
) {
    std::vector<MpcTrialV17> results(candidates.size());
    if (candidates.empty()) {
        workersUsed = 1;
        return results;
    }

    workersUsed = mpcWorkerCountV17(candidates.size());
    std::atomic<size_t> next {0};
    std::vector<std::thread> threads;
    threads.reserve(static_cast<size_t>(workersUsed));

    for (int workerIndex = 0; workerIndex < workersUsed; ++workerIndex) {
        threads.emplace_back([&] {
            while (!stop.load()) {
                size_t index = next.fetch_add(1, std::memory_order_relaxed);
                if (index >= candidates.size())
                    break;
                results[index] = evaluateMpcCandidateV17(
                    base,
                    candidates[index],
                    depthFrames,
                    geometry
                );
            }
        });
    }

    for (auto& thread : threads)
        thread.join();
    return results;
}

static uint64_t mixStateV17(uint64_t h, uint64_t v) {
    h ^= v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
    return h * 1099511628211ull;
}

static uint64_t stateKeyV17(MpcTrialV17 const& trial) {
    int qX = static_cast<int>(std::llround(trial.result.x * 0.5f));
    int qY1 = static_cast<int>(std::llround(trial.p1Y * 2.f));
    int qV1 = static_cast<int>(std::llround(trial.p1V * 4.f));
    int qY2 = static_cast<int>(std::llround(trial.p2Y * 2.f));
    int qV2 = static_cast<int>(std::llround(trial.p2V * 4.f));

    uint64_t h = 1469598103934665603ull;
    h = mixStateV17(h, static_cast<uint32_t>(qX));
    h = mixStateV17(h, static_cast<uint32_t>(qY1));
    h = mixStateV17(h, static_cast<uint32_t>(qV1));
    h = mixStateV17(h, static_cast<uint32_t>(static_cast<int>(trial.mode1) + 1));
    h = mixStateV17(h, static_cast<uint32_t>(trial.speed1 + 8));
    h = mixStateV17(h, trial.held1 ? 1u : 0u);
    h = mixStateV17(h, trial.upside1 ? 2u : 0u);
    h = mixStateV17(h, trial.dash1 ? 4u : 0u);
    h = mixStateV17(h, trial.direction < 0.f ? 8u : 0u);
    h = mixStateV17(h, trial.dual ? 16u : 0u);

    if (trial.dual) {
        h = mixStateV17(h, static_cast<uint32_t>(qY2));
        h = mixStateV17(h, static_cast<uint32_t>(qV2));
        h = mixStateV17(h, static_cast<uint32_t>(static_cast<int>(trial.mode2) + 1));
        h = mixStateV17(h, static_cast<uint32_t>(trial.speed2 + 8));
        h = mixStateV17(h, trial.held2 ? 32u : 0u);
        h = mixStateV17(h, trial.upside2 ? 64u : 0u);
        h = mixStateV17(h, trial.dash2 ? 128u : 0u);
    }
    return h;
}

static double scoreV17(MpcTrialV17 const& trial, MpcConfigV17 const& config) {
    if (trial.result.complete)
        return 1e15;
    if (trial.result.dead)
        return -1e12 + static_cast<double>(trial.result.frame);

    double clearance = std::clamp(
        static_cast<double>(trial.result.minClearance),
        0.0,
        80.0
    );
    double directionalX = trial.direction < 0.f
        ? -static_cast<double>(trial.result.x)
        : static_cast<double>(trial.result.x);

    // Progress, survival clearance and state simplicity. We intentionally keep
    // toggle penalty small for edge-trigger modes (UFO/Ball/Spider) via config.
    return directionalX * 5.0 +
           clearance * config.clearanceWeight +
           static_cast<double>(trial.result.frame) * 0.04 -
           static_cast<double>(trial.inputs.size()) * config.togglePenalty;
}

static bool betterV17(
    MpcTrialV17 const& a,
    MpcTrialV17 const& b,
    MpcConfigV17 const& config
) {
    double sa = scoreV17(a, config);
    double sb = scoreV17(b, config);
    if (std::abs(sa - sb) > 1e-8)
        return sa > sb;
    if (a.result.minClearance != b.result.minClearance)
        return a.result.minClearance > b.result.minClearance;
    return a.inputs.size() < b.inputs.size();
}

static MpcPlanV17 planMpcV17(
    Level2 const& base,
    int searchLevel,
    std::atomic_bool& stop
) {
    MpcPlanV17 plan;
    VehicleType startMode = base.latestState().vehicle.type;
    bool startDual = base.latestState().dualActive;
    MpcConfigV17 config = configForV17(startMode, searchLevel, startDual);

    // Static Show-Hitbox snapshot is useful as an extra edge sensor. When the
    // simulator knows objects are moving, do not score against a stale snapshot.
    auto geometry = base.movingObjectIDs.empty()
        ? getPathfinderRealGeometry()
        : std::shared_ptr<PathfinderRealGeometry const> {};
    plan.usedRealGeometry = static_cast<bool>(geometry);
    plan.lookaheadFrames = config.lookahead;
    plan.beamWidth = config.beamWidth;

    int frame = base.currentFrame();
    std::vector<MpcTrialV17> beam;
    beam.reserve(static_cast<size_t>(config.beamWidth));

    MpcTrialV17 root;
    root.result = {
        frame,
        base.latestState().pos.x,
        0.f,
        hazardClearance(base, base.latestState()),
        false,
        reachedGoal(base)
    };
    root.p1Y = base.latestState().pos.y;
    root.p1V = static_cast<float>(base.latestState().velocity);
    root.direction = static_cast<float>(base.latestState().direction);
    root.mode1 = startMode;
    root.held1 = base.press1;
    root.upside1 = base.latestState().upsideDown;
    root.dash1 = base.latestState().dashing;
    root.dual = startDual;
    root.speed1 = base.latestState().speed;
    if (startDual) {
        root.p2Y = base.latestState2().pos.y;
        root.p2V = static_cast<float>(base.latestState2().velocity);
        root.mode2 = base.latestState2().vehicle.type;
        root.held2 = base.press2;
        root.upside2 = base.latestState2().upsideDown;
        root.dash2 = base.latestState2().dashing;
        root.speed2 = base.latestState2().speed;
    }
    beam.push_back(root);

    float furthestDeath = 0.f;

    for (int depth = 1; depth <= config.lookahead && !stop.load(); ++depth) {
        std::vector<std::set<SearchInput>> candidates;
        bool anyDual = false;
        for (auto const& node : beam)
            anyDual = anyDual || node.dual;
        candidates.reserve(beam.size() * (anyDual ? 4 : 2));

        uint32_t decisionFrame = static_cast<uint32_t>(frame + depth - 1);
        for (auto const& node : beam) {
            // P1 stay.
            candidates.push_back(node.inputs);

            // P1 toggle. This single binary primitive covers jump presses,
            // releases, UFO taps, Ball flips, Spider edges, and flight steering.
            auto p1Toggle = node.inputs;
            p1Toggle.insert(inputKey(decisionFrame, false));
            candidates.push_back(std::move(p1Toggle));

            if (node.dual) {
                auto p2Toggle = node.inputs;
                p2Toggle.insert(inputKey(decisionFrame, true));
                candidates.push_back(std::move(p2Toggle));

                auto both = node.inputs;
                both.insert(inputKey(decisionFrame, false));
                both.insert(inputKey(decisionFrame, true));
                candidates.push_back(std::move(both));
            }
        }

        int workers = 1;
        auto results = evaluateBatchV17(
            base,
            candidates,
            depth,
            geometry,
            stop,
            workers
        );
        plan.evaluatorThreads = std::max(plan.evaluatorThreads, workers);
        plan.trials += results.size();
        plan.produced += static_cast<int>(results.size());

        std::unordered_map<uint64_t, MpcTrialV17> unique;
        unique.reserve(results.size());

        for (auto& trial : results) {
            if (trial.result.dead) {
                ++plan.dead;
                furthestDeath = std::max(furthestDeath, trial.result.deathX);
                continue;
            }

            uint64_t key = stateKeyV17(trial);
            auto it = unique.find(key);
            if (it == unique.end() || betterV17(trial, it->second, config))
                unique[key] = std::move(trial);
        }

        if (unique.empty()) {
            plan.deathX = furthestDeath;
            return plan;
        }

        beam.clear();
        beam.reserve(unique.size());
        for (auto& [_, trial] : unique)
            beam.push_back(std::move(trial));
        plan.unique = static_cast<int>(beam.size());

        std::sort(
            beam.begin(),
            beam.end(),
            [&](MpcTrialV17 const& a, MpcTrialV17 const& b) {
                return betterV17(a, b, config);
            }
        );
        if (beam.size() > static_cast<size_t>(config.beamWidth))
            beam.resize(static_cast<size_t>(config.beamWidth));

        if (beam.front().result.complete)
            break;
    }

    if (beam.empty()) {
        plan.deathX = furthestDeath;
        return plan;
    }

    auto const& best = beam.front();
    plan.inputs = best.inputs;
    plan.minClearance = best.result.minClearance;
    plan.complete = best.result.complete;
    plan.deathX = furthestDeath;

    int available = std::max(0, best.result.frame - frame);
    plan.commitFrames = plan.complete
        ? available
        : std::min(config.commitFrames, available);
    plan.found = plan.complete || plan.commitFrames > 0;
    return plan;
}

static bool validateResultV17(
    std::string const& lvlString,
    std::vector<PathfinderInput> const& inputs,
    float trustedEndX
) {
    Level2 verify(lvlString);
    if (std::isfinite(trustedEndX) && trustedEndX > verify.latestState().pos.x + 30.f)
        verify.length = trustedEndX;

    size_t cursor = 0;
    int safetyFrames = 240 * 60 * 8;

    while (!verify.latestState().dead && !reachedGoal(verify) && safetyFrames-- > 0) {
        uint32_t frame = static_cast<uint32_t>(verify.currentFrame());
        while (cursor < inputs.size() && inputs[cursor].frame <= frame) {
            auto const& input = inputs[cursor++];
            if (input.button != 1)
                continue;
            if (input.player2)
                verify.press2 = input.down;
            else
                verify.press1 = input.down;
        }
        verify.runFrame(verify.press1, verify.press2, 1.f / 240.f);
    }
    return reachedGoal(verify) && !verify.latestState().dead;
}

static PathfinderResult resultFromTimelineV17(
    Timeline const& timeline,
    std::string const& lvlString,
    float solveStartX,
    float endX,
    float furthestX,
    float trustedEndX,
    bool stopped,
    uint64_t totalTrials,
    int recoveries,
    int hardestSearch,
    int evaluatorThreads,
    bool usedRealGeometry,
    int sameWallRounds,
    float lastDeathX
) {
    PathfinderResult result;
    Replay2 output;

    for (size_t i = 1; i < timeline.p1.size(); ++i) {
        auto const& p1 = timeline.p1[i];
        auto const& previousP1 = timeline.p1[i - 1];
        if (p1.frame > 1 && p1.button != previousP1.button) {
            output.inputs.push_back(gdr::Input(p1.frame, 1, false, p1.button));
            result.inputs.push_back({
                static_cast<uint32_t>(p1.frame), false, p1.button, 1
            });
        }

        if (i < timeline.p2.size()) {
            auto const& p2 = timeline.p2[i];
            auto const& previousP2 = timeline.p2[i - 1];
            if (p2.dualActive && p2.frame > 1 && p2.button != previousP2.button) {
                output.inputs.push_back(gdr::Input(p2.frame, 1, true, p2.button));
                result.inputs.push_back({
                    static_cast<uint32_t>(p2.frame), true, p2.button, 1
                });
            }
        }
    }

    std::sort(result.inputs.begin(), result.inputs.end(), [](auto const& a, auto const& b) {
        if (a.frame != b.frame)
            return a.frame < b.frame;
        return a.player2 < b.player2;
    });

    if (result.inputs.empty() && timeline.frame() > 1 && timeline.x() > solveStartX + 1.f) {
        output.inputs.push_back(gdr::Input(1, 1, false, false));
        result.inputs.push_back({1u, false, false, 1u});
    }

    bool claimedComplete = timeline.complete();
    bool replayValid = !claimedComplete ||
        validateResultV17(lvlString, result.inputs, trustedEndX);

    result.macro = output.exportData().unwrapOr({});
    result.complete = claimedComplete && replayValid;
    result.progress = progressFor(
        furthestX,
        solveStartX,
        endX,
        result.complete
    );

    std::ostringstream diagnostics;
    diagnostics
        << "solver=universal-frame-mpc-v17"
        << " progress=" << result.progress
        << " frame=" << timeline.frame()
        << " routeX=" << timeline.x()
        << " furthestX=" << furthestX
        << " endX=" << endX
        << " totalTrials=" << totalTrials
        << " recoveries=" << recoveries
        << " hardestSearch=" << hardestSearch
        << " evaluatorThreads=" << evaluatorThreads
        << " realGeometry=" << (usedRealGeometry ? 1 : 0)
        << " sameWallRounds=" << sameWallRounds
        << " lastDeathX=" << lastDeathX
        << " replayValid=" << (replayValid ? 1 : 0)
        << " inputs=" << result.inputs.size()
        << " complete=" << (result.complete ? 1 : 0)
        << " stopped=" << (stopped ? 1 : 0)
        << " helpers=0";
    result.diagnostics = diagnostics.str();
    return result;
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
        lvl.lengthSource = "trusted-gd-v17";
    }

    Timeline bestPlayable(lvl);
    float furthestX = lvl.latestState().pos.x;
    float lastDeathX = 0.f;
    float debugClearance = 0.f;
    int recoveryCount = 0;
    int stagnantPlans = 0;
    int sameWallRounds = 0;
    int lastDeathBucket = std::numeric_limits<int>::min();
    int hardestSearch = 0;
    int debugWorkers = 1;
    int debugHorizon = 0;
    int debugCandidates = 0;
    int debugProduced = 0;
    int debugUnique = 0;
    int debugDead = 0;
    uint64_t totalTrials = 0;
    bool everUsedRealGeometry = false;
    auto lastMeaningfulAdvance = std::chrono::steady_clock::now();

    auto emit = [&](int phase, std::string reason, bool rescue = false) {
        PathfinderTelemetry t;
        t.progress = progressFor(furthestX, solveStartX, lvl.length, reachedGoal(lvl));
        t.startX = solveStartX;
        t.currentX = lvl.latestState().pos.x;
        t.furthestX = furthestX;
        t.trustedEndX = hasTrustedEnd ? trustedEndX : 0.f;
        t.inferredLength = simulatorLength;
        t.checkpointX = bestPlayable.x();
        t.deathX = lastDeathX;
        t.deathProgress = static_cast<float>(
            progressFor(lastDeathX, solveStartX, lvl.length, false)
        );
        t.bestClearance = debugClearance;
        t.frame = lvl.currentFrame();
        t.checkpointFrame = bestPlayable.frame();
        t.vehicleType = static_cast<int>(lvl.latestState().vehicle.type);
        t.searchLevel = std::min(12, recoveryCount + sameWallRounds);
        t.horizonFrames = debugHorizon;
        t.candidateCount = debugCandidates;
        t.workerCount = 1;
        t.physicalThreadCount = debugWorkers;
        t.phase = phase;
        t.frontierCount = debugUnique;
        t.producedCount = debugProduced;
        t.uniqueCount = debugUnique;
        t.deadCount = debugDead;
        t.stallLayers = stagnantPlans;
        t.recoveryCount = recoveryCount;
        t.deathClusterCount = sameWallRounds;
        t.stallRescue = rescue;
        t.totalTrials = totalTrials;
        t.mode = "universal-frame-mpc-v17";
        t.decision = std::string("Frame-exact MPC: ") +
            modeNameV17(lvl.latestState().vehicle.type) +
            " branches every 1/240 tick";
        t.recoveryReason = std::move(reason);
        publishPathfinderTelemetryV8(t);
        if (callback)
            callback(t);
    };

    auto recover = [&](int extraRetreat, std::string reason) {
        bestPlayable.restore(lvl);
        int retreat = std::min(
            std::max(0, bestPlayable.frame() - 1),
            extraRetreat
        );
        if (retreat > 0)
            lvl.rollback(std::max(1, bestPlayable.frame() - retreat));
        lvl.syncPresses();
        ++recoveryCount;
        stagnantPlans = 0;
        emit(2, std::move(reason), true);
    };

    emit(0, "all game modes use state-space MPC; every decision tick is searched");

    while (!reachedGoal(lvl) && !stop.load()) {
        try {
            if (lvl.latestState().dead) {
                recover(
                    std::min(4200, 360 + recoveryCount * 280),
                    "dead state recovery: backing up to a living timeline"
                );
                continue;
            }

            int frame = lvl.currentFrame();
            float beforeX = lvl.latestState().pos.x;
            int searchLevel = std::min(12, recoveryCount + sameWallRounds + stagnantPlans / 4);
            hardestSearch = std::max(hardestSearch, searchLevel);

            emit(0, "expanding HOLD/RELEASE state branches for this exact frame");
            auto plan = planMpcV17(lvl, searchLevel, stop);
            totalTrials += plan.trials;
            debugWorkers = plan.evaluatorThreads;
            debugHorizon = plan.lookaheadFrames;
            debugCandidates = plan.beamWidth * (lvl.latestState().dualActive ? 4 : 2);
            debugProduced = plan.produced;
            debugUnique = plan.unique;
            debugDead = plan.dead;
            debugClearance = plan.minClearance;
            everUsedRealGeometry = everUsedRealGeometry || plan.usedRealGeometry;

            if (plan.deathX > 0.f) {
                lastDeathX = plan.deathX;
                int bucket = static_cast<int>(std::floor(lastDeathX / 24.f));
                if (bucket == lastDeathBucket)
                    ++sameWallRounds;
                else
                    sameWallRounds = 1;
                lastDeathBucket = bucket;
            }

            emit(
                plan.found ? 1 : 2,
                plan.usedRealGeometry
                    ? "MPC scored every tick with gd-sim plus Show-Hitbox edge geometry"
                    : "MPC scored every tick with gd-sim collision geometry",
                !plan.found
            );

            if (stop.load())
                break;

            if (!plan.found) {
                recover(
                    std::min(5200, 480 + sameWallRounds * 360 + recoveryCount * 180),
                    "beam exhausted: deeper rollback and wider next search"
                );
                if (sameWallRounds >= 10 || recoveryCount >= 24) {
                    emit(2, "same wall survived bounded deep recoveries; returning best living route", true);
                    break;
                }
                continue;
            }

            int applyUntil = frame + plan.commitFrames;
            while (
                lvl.currentFrame() < applyUntil &&
                !lvl.latestState().dead &&
                !reachedGoal(lvl)
            ) {
                uint32_t current = static_cast<uint32_t>(lvl.currentFrame());
                if (plan.inputs.contains(inputKey(current, false)))
                    lvl.press1 = !lvl.press1;
                if (plan.inputs.contains(inputKey(current, true)))
                    lvl.press2 = !lvl.press2;
                lvl.runFrame(lvl.press1, lvl.press2, 1.f / 240.f);
            }

            if (lvl.latestState().dead) {
                lastDeathX = lvl.latestState().pos.x;
                recover(
                    std::min(5200, 600 + recoveryCount * 300),
                    "committed prefix died: reject it and replan from earlier state"
                );
                continue;
            }

            // Keep the longest living route, not only the route with the largest X.
            // This preserves legitimate reverse-direction and portal sections.
            if (lvl.currentFrame() > bestPlayable.frame())
                bestPlayable.capture(lvl);

            float nowX = lvl.latestState().pos.x;
            bool forward = lvl.latestState().direction >= 0;
            bool meaningful = forward
                ? nowX > furthestX + 3.f
                : lvl.currentFrame() > frame;

            furthestX = std::max(furthestX, nowX);

            if (meaningful || reachedGoal(lvl)) {
                stagnantPlans = 0;
                sameWallRounds = std::max(0, sameWallRounds - 1);
                recoveryCount = std::max(0, recoveryCount - 1);
                lastMeaningfulAdvance = std::chrono::steady_clock::now();
                emit(
                    3,
                    reachedGoal(lvl)
                        ? "goal reached; preparing independent replay validation"
                        : "safe MPC prefix committed; replanning from new exact state"
                );
            } else {
                ++stagnantPlans;
                emit(1, "prefix survived but made little net progress; next plan gets broader");
            }

            auto stalledFor = std::chrono::steady_clock::now() - lastMeaningfulAdvance;
            if (stagnantPlans >= 14 || stalledFor >= std::chrono::seconds(18)) {
                recover(
                    std::min(5600, 720 + recoveryCount * 360 + stagnantPlans * 40),
                    "no-progress watchdog: backing up instead of repeating one percentage"
                );
            }
        } catch (std::exception const&) {
            recover(
                std::min(5200, 720 + recoveryCount * 300),
                "solver exception recovered from best living timeline"
            );
        } catch (...) {
            recover(
                std::min(5200, 720 + recoveryCount * 300),
                "unknown solver exception recovered from best living timeline"
            );
        }
    }

    if (reachedGoal(lvl) && !lvl.latestState().dead) {
        bestPlayable.capture(lvl);
        furthestX = std::max(furthestX, lvl.latestState().pos.x);
        emit(4, "validating final input stream from level start");
    }

    return resultFromTimelineV17(
        bestPlayable,
        lvlString,
        solveStartX,
        lvl.length,
        furthestX,
        hasTrustedEnd ? trustedEndX : 0.f,
        stop.load(),
        totalTrials,
        recoveryCount,
        hardestSearch,
        debugWorkers,
        everUsedRealGeometry,
        sameWallRounds,
        lastDeathX
    );
}
