// Universal Pathfinder V18: fast incremental frame-exact MPC for every mode.
//
// V17 had the right state-space idea but the wrong execution model: at beam
// depth N it copied the root Level2 and replayed N frames for every candidate.
// That made one local plan quadratic in horizon length and copied full histories
// tens of thousands of times before committing a few frames.
//
// V18 keeps one compact simulator state per surviving beam node. Each branch
// advances exactly ONE new 1/240-second frame, near-identical physics states are
// merged immediately, and only a short safe prefix is committed before replanning.
#define pathfind pathfind_plain_unused_v18
#include "pathfinder.cpp"
#undef pathfind

#include "real_geometry.hpp"
#include "solver_dashboard.hpp"

#include <chrono>
#include <memory>
#include <unordered_map>

namespace {

struct MpcConfigV18 {
    int lookahead = 64;
    int beamWidth = 48;
    int commitFrames = 12;
    double clearanceWeight = 12.0;
    double togglePenalty = 0.16;
};

struct BeamNodeV18 {
    Level2 sim;
    std::vector<SearchInput> inputs;
    float minClearance = std::numeric_limits<float>::infinity();
    float deathX = 0.f;

    explicit BeamNodeV18(Level2 const& source) : sim(source) {}
};

struct MpcPlanV18 {
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

static char const* modeNameV18(VehicleType mode) {
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

static MpcConfigV18 configForV18(VehicleType mode, int searchLevel, bool dual) {
    MpcConfigV18 config;
    switch (mode) {
        case VehicleType::Cube:
            config = {56, 48, 16, 8.0, 0.26};
            break;
        case VehicleType::Ship:
            config = {68, 64, 12, 18.0, 0.07};
            break;
        case VehicleType::Ball:
            config = {64, 52, 14, 10.0, 0.14};
            break;
        case VehicleType::Ufo:
            config = {64, 56, 12, 14.0, 0.11};
            break;
        case VehicleType::Wave:
            config = {76, 80, 10, 22.0, 0.045};
            break;
        case VehicleType::Robot:
            config = {72, 60, 14, 10.0, 0.16};
            break;
        case VehicleType::Spider:
            config = {64, 52, 12, 12.0, 0.10};
            break;
        case VehicleType::Swing:
            config = {72, 68, 10, 18.0, 0.07};
            break;
    }

    // Recovery increases precision/capacity gradually instead of starting every
    // trivial section with the largest possible search.
    config.lookahead = std::min(112, config.lookahead + searchLevel * 4);
    config.beamWidth = std::min(128, config.beamWidth + searchLevel * 6);

    // Dual has four actions per tick. Keep it bounded while still searching both
    // players independently on every simulated frame.
    if (dual) {
        config.lookahead = std::min(config.lookahead, 76);
        config.beamWidth = std::min(config.beamWidth, 72);
        config.commitFrames = std::min(config.commitFrames, 8);
    }
    return config;
}

static bool flightLikeV18(VehicleType mode) {
    return mode == VehicleType::Ship ||
           mode == VehicleType::Wave ||
           mode == VehicleType::Swing;
}

static float realGeometryClearanceV18(
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

// Branch Level2 copies do not need the entire route history. Player::postCollision
// only needs frame-1, and Level::getState supports a vector whose first element is
// an arbitrary frame. Keeping two states makes every later branch copy roughly
// constant-size with respect to elapsed level time.
//
// Do NOT call Level2::fixStatePointers here. That routine rebuilds move-trigger
// activation history from gameStates, which we intentionally compact. The copied
// moveTriggers already contain their live activationFrame values.
static void compactBranchHistoryV18(Level2& lvl) {
    auto trim = [](std::vector<Player>& states) {
        if (states.size() > 2)
            states.erase(states.begin(), states.end() - 2);
    };
    trim(lvl.gameStates);
    trim(lvl.gameStates2);

    for (auto& state : lvl.gameStates)
        state.level = &lvl;
    for (auto& state : lvl.gameStates2)
        state.level = &lvl;
}

static float sampleClearanceV18(
    Level2 const& lvl,
    std::shared_ptr<PathfinderRealGeometry const> const& geometry
) {
    auto const& p1 = lvl.latestState();
    float clearance = hazardClearance(lvl, p1);

    // Real GD solid overlap can be a legal landing for grounded modes. Use the
    // Show-Hitbox snapshot as an extra edge-distance oracle for flight-like modes
    // and let gd-sim remain the source of truth for grounded contact legality.
    if (geometry && flightLikeV18(p1.vehicle.type))
        clearance = std::min(clearance, realGeometryClearanceV18(geometry, p1));

    if (p1.dualActive) {
        auto const& p2 = lvl.latestState2();
        clearance = std::min(clearance, hazardClearance(lvl, p2));
        if (geometry && flightLikeV18(p2.vehicle.type))
            clearance = std::min(clearance, realGeometryClearanceV18(geometry, p2));
    }
    return clearance;
}

static bool outOfBoundsV18(Player const& player, float highestY) {
    return player.pos.y > std::max(1500.f, highestY + 700.f) ||
           player.pos.y < -700.f;
}

static bool advanceNodeV18(
    BeamNodeV18& node,
    bool toggleP1,
    bool toggleP2,
    std::shared_ptr<PathfinderRealGeometry const> const& geometry
) {
    // A moved/copied BeamNode changes the Level2 address. Repair the retained
    // Player::level pointers immediately before using Player::prevPlayer().
    compactBranchHistoryV18(node.sim);

    uint32_t frame = static_cast<uint32_t>(node.sim.currentFrame());
    if (toggleP1) {
        node.sim.press1 = !node.sim.press1;
        node.inputs.push_back(inputKey(frame, false));
    }
    if (toggleP2) {
        node.sim.press2 = !node.sim.press2;
        node.inputs.push_back(inputKey(frame, true));
    }

    node.sim.runFrame(node.sim.press1, node.sim.press2, 1.f / 240.f);
    ++node.sim.latestState().frame;
    --node.sim.latestState().frame;

    auto const& p1 = node.sim.latestState();
    bool dead = p1.dead || outOfBoundsV18(p1, node.sim.highestY);
    if (p1.dualActive) {
        auto const& p2 = node.sim.latestState2();
        dead = dead || p2.dead || outOfBoundsV18(p2, node.sim.highestY);
    }

    if (dead) {
        node.deathX = p1.pos.x;
        return false;
    }

    float clearance = sampleClearanceV18(node.sim, geometry);
    node.minClearance = std::min(node.minClearance, clearance);
    compactBranchHistoryV18(node.sim);
    return true;
}

static uint64_t mixStateV18(uint64_t h, uint64_t v) {
    h ^= v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
    return h * 1099511628211ull;
}

static uint64_t addPlayerStateV18(uint64_t h, Player const& p, bool held) {
    int qX = static_cast<int>(std::llround(p.pos.x * 0.5f));
    int qY = static_cast<int>(std::llround(p.pos.y * 2.f));
    int qV = static_cast<int>(std::llround(p.velocity * 4.f));
    int qRobot = static_cast<int>(std::llround(p.robotBoostTime * 240.0));
    int qWarp = static_cast<int>(std::llround(p.timeWarp * 32.f));
    int qGravity = static_cast<int>(std::llround(p.gravityScale * 32.f));
    uint32_t coyote = std::min<unsigned int>(p.coyoteFrames, 255u);

    h = mixStateV18(h, static_cast<uint32_t>(qX));
    h = mixStateV18(h, static_cast<uint32_t>(qY));
    h = mixStateV18(h, static_cast<uint32_t>(qV));
    h = mixStateV18(h, static_cast<uint32_t>(static_cast<int>(p.vehicle.type) + 1));
    h = mixStateV18(h, static_cast<uint32_t>(p.speed + 8));
    h = mixStateV18(h, static_cast<uint32_t>(qRobot));
    h = mixStateV18(h, static_cast<uint32_t>(qWarp));
    h = mixStateV18(h, static_cast<uint32_t>(qGravity));
    h = mixStateV18(h, coyote);
    h = mixStateV18(h, held ? 1u : 0u);
    h = mixStateV18(h, p.input ? 2u : 0u);
    h = mixStateV18(h, p.buffer ? 4u : 0u);
    h = mixStateV18(h, p.vehicleBuffer ? 8u : 0u);
    h = mixStateV18(h, p.grounded ? 16u : 0u);
    h = mixStateV18(h, p.upsideDown ? 32u : 0u);
    h = mixStateV18(h, p.dashing ? 64u : 0u);
    h = mixStateV18(h, p.small ? 128u : 0u);
    h = mixStateV18(h, p.direction < 0 ? 256u : 0u);
    h = mixStateV18(h, p.dualActive ? 512u : 0u);
    return h;
}

static uint64_t stateKeyV18(BeamNodeV18 const& node) {
    auto const& p1 = node.sim.latestState();
    uint64_t h = 1469598103934665603ull;
    h = addPlayerStateV18(h, p1, node.sim.press1);
    if (p1.dualActive)
        h = addPlayerStateV18(h, node.sim.latestState2(), node.sim.press2);
    return h;
}

static double scoreV18(BeamNodeV18 const& node, MpcConfigV18 const& config) {
    auto const& p1 = node.sim.latestState();
    if (reachedGoal(node.sim))
        return 1e15;

    double clearance = std::clamp(
        static_cast<double>(node.minClearance),
        0.0,
        80.0
    );
    double directionalX = p1.direction < 0
        ? -static_cast<double>(p1.pos.x)
        : static_cast<double>(p1.pos.x);

    return directionalX * 5.0 +
           clearance * config.clearanceWeight +
           static_cast<double>(node.sim.currentFrame()) * 0.04 -
           static_cast<double>(node.inputs.size()) * config.togglePenalty;
}

static bool betterV18(
    BeamNodeV18 const& a,
    BeamNodeV18 const& b,
    MpcConfigV18 const& config
) {
    double sa = scoreV18(a, config);
    double sb = scoreV18(b, config);
    if (std::abs(sa - sb) > 1e-8)
        return sa > sb;
    if (a.minClearance != b.minClearance)
        return a.minClearance > b.minClearance;
    return a.inputs.size() < b.inputs.size();
}

static MpcPlanV18 planMpcV18(
    Level2 const& base,
    int searchLevel,
    std::atomic_bool& stop
) {
    MpcPlanV18 plan;
    VehicleType startMode = base.latestState().vehicle.type;
    bool startDual = base.latestState().dualActive;
    MpcConfigV18 config = configForV18(startMode, searchLevel, startDual);

    auto geometry = base.movingObjectIDs.empty()
        ? getPathfinderRealGeometry()
        : std::shared_ptr<PathfinderRealGeometry const> {};
    plan.usedRealGeometry = static_cast<bool>(geometry);
    plan.lookaheadFrames = config.lookahead;
    plan.beamWidth = config.beamWidth;

    int startFrame = base.currentFrame();
    std::vector<BeamNodeV18> beam;
    beam.reserve(static_cast<size_t>(config.beamWidth));
    beam.emplace_back(base);
    compactBranchHistoryV18(beam.back().sim);
    beam.back().minClearance = sampleClearanceV18(beam.back().sim, geometry);

    float furthestDeath = 0.f;

    for (int depth = 1; depth <= config.lookahead && !stop.load(); ++depth) {
        bool anyDual = false;
        for (auto const& node : beam)
            anyDual = anyDual || node.sim.latestState().dualActive;

        std::vector<BeamNodeV18> expanded;
        expanded.reserve(beam.size() * (anyDual ? 4 : 2));

        auto keepIfAlive = [&](BeamNodeV18&& child, bool t1, bool t2) {
            ++plan.trials;
            ++plan.produced;
            if (advanceNodeV18(child, t1, t2, geometry)) {
                expanded.push_back(std::move(child));
            } else {
                ++plan.dead;
                furthestDeath = std::max(furthestDeath, child.deathX);
            }
        };

        for (auto& parent : beam) {
            if (stop.load())
                break;

            bool dual = parent.sim.latestState().dualActive;

            // Copy only the alternatives. The no-toggle child reuses the parent
            // state directly, cutting Level2 copies roughly in half for normal play.
            BeamNodeV18 stay = std::move(parent);
            BeamNodeV18 p1Toggle = stay;

            if (dual) {
                BeamNodeV18 p2Toggle = stay;
                BeamNodeV18 bothToggle = stay;
                keepIfAlive(std::move(p2Toggle), false, true);
                keepIfAlive(std::move(bothToggle), true, true);
            }

            keepIfAlive(std::move(p1Toggle), true, false);
            keepIfAlive(std::move(stay), false, false);
        }

        if (expanded.empty()) {
            plan.deathX = furthestDeath;
            return plan;
        }

        // Merge equivalent physics futures immediately. We retain only the safer,
        // simpler route to a state, not every historical input script that reached it.
        std::unordered_map<uint64_t, size_t> chosen;
        chosen.reserve(expanded.size());
        std::vector<BeamNodeV18> unique;
        unique.reserve(expanded.size());

        for (auto& node : expanded) {
            uint64_t key = stateKeyV18(node);
            auto it = chosen.find(key);
            if (it == chosen.end()) {
                chosen.emplace(key, unique.size());
                unique.push_back(std::move(node));
            } else if (betterV18(node, unique[it->second], config)) {
                unique[it->second] = std::move(node);
            }
        }

        plan.unique = static_cast<int>(unique.size());

        auto better = [&](BeamNodeV18 const& a, BeamNodeV18 const& b) {
            return betterV18(a, b, config);
        };

        if (unique.size() > static_cast<size_t>(config.beamWidth)) {
            auto cut = unique.begin() + config.beamWidth;
            std::nth_element(unique.begin(), cut, unique.end(), better);
            unique.resize(static_cast<size_t>(config.beamWidth));
        }
        std::sort(unique.begin(), unique.end(), better);
        beam = std::move(unique);

        if (!beam.empty() && reachedGoal(beam.front().sim))
            break;
    }

    if (beam.empty()) {
        plan.deathX = furthestDeath;
        return plan;
    }

    auto const& best = beam.front();
    plan.inputs.insert(best.inputs.begin(), best.inputs.end());
    plan.minClearance = best.minClearance;
    plan.complete = reachedGoal(best.sim) && !best.sim.latestState().dead;
    plan.deathX = furthestDeath;

    int available = std::max(0, best.sim.currentFrame() - startFrame);
    plan.commitFrames = plan.complete
        ? available
        : std::min(config.commitFrames, available);
    plan.found = plan.complete || plan.commitFrames > 0;
    return plan;
}

static bool validateResultV18(
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

static PathfinderResult resultFromTimelineV18(
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
        validateResultV18(lvlString, result.inputs, trustedEndX);

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
        << "solver=universal-incremental-mpc-v18"
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
        lvl.lengthSource = "trusted-gd-v18";
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
        t.mode = "universal-incremental-mpc-v18";
        t.decision = std::string("Incremental frame MPC: ") +
            modeNameV18(lvl.latestState().vehicle.type) +
            " searches every 1/240 tick";
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

    emit(0, "V18 incremental MPC: no candidate replays from the horizon root");

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
            int searchLevel = std::min(12, recoveryCount + sameWallRounds + stagnantPlans / 4);
            hardestSearch = std::max(hardestSearch, searchLevel);

            emit(0, "advancing surviving physics states one frame instead of replaying them");
            auto plan = planMpcV18(lvl, searchLevel, stop);
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
                    ? "incremental beam scored with gd-sim plus Show-Hitbox edge geometry"
                    : "incremental beam scored with gd-sim collision geometry",
                !plan.found
            );

            if (stop.load())
                break;

            if (!plan.found) {
                recover(
                    std::min(5200, 480 + sameWallRounds * 360 + recoveryCount * 180),
                    "beam exhausted: rollback and expand the next incremental search"
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
                        : "safe prefix committed; fast incremental replanning continues"
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

    return resultFromTimelineV18(
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
