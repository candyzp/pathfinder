#pragma once

#include "real_geometry.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <thread>
#include <unordered_map>
#include <vector>

// This header is intentionally included from pathfinder_single.cpp after
// pathfinder.cpp. It therefore uses Level2/SearchInput/inputKey/hazardClearance
// from the single solver translation unit without exposing simulator internals.

struct WaveMpcTrialV16 {
    std::set<SearchInput> inputs;
    TrialResult result;
    float finalY = 0.f;
    float finalVelocity = 0.f;
    bool held = false;
    bool upsideDown = false;
    bool dashing = false;
    bool leftWave = false;
    int speed = 0;
};

struct WaveMpcPlanV16 {
    std::set<SearchInput> inputs;
    int commitFrames = 0;
    int lookaheadFrames = 0;
    int beamWidth = 0;
    int evaluatorThreads = 1;
    uint64_t trials = 0;
    float minClearance = 0.f;
    float deathX = 0.f;
    bool found = false;
    bool complete = false;
    bool usedRealGeometry = false;
};

static float waveRealClearanceV16(
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

static WaveMpcTrialV16 evaluateWaveMpcCandidateV16(
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
    bool syntheticCollision = false;
    bool leftWave = false;

    while (
        worker.currentFrame() < endFrame &&
        !worker.latestState().dead &&
        !reachedGoal(worker)
    ) {
        uint32_t current = static_cast<uint32_t>(worker.currentFrame());
        if (inputs.contains(inputKey(current, false)))
            worker.press1 = !worker.press1;

        worker.runFrame(worker.press1, worker.press2, 1.f / 240.f);

        auto const& state = worker.latestState();
        if (state.dead) {
            deathX = state.pos.x;
            break;
        }

        // Unlike the old flight candidate search, clearance is sampled on EVERY
        // 240-TPS tick here. This is the expensive mode by design.
        float simClearance = hazardClearance(worker, state);
        float realClearance = waveRealClearanceV16(geometry, state);
        float frameClearance = std::min(simClearance, realClearance);
        minClearance = std::min(minClearance, frameClearance);

        // The real-geometry snapshot is a second collision oracle sourced from
        // Geometry Dash's own Show-Hitbox-style object rectangles/polygons.
        if (geometry && realClearance <= 0.01f) {
            syntheticCollision = true;
            deathX = state.pos.x;
            break;
        }

        if (state.vehicle.type != VehicleType::Wave) {
            leftWave = true;
            break;
        }
    }

    auto const& finalState = worker.latestState();
    bool dead = finalState.dead || syntheticCollision;
    if (!std::isfinite(minClearance))
        minClearance = 10000.f;

    WaveMpcTrialV16 out;
    out.inputs = inputs;
    out.result = {
        worker.currentFrame(),
        finalState.pos.x,
        dead ? (deathX != 0.f ? deathX : finalState.pos.x) : 0.f,
        minClearance,
        dead,
        !dead && reachedGoal(worker)
    };
    out.finalY = finalState.pos.y;
    out.finalVelocity = static_cast<float>(finalState.velocity);
    out.held = worker.press1;
    out.upsideDown = finalState.upsideDown;
    out.dashing = finalState.dashing;
    out.leftWave = !dead && leftWave;
    out.speed = finalState.speed;
    return out;
}

static int waveMpcWorkerCountV16(size_t count) {
    if (count <= 1)
        return 1;
    constexpr size_t maxWorkers = 20;
    return static_cast<int>(std::min(count, maxWorkers));
}

static std::vector<WaveMpcTrialV16> evaluateWaveMpcBatchV16(
    Level2 const& base,
    std::vector<std::set<SearchInput>> const& candidates,
    int depthFrames,
    std::shared_ptr<PathfinderRealGeometry const> const& geometry,
    std::atomic_bool& stop,
    int& workersUsed
) {
    std::vector<WaveMpcTrialV16> results(candidates.size());
    if (candidates.empty()) {
        workersUsed = 1;
        return results;
    }

    workersUsed = waveMpcWorkerCountV16(candidates.size());
    std::atomic<size_t> next {0};
    std::vector<std::thread> threads;
    threads.reserve(static_cast<size_t>(workersUsed));

    for (int worker = 0; worker < workersUsed; ++worker) {
        threads.emplace_back([&] {
            while (!stop.load()) {
                size_t index = next.fetch_add(1, std::memory_order_relaxed);
                if (index >= candidates.size())
                    break;
                results[index] = evaluateWaveMpcCandidateV16(
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

static uint64_t waveMpcStateKeyV16(WaveMpcTrialV16 const& trial) {
    auto mix = [](uint64_t h, uint64_t v) {
        h ^= v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
        return h * 1099511628211ull;
    };

    int qY = static_cast<int>(std::llround(trial.finalY * 2.f));
    int qV = static_cast<int>(std::llround(trial.finalVelocity * 4.f));
    int qX = static_cast<int>(std::llround(trial.result.x * 0.25f));

    uint64_t h = 1469598103934665603ull;
    h = mix(h, static_cast<uint32_t>(qY));
    h = mix(h, static_cast<uint32_t>(qV));
    h = mix(h, static_cast<uint32_t>(qX));
    h = mix(h, static_cast<uint32_t>(trial.speed + 8));
    h = mix(h, trial.held ? 1u : 0u);
    h = mix(h, trial.upsideDown ? 2u : 0u);
    h = mix(h, trial.dashing ? 4u : 0u);
    h = mix(h, trial.leftWave ? 8u : 0u);
    return h;
}

static double waveMpcScoreV16(WaveMpcTrialV16 const& trial) {
    if (trial.result.complete)
        return 1e15;
    if (trial.result.dead)
        return -1e12 + static_cast<double>(trial.result.x);

    // At one beam depth most branches have similar X, so clearance becomes the
    // important signal. Cap it so an empty room does not dominate forever.
    double clearance = std::clamp(
        static_cast<double>(trial.result.minClearance),
        0.0,
        60.0
    );
    double score =
        static_cast<double>(trial.result.x) * 4.0 +
        clearance * 18.0 -
        static_cast<double>(trial.inputs.size()) * 0.35;

    // Reaching a portal out of Wave is a successful terminal state for this
    // local planner. The normal all-mode solver takes over on the next loop.
    if (trial.leftWave)
        score += 1e8;
    return score;
}

static bool waveMpcBetterV16(
    WaveMpcTrialV16 const& a,
    WaveMpcTrialV16 const& b
) {
    double sa = waveMpcScoreV16(a);
    double sb = waveMpcScoreV16(b);
    if (std::abs(sa - sb) > 1e-9)
        return sa > sb;
    if (a.result.minClearance != b.result.minClearance)
        return a.result.minClearance > b.result.minClearance;
    return a.inputs.size() < b.inputs.size();
}

static WaveMpcPlanV16 planWaveMpcV16(
    Level2 const& base,
    int searchLevel,
    bool allowRealGeometry,
    std::atomic_bool& stop
) {
    WaveMpcPlanV16 plan;
    auto geometry = allowRealGeometry ? getPathfinderRealGeometry() : nullptr;
    plan.usedRealGeometry = static_cast<bool>(geometry);

    // Every layer is ONE 1/240-second input decision. Beam width grows when the
    // solver is recovering, but remains bounded so memory does not explode.
    int lookahead = std::min(168, 96 + searchLevel * 8);
    int beamWidth = std::min(256, 112 + searchLevel * 12);
    plan.lookaheadFrames = lookahead;
    plan.beamWidth = beamWidth;

    int frame = base.currentFrame();
    std::vector<WaveMpcTrialV16> beam;
    WaveMpcTrialV16 root;
    root.result = {
        frame,
        base.latestState().pos.x,
        0.f,
        waveRealClearanceV16(geometry, base.latestState()),
        false,
        reachedGoal(base)
    };
    root.finalY = base.latestState().pos.y;
    root.finalVelocity = static_cast<float>(base.latestState().velocity);
    root.held = base.press1;
    root.upsideDown = base.latestState().upsideDown;
    root.dashing = base.latestState().dashing;
    root.speed = base.latestState().speed;
    beam.push_back(root);

    float furthestDeath = 0.f;

    for (int depth = 1; depth <= lookahead && !stop.load(); ++depth) {
        std::vector<std::set<SearchInput>> candidates;
        candidates.reserve(beam.size() * 2);

        uint32_t decisionFrame = static_cast<uint32_t>(frame + depth - 1);
        for (auto const& node : beam) {
            // Branch A: keep the current button state for this exact frame.
            candidates.push_back(node.inputs);

            // Branch B: toggle on this exact frame. Together A/B cover every
            // possible Wave decision at every simulated 240-TPS tick.
            auto toggled = node.inputs;
            toggled.insert(inputKey(decisionFrame, false));
            candidates.push_back(std::move(toggled));
        }

        int workers = 1;
        auto results = evaluateWaveMpcBatchV16(
            base,
            candidates,
            depth,
            geometry,
            stop,
            workers
        );
        plan.evaluatorThreads = std::max(plan.evaluatorThreads, workers);
        plan.trials += results.size();

        std::unordered_map<uint64_t, WaveMpcTrialV16> unique;
        unique.reserve(results.size());

        for (auto& trial : results) {
            if (trial.result.dead) {
                furthestDeath = std::max(furthestDeath, trial.result.deathX);
                continue;
            }

            uint64_t key = waveMpcStateKeyV16(trial);
            auto it = unique.find(key);
            if (it == unique.end() || waveMpcBetterV16(trial, it->second))
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

        std::sort(beam.begin(), beam.end(), waveMpcBetterV16);
        if (beam.size() > static_cast<size_t>(beamWidth))
            beam.resize(static_cast<size_t>(beamWidth));

        if (beam.front().result.complete || beam.front().leftWave)
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
    // Receding-horizon control: trust only a tiny prefix, then solve again from
    // the new exact state. Eight ticks is ~33 ms at 240 TPS.
    plan.commitFrames = std::min(8, available);
    plan.found = plan.commitFrames > 0 || plan.complete;
    return plan;
}
