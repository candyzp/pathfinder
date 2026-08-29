#pragma once

// Windows still exposes the ancient `near` keyword as a macro through some SDK
// headers. Pathfinder uses normal modern C++ identifiers, so do not let that
// compatibility macro rewrite solver code.
#ifdef near
#undef near
#endif

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

struct PathfinderInput {
    uint32_t frame = 0;
    bool player2 = false;
    bool down = false;
    uint8_t button = 1;
};

struct PathfinderResult {
    std::vector<uint8_t> macro;
    std::vector<PathfinderInput> inputs;
    std::string diagnostics;
    double progress = 0.0;
    bool complete = false;
};

struct PathfinderTelemetry {
    double progress = 0.0;
    float startX = 0.f;
    float currentX = 0.f;
    float furthestX = 0.f;
    float trustedEndX = 0.f;
    float inferredLength = 0.f;
    float checkpointX = 0.f;
    float deathX = 0.f;
    float deathProgress = 0.f;
    float bestClearance = 0.f;
    int frame = 0;
    int checkpointFrame = 0;
    int vehicleType = 0;
    int searchLevel = 0;
    int horizonFrames = 0;
    int candidateCount = 0;
    int workerCount = 1;
    int phase = 0;
    uint64_t totalTrials = 0;
    std::string mode;
    std::string recoveryReason;
};

PathfinderResult pathfind(
    std::string const& lvlString,
    std::atomic_bool& stop,
    std::function<void(PathfinderTelemetry const&)> callback,
    float trustedEndX = 0.f
);
