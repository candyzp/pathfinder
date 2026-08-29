#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

// Windows headers still expose the legacy `near` macro. It collides with local
// identifiers in the solver and causes Clang/MSVC parsing errors on Windows.
#ifdef near
#undef near
#endif

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
    int frame = 0;
    int checkpointFrame = 0;
    std::string mode;
    std::string recoveryReason;
};

PathfinderResult pathfind(
    std::string const& lvlString,
    std::atomic_bool& stop,
    std::function<void(PathfinderTelemetry const&)> callback,
    float trustedEndX = 0.f
);
