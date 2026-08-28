#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

struct PathfinderInput {
    uint32_t frame = 0;
    bool player2 = false;
    bool down = false;
};

struct PathfinderResult {
    std::vector<uint8_t> macro;
    std::vector<PathfinderInput> inputs;
    double progress = 0.0;
    bool complete = false;
};

PathfinderResult pathfind(
    std::string const& lvlString,
    std::atomic_bool& stop,
    std::function<void(double)> callback
);
