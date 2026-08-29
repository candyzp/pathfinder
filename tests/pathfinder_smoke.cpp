#include <atomic>
#include <cassert>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "pathfinder.hpp"

static PathfinderResult solve(std::string const& level, float endX, int timeoutSeconds = 15) {
    std::atomic_bool stop = false;
    std::jthread watchdog([&](std::stop_token token) {
        int ticks = timeoutSeconds * 10;
        for (int i = 0; i < ticks && !token.stop_requested(); ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!token.stop_requested())
            stop = true;
    });
    auto result = pathfind(level, stop, {}, endX);
    watchdog.request_stop();
    return result;
}

int main() {
    std::string header = "kA4,0,kA3,0,kA11,0,kA2,0;";

    auto empty = solve(header, 300.f, 3);
    assert(empty.complete);
    assert(empty.progress == 100.0);
    assert(empty.inputs.empty());

    // One normal-ground spike requires a real jump; this also verifies exact
    // one-frame branching and that committed input export remains intact.
    auto spike = solve(header + "1,8,2,115,3,15;", 420.f);
    assert(spike.complete);
    assert(!spike.inputs.empty());

    std::string waveHeader = "kA4,0,kA3,0,kA11,0,kA2,4;";
    std::string waveCorridor = waveHeader +
        "1,1,2,140,3,50,129,3;"
        "1,1,2,280,3,250,129,3;";
    auto wave = solve(waveCorridor, 520.f, 20);
    assert(wave.complete);
    assert(!wave.inputs.empty());

    // Even a uniquely resolved simulator teleport cannot poison progress when
    // PlayLayer supplied a contradictory trusted endpoint.
    auto poisonedTeleport = solve(
        header +
            "1,2902,2,40,3,15,51,7;"
            "1,9999,2,10000,3,15,57,7;",
        500.f,
        2
    );
    assert(!poisonedTeleport.complete);
    assert(poisonedTeleport.progress < 25.0);

    std::cout << empty.diagnostics << '\n'
              << spike.diagnostics << '\n'
              << wave.diagnostics << '\n'
              << poisonedTeleport.diagnostics << '\n';
    return 0;
}
