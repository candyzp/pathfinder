#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

#include <Level.hpp>

int main() {
    std::string header = "kA4,0,kA3,0,kA11,0,kA2,0;";

    Level sparse(header +
        "1,1,2,100,3,15;"
        "1,1,2,1000000,3,15;");
    assert(sparse.sections.size() == 2);
    assert(sparse.inferredLength < 1000.f);

    Level linkedByTeleport(header +
        "1,2902,2,40,3,15,51,7;"
        "1,9999,2,10000,3,15,57,7;");
    assert(linkedByTeleport.inferredLength > 10000.f);

    for (int i = 0; i < 100 && !linkedByTeleport.latestState().teleported; ++i)
        linkedByTeleport.runFrame(false);
    assert(linkedByTeleport.latestState().teleported);
    assert(std::abs(linkedByTeleport.latestState().teleportToX - 10000.f) < 0.1f);

    Level ambiguousTeleport(header +
        "1,2902,2,40,3,15,51,7;"
        "1,9998,2,10000,3,15,57,7;"
        "1,9999,2,11000,3,15,57,7;");
    assert(ambiguousTeleport.inferredLength < 1000.f);
    for (int i = 0; i < 100; ++i)
        ambiguousTeleport.runFrame(false);
    assert(ambiguousTeleport.latestState().pos.x < 1000.f);

    Level compact(header);
    for (int i = 0; i < 20; ++i)
        compact.runFrame(false);
    int absoluteFrame = compact.currentFrame();
    compact.gameStates.erase(compact.gameStates.begin(), compact.gameStates.end() - 2);
    compact.gameStates2.erase(compact.gameStates2.begin(), compact.gameStates2.end() - 2);
    assert(compact.currentFrame() == absoluteFrame);
    compact.runFrame(false);
    assert(compact.currentFrame() == absoluteFrame + 1);

    std::cout << "level regression tests passed\n";
    return 0;
}
