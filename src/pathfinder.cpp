#include <set>
#include <cstdint>
#include <algorithm>
#include <cmath>
#include <functional>
#include <Level.hpp>
#include <limits>
#include <vector>
#include <unordered_map>
#include <gdr/gdr.hpp>
#include "pathfinder.hpp"

class Replay2 : public gdr::Replay<Replay2, gdr::Input<"">> {
public:
    Replay2() : Replay("Pathfinder", 1) {}
};

struct Level2 : public Level {
    bool press1 = false;
    bool press2 = false;
    float highestY = 0.f;
    using Level::Level;

    Level2(std::string const& lvlString) : Level(lvlString) {
        for (auto const& section : sections)
            for (auto const& object : section)
                highestY = std::max(highestY, object->getTop());
    }

    void syncPresses() {
        press1 = latestState().button;
        press2 = latestState2().button;
    }
};

using SearchInput = uint32_t;
using InputSet = std::set<SearchInput>;

static SearchInput inputKey(uint32_t frame, bool player2) {
    return (frame << 1) | static_cast<uint32_t>(player2);
}

static bool continuousMode(VehicleType type) {
    return type == VehicleType::Ship || type == VehicleType::Wave || type == VehicleType::Swing;
}

static bool reachedGoal(Level2 const& lvl) {
    return lvl.gameStates.back().completed;
}

static void setButton(InputSet& inputs, int frame, bool player2, bool before, bool after) {
    if (before != after)
        inputs.insert(inputKey(static_cast<uint32_t>(frame), player2));
}

struct Snapshot {
    std::vector<Player> p1;
    std::vector<Player> p2;
    bool press1 = false;
    bool press2 = false;
};

static Snapshot capture(Level2 const& lvl) {
    return {lvl.gameStates, lvl.gameStates2, lvl.press1, lvl.press2};
}

static void restore(Level2& lvl, Snapshot const& s) {
    lvl.gameStates = s.p1;
    lvl.gameStates2 = s.p2;
    lvl.press1 = s.press1;
    lvl.press2 = s.press2;
}

static float localClearance(Level2 const& lvl, Player const& player) {
    if (lvl.sections.empty()) return 999.f;
    int section = std::clamp(static_cast<int>(player.pos.x / Level::sectionSize), 0,
                             static_cast<int>(lvl.sections.size()) - 1);
    float best = 999.f;
    for (int s = std::max(0, section - 1); s <= std::min(static_cast<int>(lvl.sections.size()) - 1, section + 2); ++s) {
        for (auto const& object : lvl.sections[s]) {
            if (object->prio != 1 && object->prio != 2) continue;
            float dx = 0.f;
            if (player.getRight() < object->getLeft()) dx = object->getLeft() - player.getRight();
            else if (object->getRight() < player.getLeft()) dx = player.getLeft() - object->getRight();
            float dy = 0.f;
            if (player.getTop() < object->getBottom()) dy = object->getBottom() - player.getTop();
            else if (object->getTop() < player.getBottom()) dy = player.getBottom() - object->getTop();
            best = std::min(best, std::sqrt(dx * dx + dy * dy));
        }
    }
    return best;
}

static bool invalidState(Level2 const& lvl, Player const& p, float trustedEndX) {
    if (!std::isfinite(p.pos.x) || !std::isfinite(p.pos.y) || !std::isfinite(p.velocity)) return true;
    if (p.pos.y > std::max(1500.f, lvl.highestY + 700.f) || p.pos.y < -700.f) return true;
    if (trustedEndX > 0.f && !p.completed && p.pos.x > trustedEndX + 120.f) return true;
    return false;
}

static uint64_t mixHash(uint64_t h, int64_t v) {
    h ^= static_cast<uint64_t>(v) + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
    return h;
}

static uint64_t stateHash(Level2 const& lvl) {
    auto const& p = lvl.gameStates.back();
    uint64_t h = 1469598103934665603ull;
    h = mixHash(h, p.frame / 4);
    h = mixHash(h, static_cast<int64_t>(std::llround(p.pos.x / 8.f)));
    h = mixHash(h, static_cast<int64_t>(std::llround(p.pos.y / 6.f)));
    h = mixHash(h, static_cast<int64_t>(std::llround(p.velocity / 45.0)));
    h = mixHash(h, static_cast<int>(p.vehicle.type));
    h = mixHash(h, p.upsideDown ? 1 : 0);
    h = mixHash(h, p.button ? 1 : 0);
    h = mixHash(h, p.direction);
    h = mixHash(h, p.dualActive ? 1 : 0);
    if (p.dualActive) {
        auto const& q = lvl.gameStates2.back();
        h = mixHash(h, static_cast<int64_t>(std::llround(q.pos.y / 6.f)));
        h = mixHash(h, static_cast<int64_t>(std::llround(q.velocity / 45.0)));
        h = mixHash(h, static_cast<int>(q.vehicle.type));
        h = mixHash(h, q.upsideDown ? 1 : 0);
        h = mixHash(h, q.button ? 1 : 0);
    }
    return h;
}

struct WaveGuide {
    bool valid = false;
    float targetY = 0.f;
    float halfWidth = 0.f;
};

static WaveGuide waveGuideAt(Level2 const& lvl, Player const& p, float lookX) {
    float lowBound = std::isfinite(p.floor) ? p.floor + p.size.y + 4.f : 0.f;
    float highBound = std::isfinite(p.ceiling) && p.ceiling < 100000.f
        ? p.ceiling - p.size.y - 4.f
        : std::max(lowBound + 120.f, lvl.highestY + 180.f);
    if (highBound <= lowBound + 12.f) return {};

    struct Interval { float a; float b; };
    std::vector<Interval> blocked;
    int section = std::clamp(static_cast<int>(lookX / Level::sectionSize), 0,
                             std::max(0, static_cast<int>(lvl.sections.size()) - 1));
    for (int s = std::max(0, section - 1); s <= std::min(static_cast<int>(lvl.sections.size()) - 1, section + 1); ++s) {
        for (auto const& o : lvl.sections[s]) {
            if (o->prio != 1 && o->prio != 2) continue;
            if (lookX < o->getLeft() - 8.f || lookX > o->getRight() + 8.f) continue;
            float margin = p.size.y + (o->prio == 2 ? 7.f : 4.f);
            blocked.push_back({o->getBottom() - margin, o->getTop() + margin});
        }
    }
    std::sort(blocked.begin(), blocked.end(), [](auto const& a, auto const& b) { return a.a < b.a; });

    float cursor = lowBound;
    float bestA = lowBound, bestB = lowBound;
    auto considerGap = [&](float a, float b) {
        if (b <= a) return;
        float center = (a + b) * .5f;
        float bestCenter = (bestA + bestB) * .5f;
        float width = b - a, bestWidth = bestB - bestA;
        float score = width - std::abs(center - p.pos.y) * .12f;
        float bestScore = bestWidth - std::abs(bestCenter - p.pos.y) * .12f;
        if (score > bestScore) { bestA = a; bestB = b; }
    };

    for (auto const& r : blocked) {
        if (r.b <= lowBound || r.a >= highBound) continue;
        considerGap(cursor, std::min(highBound, r.a));
        cursor = std::max(cursor, r.b);
        if (cursor >= highBound) break;
    }
    considerGap(cursor, highBound);
    if (bestB - bestA < 8.f) return {};
    return {true, (bestA + bestB) * .5f, (bestB - bestA) * .5f};
}

struct Branch {
    Snapshot state;
    InputSet inputs;
    float x = 0.f;
    float y = 0.f;
    double velocity = 0.0;
    float clearance = 0.f;
    float guideError = 0.f;
    int frame = 0;
    bool dead = false;
    bool complete = false;
};

static bool betterBranch(Branch const& a, Branch const& b) {
    if (a.complete != b.complete) return a.complete;
    if (a.dead != b.dead) return !a.dead;
    if (a.frame != b.frame && a.dead && b.dead) return a.frame > b.frame;
    if (std::abs(a.x - b.x) > 8.f) return a.x > b.x;
    if (std::abs(a.guideError - b.guideError) > 5.f) return a.guideError < b.guideError;
    if (std::abs(a.clearance - b.clearance) > 1.f) return a.clearance > b.clearance;
    return a.frame > b.frame;
}

static Branch simulateDecision(Level2& lvl, Branch const& parent, bool p1Held, bool p2Held,
                               int frames, float trustedEndX) {
    restore(lvl, parent.state);
    Branch out = parent;
    int start = lvl.currentFrame();
    setButton(out.inputs, start, false, lvl.press1, p1Held);
    if (lvl.latestState().dualActive)
        setButton(out.inputs, start, true, lvl.press2, p2Held);
    lvl.press1 = p1Held;
    lvl.press2 = p2Held;

    for (int i = 0; i < frames && !lvl.latestState().dead && !reachedGoal(lvl); ++i) {
        lvl.runFrame(lvl.press1, lvl.press2, 1.f / 240.f);
        if (invalidState(lvl, lvl.latestState(), trustedEndX)) {
            lvl.latestState().dead = true;
            break;
        }
    }

    auto const& p = lvl.latestState();
    out.state = capture(lvl);
    out.x = p.pos.x;
    out.y = p.pos.y;
    out.velocity = p.velocity;
    out.frame = lvl.currentFrame();
    out.dead = p.dead;
    out.complete = !p.dead && reachedGoal(lvl);
    out.clearance = p.dead ? 0.f : localClearance(lvl, p);
    out.guideError = 0.f;
    if (!p.dead && p.vehicle.type == VehicleType::Wave) {
        float speed = player_speeds[static_cast<int>(p.speed)];
        float lookX = p.pos.x + std::max(45.f, speed * .16f);
        auto guide = waveGuideAt(lvl, p, lookX);
        if (guide.valid) {
            out.guideError = std::abs(p.pos.y - guide.targetY);
            out.clearance += std::min(guide.halfWidth, 60.f) * .2f;
        }
    }
    return out;
}

static std::vector<std::pair<bool, bool>> decisionsFor(Level2 const& lvl, WaveGuide const& guide) {
    auto const& p = lvl.gameStates.back();
    std::vector<std::pair<bool, bool>> d;
    auto add = [&](bool a, bool b) {
        if (std::find(d.begin(), d.end(), std::pair<bool, bool>{a, b}) == d.end()) d.push_back({a, b});
    };

    bool preferred = p.button;
    if (p.vehicle.type == VehicleType::Wave && guide.valid) {
        bool needUp = p.pos.y < guide.targetY;
        preferred = p.upsideDown ? !needUp : needUp;
        add(preferred, lvl.press2);
        add(!preferred, lvl.press2);
    } else {
        add(p.button, lvl.press2);
        add(!p.button, lvl.press2);
    }

    if (p.dualActive) {
        auto const& q = lvl.gameStates2.back();
        add(preferred, !q.button);
        add(!preferred, !q.button);
    }
    return d;
}

static Branch beamPlan(Level2& lvl, std::atomic_bool& stop, int horizonFrames,
                       int difficulty, float trustedEndX) {
    Branch root;
    root.state = capture(lvl);
    root.x = lvl.latestState().pos.x;
    root.y = lvl.latestState().pos.y;
    root.velocity = lvl.latestState().velocity;
    root.frame = lvl.currentFrame();
    root.clearance = localClearance(lvl, lvl.latestState());

    auto mode = lvl.latestState().vehicle.type;
    int chunk = mode == VehicleType::Wave ? 6 : continuousMode(mode) ? 12 : 18;
    int beamWidth = mode == VehicleType::Wave ? 14 : continuousMode(mode) ? 12 : 9;
    beamWidth += std::min(difficulty, 3);
    int maxDepth = std::max(1, horizonFrames / chunk);

    std::vector<Branch> beam{root};
    Branch best = root;

    for (int depth = 0; depth < maxDepth && !stop && !beam.empty(); ++depth) {
        std::vector<Branch> next;
        std::unordered_map<uint64_t, size_t> buckets;
        next.reserve(static_cast<size_t>(beamWidth * 3));

        for (auto const& parent : beam) {
            restore(lvl, parent.state);
            auto const& p = lvl.latestState();
            WaveGuide guide;
            if (p.vehicle.type == VehicleType::Wave) {
                float speed = player_speeds[static_cast<int>(p.speed)];
                guide = waveGuideAt(lvl, p, p.pos.x + std::max(35.f, speed * .12f));
            }
            auto decisions = decisionsFor(lvl, guide);

            for (auto const& decision : decisions) {
                auto child = simulateDecision(lvl, parent, decision.first, decision.second, chunk, trustedEndX);
                if (betterBranch(child, best)) best = child;
                if (child.complete) return child;
                if (child.dead) continue;

                restore(lvl, child.state);
                uint64_t key = stateHash(lvl);
                auto it = buckets.find(key);
                if (it == buckets.end()) {
                    buckets[key] = next.size();
                    next.push_back(std::move(child));
                } else if (betterBranch(child, next[it->second])) {
                    next[it->second] = std::move(child);
                }
            }
        }

        std::sort(next.begin(), next.end(), betterBranch);
        if (next.size() > static_cast<size_t>(beamWidth)) next.resize(static_cast<size_t>(beamWidth));
        beam = std::move(next);
        if (!beam.empty() && betterBranch(beam.front(), best)) best = beam.front();
    }

    restore(lvl, root.state);
    return best;
}

static void applyInputs(Level2& lvl, InputSet const& inputs, int untilFrame) {
    while (lvl.currentFrame() < untilFrame && !lvl.latestState().dead && !reachedGoal(lvl)) {
        uint32_t frame = static_cast<uint32_t>(lvl.currentFrame());
        if (inputs.contains(inputKey(frame, false))) lvl.press1 = !lvl.press1;
        if (inputs.contains(inputKey(frame, true))) lvl.press2 = !lvl.press2;
        lvl.runFrame(lvl.press1, lvl.press2, 1.f / 240.f);
    }
}

PathfinderResult pathfind(std::string const& lvlString, std::atomic_bool& stop,
                          std::function<void(double)> callback, float trustedEndX) {
    Level2 lvl(lvlString);
    float solveStartX = lvl.latestState().pos.x;
    float inferredLength = lvl.length;
    bool hasTrustedEnd = std::isfinite(trustedEndX) && trustedEndX > solveStartX + 30.f;
    if (hasTrustedEnd) lvl.length = trustedEndX;

    auto progressFor = [&](float x, bool complete) {
        if (complete) return 100.0;
        double span = static_cast<double>(lvl.length - solveStartX);
        if (span <= 1.0) return 0.0;
        return std::clamp(((static_cast<double>(x) - solveStartX) / span) * 100.0, 0.0, 99.95);
    };

    float furthestX = solveStartX;
    Level2 lvlBest = lvl;
    int stagnant = 0;
    int recovery = 0;

    auto recover = [&](int extra) {
        lvl = lvlBest;
        int retreat = std::min(std::max(0, lvlBest.currentFrame() - 1), 180 + recovery * 120 + extra);
        lvl.rollback(std::max(1, lvlBest.currentFrame() - retreat));
        lvl.syncPresses();
        recovery = std::min(8, recovery + 1);
        stagnant = 0;
    };

    while (!reachedGoal(lvl) && !stop) {
        if (lvl.latestState().dead) { recover(240); continue; }

        int frame = lvl.currentFrame();
        auto mode = lvl.latestState().vehicle.type;
        int difficulty = std::min(5, recovery + stagnant);
        int horizon = (mode == VehicleType::Wave ? 420 : continuousMode(mode) ? 480 : 420) + difficulty * 48;
        auto best = beamPlan(lvl, stop, horizon, difficulty, hasTrustedEnd ? trustedEndX : 0.f);
        if (stop) break;

        int span = std::max(0, best.frame - frame);
        float gain = best.x - lvl.latestState().pos.x;
        if (best.complete) {
            applyInputs(lvl, best.inputs, best.frame);
        } else if (span > 0 && gain > 1.f) {
            int commit = 0;
            if (best.dead) {
                int margin = mode == VehicleType::Wave ? 48 : 36;
                commit = std::max(0, span - margin);
                commit = std::min(commit, mode == VehicleType::Wave ? 96 : 144);
            } else {
                int pct = mode == VehicleType::Wave ? 34 : continuousMode(mode) ? 46 : 64;
                commit = std::max(1, span * pct / 100);
            }
            if (commit <= 0) { recover(180); continue; }
            applyInputs(lvl, best.inputs, frame + commit);
        } else {
            ++stagnant;
            recover(stagnant >= 2 ? 360 : 180);
            continue;
        }

        auto const& p = lvl.latestState();
        if (invalidState(lvl, p, hasTrustedEnd ? trustedEndX : 0.f)) {
            recover(420);
            continue;
        }

        if (!p.dead && (reachedGoal(lvl) || p.pos.x > furthestX + 1.f)) {
            furthestX = std::max(furthestX, p.pos.x);
            lvlBest = lvl;
            stagnant = 0;
            recovery = std::max(0, recovery - 1);
        } else {
            ++stagnant;
        }

        if (stagnant >= 3) { recover(360); continue; }
        if (callback) callback(progressFor(furthestX, reachedGoal(lvlBest)));
    }

    if (!lvl.latestState().dead && (reachedGoal(lvl) || lvl.latestState().pos.x > furthestX)) {
        furthestX = std::max(furthestX, lvl.latestState().pos.x);
        lvlBest = lvl;
    }

    PathfinderResult result;
    Replay2 output;
    for (size_t i = 1; i < lvlBest.gameStates.size(); ++i) {
        auto const& p1 = lvlBest.gameStates[i];
        auto const& p1Prev = lvlBest.gameStates[i - 1];
        if (p1.frame > 1 && p1.button != p1Prev.button) {
            output.inputs.push_back(gdr::Input(p1.frame, 1, false, p1.button));
            result.inputs.push_back({static_cast<uint32_t>(p1.frame), false, p1.button});
        }
        if (i < lvlBest.gameStates2.size()) {
            auto const& p2 = lvlBest.gameStates2[i];
            auto const& p2Prev = lvlBest.gameStates2[i - 1];
            if (p2.dualActive && p2.frame > 1 && p2.button != p2Prev.button) {
                output.inputs.push_back(gdr::Input(p2.frame, 1, true, p2.button));
                result.inputs.push_back({static_cast<uint32_t>(p2.frame), true, p2.button});
            }
        }
    }
    std::sort(result.inputs.begin(), result.inputs.end(), [](auto const& a, auto const& b) {
        return a.frame != b.frame ? a.frame < b.frame : a.player2 < b.player2;
    });
    result.macro = output.exportData().unwrapOr({});
    result.complete = !lvlBest.latestState().dead && reachedGoal(lvlBest);
    result.progress = progressFor(furthestX, result.complete);
    (void)inferredLength;
    return result;
}
