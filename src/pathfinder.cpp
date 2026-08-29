#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Level.hpp>
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

    explicit Level2(std::string const& lvlString) : Level(lvlString) {
        for (auto const& [_, section] : sections) {
            for (auto const& object : section)
                highestY = std::max(highestY, object->pos.y);
        }
    }

    void syncPresses() {
        press1 = latestState().button;
        press2 = latestState2().button;
    }

    void fixStatePointers() {
        for (auto& state : gameStates)
            state.level = this;
        for (auto& state : gameStates2)
            state.level = this;
    }
};

using SearchInput = uint32_t;
using InputList = std::vector<SearchInput>;

static SearchInput inputKey(uint32_t frame, bool player2) {
    return (frame << 1) | static_cast<uint32_t>(player2);
}

static uint32_t inputFrame(SearchInput input) {
    return input >> 1;
}

static bool inputPlayer2(SearchInput input) {
    return (input & 1u) != 0;
}

static char const* vehicleName(VehicleType type) {
    switch (type) {
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

static bool continuousMode(VehicleType type) {
    return type == VehicleType::Ship ||
           type == VehicleType::Wave ||
           type == VehicleType::Swing;
}

static bool reachedGoal(Level2 const& lvl) {
    return !lvl.gameStates.empty() && lvl.gameStates.back().completed;
}

struct TrialResult {
    int frame = 0;
    float x = 0.f;
    float y = 0.f;
    double velocity = 0.0;
    float clearance = 0.f;
    bool dead = false;
    bool complete = false;
    bool survivedHorizon = false;
    bool validProgress = true;
};

struct CandidateResult {
    InputList inputs;
    TrialResult trial;
    uint64_t stateHash = 0;
};

struct SearchStats {
    uint64_t expandedStates = 0;
    uint64_t simulatedFrames = 0;
    uint64_t deduplicatedStates = 0;
    uint64_t rejectedJumps = 0;
    int horizon = 0;
    int beamWidth = 0;
};

struct SearchBatch {
    std::vector<CandidateResult> candidates;
    SearchStats stats;
};

static float localClearance(Level2 const& lvl, Player const& player) {
    if (lvl.sections.empty())
        return 999.f;

    int center = static_cast<int>(std::floor(player.pos.x / Level::sectionSize));
    float best = 999.f;

    for (int section = center - 2; section <= center + 2; ++section) {
        auto it = lvl.sections.find(section);
        if (it == lvl.sections.end())
            continue;

        for (auto const& object : it->second) {
            if (object->prio != 1 && object->prio != 2)
                continue;

            float dx = 0.f;
            if (player.getRight() < object->getLeft())
                dx = object->getLeft() - player.getRight();
            else if (object->getRight() < player.getLeft())
                dx = player.getLeft() - object->getRight();

            float dy = 0.f;
            if (player.getTop() < object->getBottom())
                dy = object->getBottom() - player.getTop();
            else if (object->getTop() < player.getBottom())
                dy = player.getBottom() - object->getTop();

            best = std::min(best, std::sqrt(dx * dx + dy * dy));
        }
    }
    return best;
}

static bool betterCandidate(CandidateResult const& a, CandidateResult const& b) {
    if (a.trial.complete != b.trial.complete)
        return a.trial.complete;
    if (a.trial.validProgress != b.trial.validProgress)
        return a.trial.validProgress;
    if (a.trial.survivedHorizon != b.trial.survivedHorizon)
        return a.trial.survivedHorizon;
    if (a.trial.dead != b.trial.dead)
        return !a.trial.dead;

    // Completion > survival > safe progress. A route that survives longer to
    // reach the next obstacle beats one that gains a few pixels and dies now.
    if (a.trial.frame != b.trial.frame)
        return a.trial.frame > b.trial.frame;

    constexpr float progressTie = 6.f;
    if (a.trial.x > b.trial.x + progressTie)
        return true;
    if (b.trial.x > a.trial.x + progressTie)
        return false;

    if (a.trial.clearance > b.trial.clearance + 1.f)
        return true;
    if (b.trial.clearance > a.trial.clearance + 1.f)
        return false;

    // Input count is deliberately absent. A one-frame Wave train is allowed to
    // beat a visually cleaner route whenever it is safer.
    return false;
}

static bool sameStateBucket(CandidateResult const& a, CandidateResult const& b) {
    return a.stateHash == b.stateHash &&
           a.trial.complete == b.trial.complete &&
           a.trial.dead == b.trial.dead &&
           a.trial.survivedHorizon == b.trial.survivedHorizon &&
           std::abs(a.trial.x - b.trial.x) < 8.f &&
           std::abs(a.trial.y - b.trial.y) < 6.f &&
           std::abs(a.trial.velocity - b.trial.velocity) < 45.0;
}

struct VerticalInterval {
    float low = 0.f;
    float high = 0.f;
};

static std::pair<float, float> objectHalfExtents(Object const& object) {
    float radians = deg2rad(object.rotation);
    float c = std::abs(std::cos(radians));
    float s = std::abs(std::sin(radians));
    return {
        c * object.size.x * 0.5f + s * object.size.y * 0.5f,
        s * object.size.x * 0.5f + c * object.size.y * 0.5f
    };
}

static std::vector<VerticalInterval> waveSafeIntervalsAt(
    Level2 const& lvl,
    Player const& player,
    float sampleX
) {
    float waveHalf = player.small ? 3.f : 5.f;
    float lowBound = player.floor + waveHalf + 2.5f;
    float highBound = player.ceiling - waveHalf - 2.5f;

    if (!std::isfinite(highBound) || highBound > lowBound + 1200.f)
        highBound = lowBound + 300.f;
    if (highBound <= lowBound)
        return {};

    std::vector<VerticalInterval> blocked;
    int center = static_cast<int>(std::floor(sampleX / Level::sectionSize));

    for (int section = center - 2; section <= center + 2; ++section) {
        auto it = lvl.sections.find(section);
        if (it == lvl.sections.end())
            continue;

        for (auto const& container : it->second) {
            auto const& object = *container.operator->();
            if (object.prio != 1 && object.prio != 2)
                continue;

            auto [halfX, halfY] = objectHalfExtents(object);
            float horizontalMargin = waveHalf + (object.prio == 2 ? 5.f : 2.f);
            if (std::abs(sampleX - object.pos.x) > halfX + horizontalMargin)
                continue;

            float verticalMargin = waveHalf + (object.prio == 2 ? 6.f : 3.f);
            blocked.push_back({
                std::max(lowBound, object.pos.y - halfY - verticalMargin),
                std::min(highBound, object.pos.y + halfY + verticalMargin)
            });
        }
    }

    std::sort(blocked.begin(), blocked.end(), [](auto const& a, auto const& b) {
        return a.low < b.low;
    });

    std::vector<VerticalInterval> merged;
    for (auto interval : blocked) {
        if (interval.high <= interval.low)
            continue;
        if (merged.empty() || interval.low > merged.back().high)
            merged.push_back(interval);
        else
            merged.back().high = std::max(merged.back().high, interval.high);
    }

    std::vector<VerticalInterval> safe;
    float cursor = lowBound;
    for (auto const& interval : merged) {
        if (interval.low > cursor + 2.f)
            safe.push_back({cursor, interval.low});
        cursor = std::max(cursor, interval.high);
    }
    if (cursor < highBound - 2.f)
        safe.push_back({cursor, highBound});
    return safe;
}

static float waveClearance(Level2 const& lvl, Player const& player) {
    auto intervals = waveSafeIntervalsAt(lvl, player, player.pos.x);
    for (auto const& interval : intervals) {
        if (player.pos.y >= interval.low && player.pos.y <= interval.high)
            return std::min(player.pos.y - interval.low, interval.high - player.pos.y);
    }
    return -40.f;
}

struct WaveGuide {
    float startX = 0.f;
    float stepX = 1.f;
    std::vector<float> targets;

    bool empty() const { return targets.empty(); }

    float targetAt(float x) const {
        if (targets.empty())
            return 0.f;
        int index = static_cast<int>(std::llround((x - startX) / stepX));
        index = std::clamp(index, 0, static_cast<int>(targets.size()) - 1);
        return targets[static_cast<size_t>(index)];
    }
};

static WaveGuide buildWaveGuide(Level2 const& lvl, Player const& player, int horizonFrames) {
    WaveGuide guide;
    guide.startX = player.pos.x;
    guide.stepX = player.direction >= 0 ? 24.f : -24.f;

    float speed = static_cast<float>(player_speeds[std::clamp(player.speed, 0, 4)]);
    float travel = speed * std::clamp(player.timeWarp, 0.05f, 4.f) *
                   static_cast<float>(horizonFrames) / 240.f;
    int slices = std::clamp(static_cast<int>(travel / 24.f) + 3, 4, 80);

    struct GuideNode {
        float cost = std::numeric_limits<float>::infinity();
        float target = 0.f;
        int parent = -1;
        VerticalInterval interval;
    };

    std::vector<std::vector<GuideNode>> layers;
    layers.reserve(static_cast<size_t>(slices));
    float reachableSlope = player.small ? 2.f : 1.f;

    for (int slice = 0; slice < slices; ++slice) {
        float x = guide.startX + guide.stepX * static_cast<float>(slice);
        auto intervals = waveSafeIntervalsAt(lvl, player, x);
        if (intervals.empty())
            break;

        std::vector<GuideNode> layer;
        layer.reserve(intervals.size());
        for (auto interval : intervals) {
            GuideNode node;
            node.interval = interval;
            float inset = std::min(6.f, std::max(0.f, (interval.high - interval.low) * 0.15f));
            float usableLow = interval.low + inset;
            float usableHigh = interval.high - inset;

            if (layers.empty()) {
                node.target = std::clamp(player.pos.y, usableLow, usableHigh);
                float width = std::max(1.f, interval.high - interval.low);
                node.cost = std::abs(node.target - player.pos.y) * 0.2f + 24.f / width;
            } else {
                float maxDelta = std::abs(guide.stepX) * reachableSlope + 7.f;
                for (size_t prevIndex = 0; prevIndex < layers.back().size(); ++prevIndex) {
                    auto const& prev = layers.back()[prevIndex];
                    float target = std::clamp(prev.target, usableLow, usableHigh);
                    float delta = std::abs(target - prev.target);
                    float reachPenalty = delta > maxDelta ? 500.f + (delta - maxDelta) * 15.f : 0.f;
                    float width = std::max(1.f, interval.high - interval.low);
                    float cost = prev.cost + delta * 0.12f + 24.f / width + reachPenalty;
                    if (cost < node.cost) {
                        node.cost = cost;
                        node.target = target;
                        node.parent = static_cast<int>(prevIndex);
                    }
                }
            }
            layer.push_back(node);
        }
        layers.push_back(std::move(layer));
    }

    if (layers.empty())
        return guide;

    size_t best = 0;
    for (size_t i = 1; i < layers.back().size(); ++i) {
        if (layers.back()[i].cost < layers.back()[best].cost)
            best = i;
    }

    guide.targets.resize(layers.size());
    std::vector<VerticalInterval> chosenIntervals(layers.size());
    for (size_t layer = layers.size(); layer-- > 0;) {
        auto const& node = layers[layer][best];
        guide.targets[layer] = node.target;
        chosenIntervals[layer] = node.interval;
        if (node.parent >= 0)
            best = static_cast<size_t>(node.parent);
    }

    // Pull earlier targets toward upcoming openings at the Wave's reachable
    // slope. Without this backward pass, a centerline could notice a high gap
    // only at the wall and ask the Wave to climb after it was already too late.
    float maxTargetDelta = std::abs(guide.stepX) * reachableSlope * 0.85f;
    for (size_t i = guide.targets.size(); i-- > 1;) {
        float wanted = std::clamp(
            guide.targets[i - 1],
            guide.targets[i] - maxTargetDelta,
            guide.targets[i] + maxTargetDelta
        );
        guide.targets[i - 1] = std::clamp(
            wanted,
            chosenIntervals[i - 1].low + 3.f,
            chosenIntervals[i - 1].high - 3.f
        );
    }

    float previous = player.pos.y;
    for (size_t i = 0; i < guide.targets.size(); ++i) {
        guide.targets[i] = std::clamp(
            guide.targets[i],
            previous - maxTargetDelta,
            previous + maxTargetDelta
        );
        previous = guide.targets[i];
    }
    return guide;
}

struct CompactSnapshot {
    std::array<Player, 2> p1;
    std::array<Player, 2> p2;
    uint8_t count = 0;
    bool press1 = false;
    bool press2 = false;
};

static CompactSnapshot captureSnapshot(Level2 const& lvl) {
    CompactSnapshot snapshot;
    size_t count = std::min<size_t>(2, lvl.gameStates.size());
    snapshot.count = static_cast<uint8_t>(count);
    size_t start = lvl.gameStates.size() - count;
    for (size_t i = 0; i < count; ++i) {
        snapshot.p1[i] = lvl.gameStates[start + i];
        snapshot.p2[i] = lvl.gameStates2[start + i];
    }
    snapshot.press1 = lvl.press1;
    snapshot.press2 = lvl.press2;
    return snapshot;
}

static CompactSnapshot captureHistoryTail(
    std::vector<Player> const& p1,
    std::vector<Player> const& p2,
    bool press1,
    bool press2
) {
    CompactSnapshot snapshot;
    size_t count = std::min<size_t>(2, p1.size());
    snapshot.count = static_cast<uint8_t>(count);
    size_t start = p1.size() - count;
    for (size_t i = 0; i < count; ++i) {
        snapshot.p1[i] = p1[start + i];
        snapshot.p2[i] = p2[start + i];
    }
    snapshot.press1 = press1;
    snapshot.press2 = press2;
    return snapshot;
}

static Player const& snapshotP1(CompactSnapshot const& snapshot) {
    return snapshot.p1[static_cast<size_t>(snapshot.count - 1)];
}

static Player const& snapshotP2(CompactSnapshot const& snapshot) {
    return snapshot.p2[static_cast<size_t>(snapshot.count - 1)];
}

static void restoreSnapshot(Level2& lvl, CompactSnapshot const& snapshot) {
    lvl.gameStates.clear();
    lvl.gameStates2.clear();
    lvl.gameStates.reserve(2);
    lvl.gameStates2.reserve(2);
    for (size_t i = 0; i < snapshot.count; ++i) {
        lvl.gameStates.push_back(snapshot.p1[i]);
        lvl.gameStates2.push_back(snapshot.p2[i]);
    }
    lvl.press1 = snapshot.press1;
    lvl.press2 = snapshot.press2;
    lvl.fixStatePointers();
}

class CompactSimulationGuard {
    Level2& m_level;
    std::vector<Player> m_p1;
    std::vector<Player> m_p2;
    bool m_press1;
    bool m_press2;

public:
    CompactSnapshot root;

    explicit CompactSimulationGuard(Level2& level)
        : m_level(level),
          m_p1(std::move(level.gameStates)),
          m_p2(std::move(level.gameStates2)),
          m_press1(level.press1),
          m_press2(level.press2) {
        // Capture only the two states future physics can observe, then move the
        // committed history out of the hot search loop. Static level geometry is
        // never copied per branch.
        root = captureHistoryTail(m_p1, m_p2, m_press1, m_press2);
        restoreSnapshot(level, root);
    }

    ~CompactSimulationGuard() {
        m_level.gameStates = std::move(m_p1);
        m_level.gameStates2 = std::move(m_p2);
        m_level.press1 = m_press1;
        m_level.press2 = m_press2;
        m_level.fixStatePointers();
    }
};

struct TraceNode {
    std::shared_ptr<TraceNode const> parent;
    InputList toggles;
};

using Trace = std::shared_ptr<TraceNode const>;

static Trace extendTrace(Trace parent, InputList toggles) {
    if (toggles.empty())
        return parent;
    return std::make_shared<TraceNode const>(TraceNode {std::move(parent), std::move(toggles)});
}

static InputList materializeTrace(Trace trace) {
    std::vector<TraceNode const*> chain;
    for (auto current = trace; current; current = current->parent)
        chain.push_back(current.get());

    InputList inputs;
    for (auto it = chain.rbegin(); it != chain.rend(); ++it)
        inputs.insert(inputs.end(), (*it)->toggles.begin(), (*it)->toggles.end());

    std::sort(inputs.begin(), inputs.end());
    inputs.erase(std::unique(inputs.begin(), inputs.end()), inputs.end());
    return inputs;
}

static uint64_t mix64(uint64_t value) {
    value += 0x9e3779b97f4a7c15ull;
    value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ull;
    value = (value ^ (value >> 27)) * 0x94d049bb133111ebull;
    return value ^ (value >> 31);
}

static void hashCombine(uint64_t& hash, uint64_t value) {
    hash ^= mix64(value + hash);
}

static int64_t quantize(double value, double quantum) {
    if (!std::isfinite(value))
        return std::numeric_limits<int64_t>::max();
    return static_cast<int64_t>(std::llround(value / quantum));
}

static uint64_t effectsFingerprint(Player const& player) {
    uint64_t fingerprint = mix64(player.usedEffects.size());
    for (int id : player.usedEffects.get())
        fingerprint ^= mix64(static_cast<uint64_t>(static_cast<uint32_t>(id)));
    return fingerprint;
}

static void hashPlayer(uint64_t& hash, Player const& player) {
    hashCombine(hash, static_cast<uint64_t>(player.frame));
    hashCombine(hash, static_cast<uint64_t>(quantize(player.pos.x, 3.0)));
    hashCombine(hash, static_cast<uint64_t>(quantize(player.pos.y, 2.5)));
    hashCombine(hash, static_cast<uint64_t>(quantize(player.velocity, 27.0)));
    hashCombine(hash, static_cast<uint64_t>(quantize(player.acceleration, 80.0)));
    hashCombine(hash, static_cast<uint64_t>(quantize(player.rotation, 4.0)));
    hashCombine(hash, static_cast<uint64_t>(quantize(player.size.x, 1.0)));
    hashCombine(hash, static_cast<uint64_t>(quantize(player.size.y, 1.0)));
    hashCombine(hash, static_cast<uint64_t>(quantize(player.floor, 6.0)));
    hashCombine(hash, static_cast<uint64_t>(quantize(player.ceiling, 6.0)));
    hashCombine(hash, static_cast<uint64_t>(quantize(player.gravityScale, 0.05)));
    hashCombine(hash, static_cast<uint64_t>(quantize(player.timeWarp, 0.05)));
    hashCombine(hash, static_cast<uint64_t>(quantize(player.robotBoostTime, 0.01)));
    hashCombine(hash, static_cast<uint64_t>(quantize(player.dashAngle, 2.0)));
    hashCombine(hash, static_cast<uint64_t>(quantize(player.dashSpeed, 12.0)));
    hashCombine(hash, static_cast<uint64_t>(std::min(player.coyoteFrames, 64u)));
    hashCombine(hash, static_cast<uint64_t>(player.snapData.playerFrame));
    hashCombine(hash, static_cast<uint64_t>(quantize(player.snapData.nextX, 2.0)));

    uint64_t flags = static_cast<uint64_t>(player.vehicle.type);
    flags |= static_cast<uint64_t>(player.speed & 7) << 4;
    flags |= static_cast<uint64_t>(player.button) << 8;
    flags |= static_cast<uint64_t>(player.input) << 9;
    flags |= static_cast<uint64_t>(player.buffer) << 10;
    flags |= static_cast<uint64_t>(player.vehicleBuffer) << 11;
    flags |= static_cast<uint64_t>(player.grounded) << 12;
    flags |= static_cast<uint64_t>(player.upsideDown) << 13;
    flags |= static_cast<uint64_t>(player.small) << 14;
    flags |= static_cast<uint64_t>(player.dashing) << 15;
    flags |= static_cast<uint64_t>(player.dualActive) << 16;
    flags |= static_cast<uint64_t>(player.direction < 0) << 17;
    flags |= static_cast<uint64_t>(!player.actions.empty()) << 18;
    flags |= static_cast<uint64_t>(player.slopeData.slope.has_value()) << 19;
    hashCombine(hash, flags);
    hashCombine(hash, effectsFingerprint(player));
}

static uint64_t hashSnapshot(CompactSnapshot const& snapshot) {
    uint64_t hash = 0xcbf29ce484222325ull;
    hashCombine(hash, snapshot.count);
    for (size_t i = 0; i < snapshot.count; ++i) {
        hashPlayer(hash, snapshot.p1[i]);
        if (snapshot.p1[i].dualActive)
            hashPlayer(hash, snapshot.p2[i]);
    }
    hashCombine(hash, static_cast<uint64_t>(snapshot.press1));
    hashCombine(hash, static_cast<uint64_t>(snapshot.press2));
    return hash;
}

struct InputPattern {
    std::vector<uint8_t> p1;
    std::vector<uint8_t> p2;
};

static uint64_t hashBits(std::vector<uint8_t> const& bits) {
    uint64_t hash = bits.size();
    for (uint8_t bit : bits)
        hash = (hash << 1) ^ (hash >> 61) ^ static_cast<uint64_t>(bit + 1);
    return hash;
}

static std::vector<uint8_t> guidedWavePattern(
    Player const& player,
    WaveGuide const& guide,
    int frames
) {
    std::vector<uint8_t> pattern(static_cast<size_t>(frames), player.button);
    float x = player.pos.x;
    float y = player.pos.y;
    float timeScale = std::clamp(player.timeWarp, 0.05f, 4.f);
    float horizontal = static_cast<float>(player_speeds[std::clamp(player.speed, 0, 4)]) *
                       timeScale / 240.f;
    float vertical = horizontal * (player.small ? 2.f : 1.f);

    for (int frame = 0; frame < frames; ++frame) {
        float target = guide.targetAt(x + guide.stepX * 1.75f);
        bool wantsWorldUp;
        if (std::abs(target - y) <= 1.25f)
            wantsWorldUp = player.upsideDown ? !pattern[static_cast<size_t>(frame)]
                                             : pattern[static_cast<size_t>(frame)];
        else
            wantsWorldUp = target > y;

        bool pressed = player.upsideDown ? !wantsWorldUp : wantsWorldUp;
        pattern[static_cast<size_t>(frame)] = static_cast<uint8_t>(pressed);
        float worldSign = (pressed ? 1.f : -1.f) * (player.upsideDown ? -1.f : 1.f);
        y += worldSign * vertical;
        x += static_cast<float>(player.direction) * horizontal;
    }
    return pattern;
}

static std::vector<std::vector<uint8_t>> playerPatterns(
    Player const& player,
    int frames,
    WaveGuide const* waveGuide
) {
    std::vector<std::vector<uint8_t>> patterns;
    std::unordered_set<uint64_t> seen;

    auto add = [&](std::vector<uint8_t> pattern) {
        uint64_t key = hashBits(pattern);
        if (seen.insert(key).second)
            patterns.push_back(std::move(pattern));
    };

    auto constant = [frames](bool value) {
        return std::vector<uint8_t>(static_cast<size_t>(frames), static_cast<uint8_t>(value));
    };

    if (player.vehicle.type == VehicleType::Wave && waveGuide && !waveGuide->empty())
        add(guidedWavePattern(player, *waveGuide, frames));

    add(constant(player.button));
    add(constant(false));
    add(constant(true));

    if (player.vehicle.type == VehicleType::Wave) {
        std::vector<uint8_t> alternateA(static_cast<size_t>(frames));
        std::vector<uint8_t> alternateB(static_cast<size_t>(frames));
        for (int i = 0; i < frames; ++i) {
            alternateA[static_cast<size_t>(i)] = static_cast<uint8_t>((i & 1) != 0);
            alternateB[static_cast<size_t>(i)] = static_cast<uint8_t>((i & 1) == 0);
        }
        add(std::move(alternateA));
        add(std::move(alternateB));

        if (frames >= 3) {
            auto pulse = constant(false);
            pulse[0] = 1;
            add(std::move(pulse));
            auto latePulse = constant(false);
            latePulse[static_cast<size_t>(frames / 2)] = 1;
            add(std::move(latePulse));
        }
        return patterns;
    }

    if (continuousMode(player.vehicle.type)) {
        auto firstUp = constant(false);
        auto firstDown = constant(true);
        for (int i = 0; i < frames / 2; ++i) {
            firstUp[static_cast<size_t>(i)] = 1;
            firstDown[static_cast<size_t>(i)] = 0;
        }
        add(std::move(firstUp));
        add(std::move(firstDown));
        return patterns;
    }

    // Exact one-frame decision coverage is important for orbs and tight ground
    // timings. Longer holds are sampled more sparsely to keep the branch factor
    // bounded instead of sweeping every duration at every frame.
    for (int offset = 0; offset < frames; ++offset) {
        auto pulse = constant(false);
        pulse[static_cast<size_t>(offset)] = 1;
        add(std::move(pulse));
    }

    for (int offset : {0, std::max(1, frames / 2)}) {
        for (int duration : {2, player.vehicle.type == VehicleType::Robot ? 4 : 3}) {
            auto pulse = constant(false);
            int end = std::min(frames, offset + duration);
            for (int i = offset; i < end; ++i)
                pulse[static_cast<size_t>(i)] = 1;
            add(std::move(pulse));
        }
    }

    auto delayedHold = constant(false);
    for (int i = frames / 2; i < frames; ++i)
        delayedHold[static_cast<size_t>(i)] = 1;
    add(std::move(delayedHold));
    return patterns;
}

static std::vector<InputPattern> combinedPatterns(
    CompactSnapshot const& snapshot,
    int frames,
    WaveGuide const* p1Guide,
    WaveGuide const* p2Guide,
    std::mt19937& rng
) {
    auto p1Patterns = playerPatterns(snapshotP1(snapshot), frames, p1Guide);
    std::vector<InputPattern> result;

    if (!snapshotP1(snapshot).dualActive) {
        auto p2 = std::vector<uint8_t>(static_cast<size_t>(frames), snapshot.press2);
        result.reserve(p1Patterns.size());
        for (auto& p1 : p1Patterns)
            result.push_back({std::move(p1), p2});
    } else {
        auto p2Patterns = playerPatterns(snapshotP2(snapshot), frames, p2Guide);
        size_t p1Limit = std::min<size_t>(4, p1Patterns.size());
        size_t p2Limit = std::min<size_t>(4, p2Patterns.size());

        for (size_t i = 0; i < p1Limit; ++i) {
            for (size_t j = 0; j < p2Limit; ++j) {
                if (i == 0 || j == 0 || (i < 3 && j < 3))
                    result.push_back({p1Patterns[i], p2Patterns[j]});
            }
        }
    }

    std::shuffle(result.begin(), result.end(), rng);
    return result;
}

static bool crediblePlayerStep(float beforeX, Player const& after, Level2 const& lvl) {
    if (!std::isfinite(after.pos.x) || !std::isfinite(after.pos.y) ||
        !std::isfinite(after.velocity)) {
        return false;
    }

    constexpr float maxOrdinaryStep = 24.f;
    if (after.teleported) {
        if (!std::isfinite(after.teleportFromX) || !std::isfinite(after.teleportToX))
            return false;
        if (std::abs(after.teleportFromX - beforeX) > maxOrdinaryStep)
            return false;
        if (std::abs(after.pos.x - after.teleportToX) > 60.f)
            return false;

        // A uniquely resolved teleport may jump, but it still must land in the
        // inferred/trusted playable span. This keeps a bad target from becoming
        // permanent 90% progress even if later frames continue normally.
        float allowance = std::max(240.f, (lvl.length - lvl.gameStates.front().pos.x) * 0.05f);
        if (!after.completed && after.pos.x > lvl.length + allowance)
            return false;
    } else if (std::abs(after.pos.x - beforeX) > maxOrdinaryStep) {
        return false;
    }

    return after.pos.y <= std::max(1500.f, lvl.highestY + 650.f) && after.pos.y >= -650.f;
}

struct BeamState {
    CompactSnapshot snapshot;
    Trace trace;
    float clearance = 0.f;
    float minClearance = 999.f;
    float guideScore = 0.f;
    float maxX = 0.f;
    bool invalid = false;
    uint32_t tie = 0;
};

static TrialResult trialFromState(BeamState const& state, int horizonEnd) {
    auto const& player = snapshotP1(state.snapshot);
    TrialResult trial;
    trial.frame = player.frame;
    trial.x = player.pos.x;
    trial.y = player.pos.y;
    trial.velocity = player.velocity;
    trial.clearance = state.clearance;
    trial.dead = player.dead || state.invalid;
    trial.complete = !trial.dead && player.completed;
    trial.survivedHorizon = !trial.dead && !trial.complete && player.frame >= horizonEnd;
    trial.validProgress = !state.invalid;
    return trial;
}

static CandidateResult candidateFromState(BeamState const& state, int horizonEnd) {
    return {
        materializeTrace(state.trace),
        trialFromState(state, horizonEnd),
        hashSnapshot(state.snapshot)
    };
}

static bool betterBeam(BeamState const& a, BeamState const& b, int horizonEnd) {
    auto aTrial = trialFromState(a, horizonEnd);
    auto bTrial = trialFromState(b, horizonEnd);

    if (aTrial.complete != bTrial.complete)
        return aTrial.complete;
    if (aTrial.validProgress != bTrial.validProgress)
        return aTrial.validProgress;
    if (aTrial.dead != bTrial.dead)
        return !aTrial.dead;
    if (aTrial.frame != bTrial.frame)
        return aTrial.frame > bTrial.frame;

    if (std::abs(a.guideScore - b.guideScore) > 1.f)
        return a.guideScore > b.guideScore;
    if (std::abs(a.minClearance - b.minClearance) > 1.f)
        return a.minClearance > b.minClearance;
    if (std::abs(a.maxX - b.maxX) > 4.f)
        return a.maxX > b.maxX;
    if (std::abs(a.clearance - b.clearance) > 1.f)
        return a.clearance > b.clearance;
    return a.tie < b.tie;
}

static BeamState simulatePattern(
    Level2& lvl,
    BeamState const& parent,
    InputPattern const& pattern,
    int frames,
    WaveGuide const* p1Guide,
    SearchStats& stats,
    std::mt19937& rng
) {
    restoreSnapshot(lvl, parent.snapshot);
    InputList toggles;
    bool invalid = false;

    for (int offset = 0; offset < frames; ++offset) {
        int frame = lvl.currentFrame();
        bool desired1 = pattern.p1[static_cast<size_t>(offset)] != 0;
        bool desired2 = pattern.p2[static_cast<size_t>(offset)] != 0;

        if (lvl.press1 != desired1) {
            lvl.press1 = desired1;
            toggles.push_back(inputKey(static_cast<uint32_t>(frame), false));
        }
        if (lvl.press2 != desired2) {
            lvl.press2 = desired2;
            toggles.push_back(inputKey(static_cast<uint32_t>(frame), true));
        }

        float beforeX1 = lvl.latestState().pos.x;
        float beforeX2 = lvl.latestState2().pos.x;
        int beforeFrame = lvl.currentFrame();
        lvl.runFrame(lvl.press1, lvl.press2, 1.f / 240.f);
        ++stats.simulatedFrames;

        bool credible = crediblePlayerStep(beforeX1, lvl.latestState(), lvl);
        if (lvl.latestState().dualActive)
            credible = credible && crediblePlayerStep(beforeX2, lvl.latestState2(), lvl);

        if (!credible) {
            lvl.rollback(beforeFrame);
            lvl.syncPresses();
            invalid = true;
            ++stats.rejectedJumps;
            break;
        }
        if (lvl.latestState().dead || reachedGoal(lvl))
            break;
    }

    BeamState child;
    child.snapshot = captureSnapshot(lvl);
    child.trace = extendTrace(parent.trace, std::move(toggles));
    child.invalid = invalid;
    child.maxX = std::max(parent.maxX, snapshotP1(child.snapshot).pos.x);
    child.tie = rng();

    auto const& player = snapshotP1(child.snapshot);
    child.clearance = player.dead || invalid
        ? 0.f
        : player.vehicle.type == VehicleType::Wave
            ? waveClearance(lvl, player)
            : localClearance(lvl, player);
    child.minClearance = std::min(parent.minClearance, child.clearance);

    if (p1Guide && !p1Guide->empty() && player.vehicle.type == VehicleType::Wave) {
        float target = p1Guide->targetAt(player.pos.x + p1Guide->stepX * 1.5f);
        child.guideScore = -std::abs(player.pos.y - target);
    }
    return child;
}

static void keepTopStates(std::vector<BeamState>& states, size_t count, int horizonEnd) {
    auto comparator = [horizonEnd](BeamState const& a, BeamState const& b) {
        return betterBeam(a, b, horizonEnd);
    };

    if (states.size() > count) {
        std::nth_element(states.begin(), states.begin() + static_cast<long>(count), states.end(), comparator);
        states.resize(count);
    }
    std::sort(states.begin(), states.end(), comparator);
}

static SearchBatch searchBestInputs(
    Level2& lvl,
    std::atomic_bool& stop,
    std::mt19937& rng,
    int maximumHorizon,
    int difficulty
) {
    SearchBatch batch;
    CompactSimulationGuard guard(lvl);
    BeamState root;
    root.snapshot = guard.root;
    root.maxX = snapshotP1(root.snapshot).pos.x;
    root.minClearance = 999.f;
    root.tie = rng();

    int startFrame = snapshotP1(root.snapshot).frame;
    VehicleType rootMode = snapshotP1(root.snapshot).vehicle.type;
    int probeFrames = continuousMode(rootMode) ? 180 : 300;
    probeFrames = std::min(probeFrames, maximumHorizon);

    InputPattern noChange {
        std::vector<uint8_t>(static_cast<size_t>(probeFrames), root.snapshot.press1),
        std::vector<uint8_t>(static_cast<size_t>(probeFrames), root.snapshot.press2)
    };
    auto probe = simulatePattern(lvl, root, noChange, probeFrames, nullptr, batch.stats, rng);
    auto probeTrial = trialFromState(probe, startFrame + probeFrames);

    if (probeTrial.complete) {
        batch.candidates.push_back(candidateFromState(probe, startFrame + probeFrames));
        batch.stats.horizon = probeFrames;
        batch.stats.beamWidth = 1;
        return batch;
    }

    // Empty/easy ground is intentionally cheap. This is a real probe horizon,
    // not a short trial mixed into a longer elite pool.
    if (!continuousMode(rootMode) && probeTrial.survivedHorizon) {
        batch.candidates.push_back(candidateFromState(probe, startFrame + probeFrames));
        batch.stats.horizon = probeFrames;
        batch.stats.beamWidth = 1;
        return batch;
    }

    int horizon = maximumHorizon;
    if (probeTrial.dead && probeTrial.frame > startFrame) {
        int dangerOffset = probeTrial.frame - startFrame;
        horizon = std::min(horizon, std::max(260, dangerOffset + (continuousMode(rootMode) ? 260 : 220)));
    }

    WaveGuide p1Guide;
    WaveGuide p2Guide;
    if (rootMode == VehicleType::Wave)
        p1Guide = buildWaveGuide(lvl, snapshotP1(root.snapshot), horizon);
    if (snapshotP1(root.snapshot).dualActive && snapshotP2(root.snapshot).vehicle.type == VehicleType::Wave)
        p2Guide = buildWaveGuide(lvl, snapshotP2(root.snapshot), horizon);

    int step = rootMode == VehicleType::Wave
        ? (difficulty >= 2 ? 3 : 4)
        : continuousMode(rootMode)
            ? (difficulty >= 3 ? 5 : 6)
            : (difficulty >= 3 ? 4 : 6);
    int beamWidth = rootMode == VehicleType::Wave
        ? 34 + difficulty * 2
        : continuousMode(rootMode)
            ? 26 + difficulty * 2
            : 22 + difficulty * 2;
    beamWidth = std::min(44, beamWidth);

    batch.stats.horizon = horizon;
    batch.stats.beamWidth = beamWidth;
    int horizonEnd = startFrame + horizon;

    std::vector<BeamState> frontier {root};
    std::vector<BeamState> bestDeaths;
    std::vector<BeamState> completions;
    std::unordered_map<uint64_t, float> visited;
    visited.emplace(hashSnapshot(root.snapshot), root.minClearance);

    int maxDepth = (horizon + step - 1) / step;
    for (int depth = 0; depth < maxDepth && !frontier.empty() && !stop.load(); ++depth) {
        std::vector<BeamState> next;
        std::unordered_map<uint64_t, size_t> generationStates;

        for (auto const& state : frontier) {
            if (stop.load())
                break;

            int remaining = horizonEnd - snapshotP1(state.snapshot).frame;
            if (remaining <= 0) {
                next.push_back(state);
                continue;
            }

            int edgeFrames = std::min(step, remaining);
            auto patterns = combinedPatterns(
                state.snapshot,
                edgeFrames,
                p1Guide.empty() ? nullptr : &p1Guide,
                p2Guide.empty() ? nullptr : &p2Guide,
                rng
            );

            for (auto const& pattern : patterns) {
                if (stop.load())
                    break;
                ++batch.stats.expandedStates;

                auto child = simulatePattern(
                    lvl, state, pattern, edgeFrames,
                    p1Guide.empty() ? nullptr : &p1Guide,
                    batch.stats, rng
                );
                auto trial = trialFromState(child, horizonEnd);

                if (trial.complete) {
                    completions.push_back(std::move(child));
                    continue;
                }
                if (trial.dead || !trial.validProgress) {
                    if (trial.validProgress)
                        bestDeaths.push_back(std::move(child));
                    continue;
                }

                uint64_t key = hashSnapshot(child.snapshot);
                float quality = child.minClearance + child.guideScore * 0.25f;
                auto global = visited.find(key);
                if (global != visited.end() && global->second >= quality - 0.25f) {
                    ++batch.stats.deduplicatedStates;
                    continue;
                }
                visited[key] = quality;

                auto existing = generationStates.find(key);
                if (existing == generationStates.end()) {
                    generationStates.emplace(key, next.size());
                    next.push_back(std::move(child));
                } else if (betterBeam(child, next[existing->second], horizonEnd)) {
                    next[existing->second] = std::move(child);
                } else {
                    ++batch.stats.deduplicatedStates;
                }
            }
        }

        if (!completions.empty())
            break;

        keepTopStates(next, static_cast<size_t>(beamWidth), horizonEnd);
        if (bestDeaths.size() > static_cast<size_t>(beamWidth))
            keepTopStates(bestDeaths, static_cast<size_t>(beamWidth), horizonEnd);
        frontier = std::move(next);
    }

    std::vector<BeamState> endpoints;
    if (!completions.empty()) {
        keepTopStates(completions, 8, horizonEnd);
        endpoints = std::move(completions);
    } else {
        keepTopStates(frontier, 12, horizonEnd);
        endpoints = std::move(frontier);
        keepTopStates(bestDeaths, 12, horizonEnd);
        endpoints.insert(endpoints.end(), bestDeaths.begin(), bestDeaths.end());
    }

    for (auto const& state : endpoints) {
        auto candidate = candidateFromState(state, horizonEnd);
        bool duplicate = false;
        for (auto const& existing : batch.candidates) {
            if (sameStateBucket(existing, candidate)) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate)
            batch.candidates.push_back(std::move(candidate));
    }

    std::sort(batch.candidates.begin(), batch.candidates.end(), betterCandidate);
    if (batch.candidates.size() > 8)
        batch.candidates.resize(8);

    if (batch.candidates.empty()) {
        CandidateResult fallback;
        fallback.trial.frame = startFrame;
        fallback.trial.x = snapshotP1(root.snapshot).pos.x;
        fallback.trial.y = snapshotP1(root.snapshot).pos.y;
        fallback.trial.velocity = snapshotP1(root.snapshot).velocity;
        fallback.trial.dead = true;
        batch.candidates.push_back(std::move(fallback));
    }
    return batch;
}

struct ApplyResult {
    bool valid = true;
    float deathX = 0.f;
};

static ApplyResult applyInputsUntil(Level2& lvl, InputList const& inputs, int applyUntil) {
    ApplyResult result;
    auto cursor = std::lower_bound(inputs.begin(), inputs.end(), inputKey(lvl.currentFrame(), false));

    while (lvl.currentFrame() < applyUntil && !lvl.latestState().dead && !reachedGoal(lvl)) {
        uint32_t frame = static_cast<uint32_t>(lvl.currentFrame());
        while (cursor != inputs.end() && inputFrame(*cursor) == frame) {
            if (inputPlayer2(*cursor))
                lvl.press2 = !lvl.press2;
            else
                lvl.press1 = !lvl.press1;
            ++cursor;
        }

        float beforeX1 = lvl.latestState().pos.x;
        float beforeX2 = lvl.latestState2().pos.x;
        int beforeFrame = lvl.currentFrame();
        lvl.runFrame(lvl.press1, lvl.press2, 1.f / 240.f);

        bool credible = crediblePlayerStep(beforeX1, lvl.latestState(), lvl);
        if (lvl.latestState().dualActive)
            credible = credible && crediblePlayerStep(beforeX2, lvl.latestState2(), lvl);
        if (!credible) {
            lvl.rollback(beforeFrame);
            lvl.syncPresses();
            result.valid = false;
            result.deathX = beforeX1;
            return result;
        }
    }

    result.deathX = lvl.latestState().pos.x;
    return result;
}

struct BestTimeline {
    std::vector<Player> p1;
    std::vector<Player> p2;
    bool press1 = false;
    bool press2 = false;

    explicit BestTimeline(Level2 const& lvl)
        : p1(lvl.gameStates), p2(lvl.gameStates2), press1(lvl.press1), press2(lvl.press2) {}

    int frame() const { return p1.empty() ? 0 : p1.back().frame; }
    float x() const { return p1.empty() ? 0.f : p1.back().pos.x; }

    void capture(Level2 const& lvl) {
        p1 = lvl.gameStates;
        p2 = lvl.gameStates2;
        press1 = lvl.press1;
        press2 = lvl.press2;
    }

    void appendFrom(Level2 const& lvl) {
        if (lvl.gameStates.size() < p1.size() ||
            (!p1.empty() && lvl.gameStates[p1.size() - 1].frame != p1.back().frame)) {
            capture(lvl);
            return;
        }

        p1.insert(p1.end(), lvl.gameStates.begin() + static_cast<long>(p1.size()), lvl.gameStates.end());
        p2.insert(p2.end(), lvl.gameStates2.begin() + static_cast<long>(p2.size()), lvl.gameStates2.end());
        press1 = lvl.press1;
        press2 = lvl.press2;
    }

    void restore(Level2& lvl) const {
        lvl.gameStates = p1;
        lvl.gameStates2 = p2;
        lvl.press1 = press1;
        lvl.press2 = press2;
        lvl.fixStatePointers();
    }
};

static uint64_t hashInputs(InputList const& inputs) {
    uint64_t hash = 0xcbf29ce484222325ull;
    for (SearchInput input : inputs)
        hashCombine(hash, input);
    return hash;
}

struct AlternativeBranch {
    int generation = 0;
    int baseFrame = 0;
    int applyUntil = 0;
    float baseX = 0.f;
    uint64_t inputHash = 0;
    InputList inputs;
};

static int safePrefixFrames(CandidateResult const& candidate, int baseFrame, VehicleType mode) {
    int span = std::max(0, candidate.trial.frame - baseFrame);
    if (candidate.trial.complete)
        return span;

    if (candidate.trial.dead) {
        int margin = mode == VehicleType::Wave ? 90 : continuousMode(mode) ? 75 : 60;
        int safe = std::max(0, span - margin);
        safe = std::min(
            safe,
            span * (mode == VehicleType::Wave ? 34 : continuousMode(mode) ? 42 : 50) / 100
        );
        return safe;
    }

    int percent = mode == VehicleType::Wave ? 34 : continuousMode(mode) ? 44 : 64;
    return span * percent / 100;
}

static char const* unsupportedMechanicName(int id) {
    switch (id) {
        case 901: return "Move";
        case 1049: return "Toggle";
        case 1268: return "Spawn";
        case 1346: return "Rotate";
        case 1347: return "Follow";
        case 1595: return "Touch";
        case 1616: return "Stop";
        case 1814: return "FollowPlayerY";
        case 1815: return "Collision";
        case 2067: return "Scale";
        case 2068: return "AdvancedRandom";
        case 3032: return "Keyframe";
        case 3607: return "Sequence";
        default: return nullptr;
    }
}

PathfinderResult pathfind(
    std::string const& lvlString,
    std::atomic_bool& stop,
    std::function<void(PathfinderTelemetry const&)> callback,
    float trustedEndX
) {
    Level2 lvl(lvlString);
    float solveStartX = lvl.latestState().pos.x;
    float simulatorInferredLength = lvl.inferredLength;
    bool hasTrustedEnd = std::isfinite(trustedEndX) && trustedEndX > solveStartX + 30.f;
    if (hasTrustedEnd) {
        lvl.length = trustedEndX;
        lvl.lengthSource = "trusted-gd";
    }

    auto progressFor = [&](float x, bool complete) -> double {
        if (complete)
            return 100.0;
        double span = static_cast<double>(lvl.length - solveStartX);
        if (span <= 1.0)
            return 0.0;
        return std::clamp(
            ((static_cast<double>(x) - solveStartX) / span) * 100.0,
            0.0,
            99.99
        );
    };

    uint32_t seed = static_cast<uint32_t>(std::hash<std::string>{}(lvlString));
    seed ^= static_cast<uint32_t>(std::llround(std::max(0.f, trustedEndX)));
    std::random_device randomDevice;
    seed ^= randomDevice();
    std::mt19937 rng(seed);

    BestTimeline best(lvl);
    std::deque<AlternativeBranch> alternatives;
    int bestGeneration = 0;
    bool currentExtendsBest = true;
    float furthestX = solveStartX;
    float lastDeathX = solveStartX;
    int recoveryLevel = 0;
    int repeatedDeathZone = 0;
    int stagnantRounds = 0;
    std::string lastRecoveryReason = "start";
    std::unordered_map<uint64_t, int> failedRoots;
    bool solverExhausted = false;
    SearchStats totals;

    auto mergeStats = [&totals](SearchStats const& stats) {
        totals.expandedStates += stats.expandedStates;
        totals.simulatedFrames += stats.simulatedFrames;
        totals.deduplicatedStates += stats.deduplicatedStates;
        totals.rejectedJumps += stats.rejectedJumps;
        totals.horizon = stats.horizon;
        totals.beamWidth = stats.beamWidth;
    };

    auto emitTelemetry = [&](std::string reason, float deathX = 0.f) {
        if (!callback)
            return;
        PathfinderTelemetry telemetry;
        telemetry.progress = progressFor(furthestX, reachedGoal(lvl));
        telemetry.startX = solveStartX;
        telemetry.currentX = lvl.latestState().pos.x;
        telemetry.furthestX = furthestX;
        telemetry.trustedEndX = hasTrustedEnd ? trustedEndX : 0.f;
        telemetry.inferredLength = simulatorInferredLength;
        telemetry.frame = lvl.currentFrame();
        telemetry.mode = vehicleName(lvl.latestState().vehicle.type);
        telemetry.checkpointFrame = best.frame();
        telemetry.checkpointX = best.x();
        telemetry.deathX = deathX;
        telemetry.recoveryReason = std::move(reason);
        callback(telemetry);
    };

    auto recover = [&](std::string reason, int extraRetreat) {
        // Try a retained sibling from an earlier beam before erasing more of the
        // best route. These branches share the exact committed base checkpoint.
        while (!alternatives.empty()) {
            AlternativeBranch alternative = std::move(alternatives.back());
            alternatives.pop_back();
            if (alternative.generation != bestGeneration ||
                alternative.baseFrame >= best.frame() ||
                alternative.applyUntil <= alternative.baseFrame) {
                continue;
            }

            best.restore(lvl);
            lvl.rollback(alternative.baseFrame);
            lvl.syncPresses();
            auto applied = applyInputsUntil(lvl, alternative.inputs, alternative.applyUntil);
            if (applied.valid && !lvl.latestState().dead &&
                lvl.currentFrame() > alternative.baseFrame + 8) {
                currentExtendsBest = false;
                recoveryLevel = std::min(10, recoveryLevel + 1);
                repeatedDeathZone = 0;
                stagnantRounds = 0;
                lastRecoveryReason = reason + ":alternate-branch";
                emitTelemetry(lastRecoveryReason, lastDeathX);
                return;
            }
        }

        best.restore(lvl);
        int available = std::max(0, best.frame() - 1);
        int retreat = std::min(
            available,
            150 + recoveryLevel * 140 + stagnantRounds * 70 + extraRetreat
        );
        lvl.rollback(std::max(1, best.frame() - retreat));
        lvl.syncPresses();
        currentExtendsBest = lvl.currentFrame() == best.frame();
        recoveryLevel = std::min(10, recoveryLevel + 1);
        repeatedDeathZone = 0;
        stagnantRounds = 0;
        lastRecoveryReason = reason + ":retreat";
        rng.seed(seed ^ static_cast<uint32_t>(recoveryLevel * 0x45d9f3bu) ^
                 static_cast<uint32_t>(lvl.currentFrame() * 7919));
        emitTelemetry(lastRecoveryReason, lastDeathX);
    };

    while (!reachedGoal(lvl) && !stop.load()) {
        if (lvl.latestState().dead) {
            recover("committed-death", 240);
            continue;
        }

        int frame = lvl.currentFrame();
        float rootX = lvl.latestState().pos.x;
        VehicleType mode = lvl.latestState().vehicle.type;
        bool rootIsBest = currentExtendsBest && frame == best.frame();
        int difficulty = std::min(5, recoveryLevel / 2 + repeatedDeathZone + stagnantRounds);

        // Adaptive maximum only; searchBestInputs shortens this again around the
        // first observed danger and fast-paths empty ground.
        int maximumHorizon = continuousMode(mode) ? 660 : 540;
        maximumHorizon += difficulty * (continuousMode(mode) ? 45 : 35);

        auto batch = searchBestInputs(lvl, stop, rng, maximumHorizon, difficulty);
        mergeStats(batch.stats);
        if (stop.load())
            break;

        auto const& chosen = batch.candidates.front();

        // If nearly every explored branch is rejected by the trusted-endpoint X
        // validator at the initial checkpoint, more random ordering cannot fix the
        // level data contradiction. Return diagnostics instead of looping forever.
        if (best.frame() <= 1 && frame <= 1 && batch.stats.expandedStates > 50 &&
            batch.stats.rejectedJumps * 3 > batch.stats.expandedStates) {
            lastRecoveryReason = "unrecoverable-invalid-x-jump";
            solverExhausted = true;
            break;
        }

        // Preserve several genuinely different continuations from this exact best
        // checkpoint. A later stall can resume one without rediscovering the prefix.
        if (rootIsBest) {
            for (size_t i = 1; i < batch.candidates.size() && i < 6; ++i) {
                auto const& candidate = batch.candidates[i];
                int safeFrames = safePrefixFrames(candidate, frame, mode);
                if (safeFrames < 18)
                    continue;

                AlternativeBranch alternative;
                alternative.generation = bestGeneration;
                alternative.baseFrame = frame;
                alternative.baseX = rootX;
                alternative.applyUntil = frame + std::min(safeFrames, 300);
                alternative.inputs = candidate.inputs;
                alternative.inputHash = hashInputs(alternative.inputs);

                bool duplicate = std::any_of(
                    alternatives.begin(), alternatives.end(),
                    [&](AlternativeBranch const& existing) {
                        return existing.generation == alternative.generation &&
                               existing.baseFrame == alternative.baseFrame &&
                               existing.inputHash == alternative.inputHash;
                    }
                );
                if (!duplicate)
                    alternatives.push_back(std::move(alternative));
            }
            while (alternatives.size() > 24)
                alternatives.pop_front();
        }

        int commitFrames = 0;
        if (chosen.trial.complete) {
            commitFrames = std::max(0, chosen.trial.frame - frame);
        } else if (chosen.trial.dead) {
            if (std::abs(chosen.trial.x - lastDeathX) <= 40.f)
                ++repeatedDeathZone;
            else
                repeatedDeathZone = 1;
            lastDeathX = chosen.trial.x;

            commitFrames = safePrefixFrames(chosen, frame, mode);
            float deathGain = chosen.trial.x - rootX;
            if (commitFrames < 18 || (deathGain < 20.f && lvl.latestState().direction >= 0)) {
                uint64_t rootHash = hashSnapshot(captureSnapshot(lvl));
                int failures = ++failedRoots[rootHash];
                if (failures >= 6 && alternatives.empty()) {
                    lastRecoveryReason = "unrecoverable-repeated-state";
                    solverExhausted = true;
                    break;
                }
                recover(
                    repeatedDeathZone >= 2 ? "repeated-death-zone" : "no-safe-prefix",
                    repeatedDeathZone >= 2 ? 360 : 160
                );
                continue;
            }
        } else {
            int span = std::max(0, chosen.trial.frame - frame);
            commitFrames = safePrefixFrames(chosen, frame, mode);
            if (chosen.trial.survivedHorizon && chosen.inputs.empty() && !continuousMode(mode))
                commitFrames = span * 78 / 100;
            if (commitFrames <= 0) {
                recover("zero-advance", 220);
                continue;
            }
        }

        int cap = mode == VehicleType::Wave ? 190 : continuousMode(mode) ? 250 : 360;
        if (!chosen.trial.complete)
            commitFrames = std::min(commitFrames, cap);

        auto applied = applyInputsUntil(lvl, chosen.inputs, frame + commitFrames);
        if (!applied.valid) {
            ++totals.rejectedJumps;
            lastDeathX = applied.deathX;
            recover("invalid-x-jump", 360);
            continue;
        }
        if (lvl.currentFrame() <= frame) {
            ++stagnantRounds;
            recover("no-frame-advance", 240);
            continue;
        }
        if (lvl.latestState().dead) {
            lastDeathX = lvl.latestState().pos.x;
            recover("prefix-died", 280);
            continue;
        }

        float previousFurthest = furthestX;
        furthestX = std::max(furthestX, lvl.latestState().pos.x);
        bool newFurthest = furthestX > previousFurthest + 1.f;

        if (currentExtendsBest) {
            best.appendFrom(lvl);
        } else if (newFurthest || reachedGoal(lvl)) {
            best.capture(lvl);
            currentExtendsBest = true;
            ++bestGeneration;
            alternatives.clear();
        }

        if (newFurthest) {
            stagnantRounds = 0;
            repeatedDeathZone = 0;
            recoveryLevel = std::max(0, recoveryLevel - 1);
            lastRecoveryReason = "advance";
        } else if (lvl.latestState().direction < 0) {
            // Reverse sections intentionally lower X. Frame survival and trigger
            // state remain valid even though the progress bar cannot increase.
            stagnantRounds = 0;
            lastRecoveryReason = "reverse-traversal";
        } else {
            ++stagnantRounds;
            lastRecoveryReason = "replaying-checkpoint";
        }

        int stallLimit = currentExtendsBest ? 4 : 10;
        if (stagnantRounds >= stallLimit && best.frame() > 1) {
            recover("x-stagnation", 300);
            continue;
        }

        emitTelemetry(lastRecoveryReason, chosen.trial.dead ? chosen.trial.x : 0.f);
    }

    if (!lvl.latestState().dead && reachedGoal(lvl)) {
        best.capture(lvl);
        currentExtendsBest = true;
        furthestX = std::max(furthestX, lvl.latestState().pos.x);
    } else if (currentExtendsBest && lvl.currentFrame() > best.frame()) {
        best.appendFrom(lvl);
    }

    PathfinderResult result;
    Replay2 output;
    for (size_t i = 1; i < best.p1.size(); ++i) {
        auto const& p1 = best.p1[i];
        auto const& previousP1 = best.p1[i - 1];
        if (p1.frame > 1 && p1.button != previousP1.button) {
            output.inputs.push_back(gdr::Input(p1.frame, 1, false, p1.button));
            result.inputs.push_back({static_cast<uint32_t>(p1.frame), false, p1.button});
        }

        if (i < best.p2.size()) {
            auto const& p2 = best.p2[i];
            auto const& previousP2 = best.p2[i - 1];
            if (p2.dualActive && p2.frame > 1 && p2.button != previousP2.button) {
                output.inputs.push_back(gdr::Input(p2.frame, 1, true, p2.button));
                result.inputs.push_back({static_cast<uint32_t>(p2.frame), true, p2.button});
            }
        }
    }

    std::sort(result.inputs.begin(), result.inputs.end(), [](auto const& a, auto const& b) {
        if (a.frame != b.frame)
            return a.frame < b.frame;
        return a.player2 < b.player2;
    });

    result.macro = output.exportData().unwrapOr({});
    result.complete = !best.p1.empty() && !best.p1.back().dead && best.p1.back().completed;
    result.progress = progressFor(furthestX, result.complete);

    std::map<int, size_t> unsupportedGameplay;
    for (auto const& object : lvl.unsupportedObjects) {
        if (unsupportedMechanicName(object.objectID))
            ++unsupportedGameplay[object.objectID];
    }

    std::ostringstream diagnostics;
    diagnostics << std::fixed << std::setprecision(2)
                << "startX=" << solveStartX
                << " currentX=" << (best.p1.empty() ? solveStartX : best.p1.back().pos.x)
                << " furthestX=" << furthestX
                << " trustedEndX=" << (hasTrustedEnd ? trustedEndX : 0.f)
                << " inferredLength=" << simulatorInferredLength
                << " endpointSource=" << lvl.lengthSource
                << " frame=" << (best.p1.empty() ? 0 : best.p1.back().frame)
                << " mode=" << (best.p1.empty() ? "Unknown" : vehicleName(best.p1.back().vehicle.type))
                << " checkpointFrame=" << best.frame()
                << " checkpointX=" << best.x()
                << " deathX=" << lastDeathX
                << " recovery=" << lastRecoveryReason
                << " exhausted=" << (solverExhausted ? 1 : 0)
                << " expanded=" << totals.expandedStates
                << " simulatedFrames=" << totals.simulatedFrames
                << " deduplicated=" << totals.deduplicatedStates
                << " rejectedJumps=" << totals.rejectedJumps
                << " lastHorizon=" << totals.horizon
                << " lastBeam=" << totals.beamWidth;

    if (!unsupportedGameplay.empty()) {
        diagnostics << " unsupportedGameplay=";
        bool first = true;
        for (auto const& [id, count] : unsupportedGameplay) {
            if (!first)
                diagnostics << ',';
            first = false;
            diagnostics << unsupportedMechanicName(id) << '(' << id << ")x" << count;
        }
    }
    result.diagnostics = diagnostics.str();
    return result;
}
