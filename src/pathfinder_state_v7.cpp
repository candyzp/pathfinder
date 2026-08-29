#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <random>
#include <set>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Orb.hpp>

#define pathfind pathfind_legacy_v7
#include "pathfinder.cpp"
#undef pathfind

namespace {

constexpr int kLogicalHelpersV7 = 100;
constexpr int kPhysicalThreadsV7 = 30;
constexpr int kGuidedSlotsV7 = 50;
constexpr int kExplorerSlotsV7 = 50;
constexpr int kArchiveLimitV7 = 320;

struct SnapshotV7 {
    Player p1;
    Player p2;
    bool press1 = false;
    bool press2 = false;
    std::vector<int> moveActivationFrames;
};

struct RelativeToggleV7 {
    int offset = 0;
    bool player2 = false;
};

struct MacroActionV7 {
    int duration = 24;
    std::vector<RelativeToggleV7> toggles;
    uint32_t signature = 0;
};

struct SearchNodeV7 {
    SnapshotV7 snapshot;
    std::vector<SearchInput> route;
    float score = -std::numeric_limits<float>::infinity();
    float minClearance = 0.f;
    float x = 0.f;
    float y = 0.f;
    double velocity = 0.0;
    bool complete = false;
    bool explorer = false;
    int depth = 0;
};

struct SimResultV7 {
    SearchNodeV7 node;
    float deathX = 0.f;
    bool dead = false;
};

struct StateKeyV7 {
    int x = 0;
    int y = 0;
    int velocity = 0;
    int frame = 0;
    int p2y = 0;
    int p2velocity = 0;
    uint32_t flags = 0;
    uint64_t triggerHash = 0;

    bool operator==(StateKeyV7 const&) const = default;
};

struct StateKeyHashV7 {
    size_t operator()(StateKeyV7 const& key) const noexcept {
        uint64_t h = 1469598103934665603ull;
        auto mix = [&](uint64_t value) {
            h ^= value + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
            h *= 1099511628211ull;
        };
        mix(static_cast<uint32_t>(key.x));
        mix(static_cast<uint32_t>(key.y));
        mix(static_cast<uint32_t>(key.velocity));
        mix(static_cast<uint32_t>(key.frame));
        mix(static_cast<uint32_t>(key.p2y));
        mix(static_cast<uint32_t>(key.p2velocity));
        mix(key.flags);
        mix(key.triggerHash);
        return static_cast<size_t>(h);
    }
};

struct CoarseKeyV7 {
    int x = 0;
    int y = 0;
    int velocity = 0;
    uint32_t flags = 0;

    bool operator==(CoarseKeyV7 const&) const = default;
};

struct CoarseKeyHashV7 {
    size_t operator()(CoarseKeyV7 const& key) const noexcept {
        size_t h = static_cast<size_t>(key.x * 73856093);
        h ^= static_cast<size_t>(key.y * 19349663);
        h ^= static_cast<size_t>(key.velocity * 83492791);
        h ^= static_cast<size_t>(key.flags * 2654435761u);
        return h;
    }
};

struct UpcomingOrbV7 {
    bool found = false;
    OrbType type = OrbType::Yellow;
    float x = 0.f;
    float entryX = 0.f;
    int contactOffset = -1;
};

int pruneNoTouchPhysicsV7(Level2& lvl, std::string const& lvlString) {
    std::unordered_set<int> noTouchIDs;
    std::stringstream stream(lvlString);
    std::string objectString;
    bool first = true;
    int supportedIndex = 0;

    while (std::getline(stream, objectString, ';')) {
        if (first) {
            first = false;
            continue;
        }

        ObjectFields fields = parseFields(objectString);
        int objectID = intField(fields, 1);
        if (objectID <= 0 || objectID == 31)
            continue;

        ObjectFields probe = fields;
        if (Object::create(std::move(probe))) {
            if (boolField(fields, 121))
                noTouchIDs.insert(supportedIndex);
            ++supportedIndex;
        }
    }

    if (!noTouchIDs.empty()) {
        for (auto& [_, section] : lvl.sections) {
            section.erase(
                std::remove_if(
                    section.begin(),
                    section.end(),
                    [&](ObjectContainer const& object) {
                        return noTouchIDs.contains(object->id);
                    }
                ),
                section.end()
            );
        }

        for (int id : noTouchIDs) {
            lvl.baseObjectPositions.erase(id);
            lvl.objectSections.erase(id);
            lvl.movingObjectIDs.erase(id);
        }

        for (auto& [_, members] : lvl.groupObjects) {
            members.erase(
                std::remove_if(
                    members.begin(),
                    members.end(),
                    [&](int id) { return noTouchIDs.contains(id); }
                ),
                members.end()
            );
        }

        lvl.highestY = 0.f;
        for (auto const& [_, section] : lvl.sections) {
            for (auto const& object : section)
                lvl.highestY = std::max(lvl.highestY, object->pos.y);
        }
    }

    lvl.unsupportedObjects.clear();
    lvl.unsupportedObjects.shrink_to_fit();
    return static_cast<int>(noTouchIDs.size());
}

Level2 compactWorkerV7(Level2 const& source) {
    Level2 worker = source;
    Player p1 = source.latestState();
    Player p2 = source.latestState2();
    worker.gameStates.assign(1, p1);
    worker.gameStates2.assign(1, p2);
    worker.gameStates.front().level = &worker;
    worker.gameStates2.front().level = &worker;
    worker.press1 = source.press1;
    worker.press2 = source.press2;
    worker.unsupportedObjects.clear();
    return worker;
}

SnapshotV7 captureSnapshotV7(Level2 const& level) {
    SnapshotV7 snapshot;
    snapshot.p1 = level.latestState();
    snapshot.p2 = level.latestState2();
    snapshot.press1 = level.press1;
    snapshot.press2 = level.press2;
    snapshot.moveActivationFrames.reserve(level.moveTriggers.size());
    for (auto const& trigger : level.moveTriggers)
        snapshot.moveActivationFrames.push_back(trigger.activationFrame);
    return snapshot;
}

void restoreSnapshotV7(Level2& level, SnapshotV7 const& snapshot) {
    level.gameStates.resize(1);
    level.gameStates2.resize(1);
    level.gameStates.front() = snapshot.p1;
    level.gameStates2.front() = snapshot.p2;
    level.gameStates.front().level = &level;
    level.gameStates2.front().level = &level;
    level.press1 = snapshot.press1;
    level.press2 = snapshot.press2;

    size_t count = std::min(
        level.moveTriggers.size(),
        snapshot.moveActivationFrames.size()
    );
    for (size_t i = 0; i < count; ++i)
        level.moveTriggers[i].activationFrame = snapshot.moveActivationFrames[i];

    level.applyMoveGeometry(snapshot.p1.frame);
}

uint64_t triggerHashV7(SnapshotV7 const& snapshot) {
    uint64_t h = 1469598103934665603ull;
    int frame = snapshot.p1.frame;
    for (int activation : snapshot.moveActivationFrames) {
        int relative = activation < 0 ? -1 : std::clamp(frame - activation, 0, 4095);
        h ^= static_cast<uint64_t>(static_cast<uint32_t>(relative + 1));
        h *= 1099511628211ull;
    }
    return h;
}

uint32_t stateFlagsV7(Player const& p1, Player const& p2, bool press1, bool press2) {
    uint32_t flags = static_cast<uint32_t>(p1.vehicle.type) & 0xFu;
    flags |= (static_cast<uint32_t>(std::clamp(p1.speed, 0, 7)) & 0x7u) << 4;
    flags |= static_cast<uint32_t>(p1.upsideDown) << 7;
    flags |= static_cast<uint32_t>(p1.small) << 8;
    flags |= static_cast<uint32_t>(p1.grounded) << 9;
    flags |= static_cast<uint32_t>(press1) << 10;
    flags |= static_cast<uint32_t>(p1.dashing) << 11;
    flags |= static_cast<uint32_t>(p1.dualActive) << 12;
    flags |= static_cast<uint32_t>(press2) << 13;
    if (p1.dualActive) {
        flags |= static_cast<uint32_t>(p2.upsideDown) << 14;
        flags |= static_cast<uint32_t>(p2.small) << 15;
        flags |= (static_cast<uint32_t>(p2.vehicle.type) & 0xFu) << 16;
    }
    return flags;
}

StateKeyV7 stateKeyV7(SnapshotV7 const& snapshot, int precisionLevel) {
    Player const& p1 = snapshot.p1;
    Player const& p2 = snapshot.p2;
    float xyQuantum = precisionLevel >= 3 ? 3.f : precisionLevel >= 1 ? 5.f : 8.f;
    float velocityQuantum = precisionLevel >= 3 ? 18.f : precisionLevel >= 1 ? 28.f : 45.f;
    int frameQuantum = precisionLevel >= 3 ? 1 : precisionLevel >= 1 ? 2 : 3;

    StateKeyV7 key;
    key.x = static_cast<int>(std::lround(p1.pos.x / xyQuantum));
    key.y = static_cast<int>(std::lround(p1.pos.y / xyQuantum));
    key.velocity = static_cast<int>(std::lround(p1.velocity / velocityQuantum));
    key.frame = p1.frame / frameQuantum;
    key.flags = stateFlagsV7(p1, p2, snapshot.press1, snapshot.press2);
    if (p1.dualActive) {
        key.p2y = static_cast<int>(std::lround(p2.pos.y / xyQuantum));
        key.p2velocity = static_cast<int>(std::lround(p2.velocity / velocityQuantum));
    }
    key.triggerHash = triggerHashV7(snapshot);
    return key;
}

CoarseKeyV7 coarseKeyV7(SearchNodeV7 const& node) {
    Player const& p1 = node.snapshot.p1;
    CoarseKeyV7 key;
    key.x = static_cast<int>(std::floor(p1.pos.x / 80.f));
    key.y = static_cast<int>(std::floor(p1.pos.y / 45.f));
    key.velocity = static_cast<int>(std::floor(p1.velocity / 120.f));
    key.flags = stateFlagsV7(
        p1,
        node.snapshot.p2,
        node.snapshot.press1,
        node.snapshot.press2
    ) & 0x1FFFFu;
    return key;
}

int baseSegmentFramesV7(VehicleType mode, int precisionLevel, bool explorer) {
    int base = 30;
    switch (mode) {
        case VehicleType::Ship: base = 20; break;
        case VehicleType::Ball: base = 22; break;
        case VehicleType::Ufo: base = 18; break;
        case VehicleType::Wave: base = 14; break;
        case VehicleType::Robot: base = 28; break;
        case VehicleType::Spider: base = 16; break;
        case VehicleType::Swing: base = 18; break;
        default: base = 30; break;
    }

    base -= std::min(precisionLevel, 4) * (flightMode(mode) ? 2 : 3);
    base = std::max(base, flightMode(mode) ? 8 : 10);
    if (explorer)
        base = std::min(42, base + 6);
    return base;
}

int estimateOffsetForXV7(
    Player const& player,
    float targetX,
    int duration
) {
    if (player.direction <= 0)
        return -1;
    int speedIndex = std::clamp(player.speed, 0, 4);
    double scale = std::clamp(static_cast<double>(player.timeWarp), 0.05, 4.0);
    double xPerSecond = player_speeds[speedIndex] * scale;
    if (xPerSecond < 1.0)
        return -1;
    double dx = static_cast<double>(targetX - player.pos.x);
    int offset = static_cast<int>(std::lround(dx * 240.0 / xPerSecond));
    return offset >= 0 && offset < duration ? offset : -1;
}

UpcomingOrbV7 upcomingOrbV7(Level2 const& level, int duration) {
    UpcomingOrbV7 result;
    Player const& player = level.latestState();
    if (player.direction <= 0 || player.dashing || flightMode(player.vehicle.type))
        return result;

    int speedIndex = std::clamp(player.speed, 0, 4);
    double scale = std::clamp(static_cast<double>(player.timeWarp), 0.05, 4.0);
    double travel = player_speeds[speedIndex] * scale * duration / 240.0;
    float lowX = player.pos.x - 25.f;
    float highX = player.pos.x + static_cast<float>(travel) + 70.f;
    int firstSection = static_cast<int>(std::floor(lowX / Level::sectionSize));
    int lastSection = static_cast<int>(std::floor(highX / Level::sectionSize));
    float nearest = std::numeric_limits<float>::max();

    for (int section = firstSection; section <= lastSection; ++section) {
        auto it = level.sections.find(section);
        if (it == level.sections.end())
            continue;
        for (auto const& object : it->second) {
            auto const* orb = dynamic_cast<Orb const*>(object.operator->());
            if (!orb)
                continue;
            OrbType type = orb->type;
            bool supported =
                type == OrbType::Yellow || type == OrbType::Blue ||
                type == OrbType::Pink || type == OrbType::Red ||
                type == OrbType::Green || type == OrbType::Black ||
                type == OrbType::Dash || type == OrbType::GravityDash ||
                type == OrbType::Spider || type == OrbType::Teleport;
            if (!supported)
                continue;

            float dx = orb->pos.x - player.pos.x;
            if (dx < -25.f || dx > static_cast<float>(travel) + 70.f)
                continue;
            if (dx < nearest) {
                nearest = dx;
                result.found = true;
                result.type = type;
                result.x = orb->pos.x;
                result.entryX = orb->getLeft() - player.size.x * 0.5f;
            }
        }
    }

    if (result.found) {
        int entry = estimateOffsetForXV7(player, result.entryX, duration);
        int center = estimateOffsetForXV7(player, result.x, duration);
        result.contactOffset = entry >= 0 ? entry : center;
    }
    return result;
}

void normalizeActionV7(MacroActionV7& action) {
    std::sort(
        action.toggles.begin(),
        action.toggles.end(),
        [](RelativeToggleV7 const& a, RelativeToggleV7 const& b) {
            if (a.offset != b.offset)
                return a.offset < b.offset;
            return a.player2 < b.player2;
        }
    );
    action.toggles.erase(
        std::unique(
            action.toggles.begin(),
            action.toggles.end(),
            [](RelativeToggleV7 const& a, RelativeToggleV7 const& b) {
                return a.offset == b.offset && a.player2 == b.player2;
            }
        ),
        action.toggles.end()
    );

    uint32_t h = 2166136261u;
    h ^= static_cast<uint32_t>(action.duration);
    h *= 16777619u;
    for (auto const& toggle : action.toggles) {
        h ^= static_cast<uint32_t>((toggle.offset << 1) | (toggle.player2 ? 1 : 0));
        h *= 16777619u;
    }
    action.signature = h;
}

MacroActionV7 makeActionV7(
    int duration,
    bool initialP1,
    bool initialP2,
    std::vector<std::pair<int, bool>> p1States,
    std::vector<std::pair<int, bool>> p2States = {}
) {
    MacroActionV7 action;
    action.duration = duration;

    auto append = [&](bool initial, bool player2, std::vector<std::pair<int, bool>> states) {
        std::sort(states.begin(), states.end());
        bool held = initial;
        int lastOffset = -1;
        bool lastDesired = held;
        for (auto [offset, desired] : states) {
            offset = std::clamp(offset, 0, std::max(0, duration - 1));
            if (offset == lastOffset) {
                lastDesired = desired;
                continue;
            }
            if (lastOffset >= 0 && lastDesired != held) {
                action.toggles.push_back({lastOffset, player2});
                held = lastDesired;
            }
            lastOffset = offset;
            lastDesired = desired;
        }
        if (lastOffset >= 0 && lastDesired != held)
            action.toggles.push_back({lastOffset, player2});
    };

    append(initialP1, false, std::move(p1States));
    append(initialP2, true, std::move(p2States));
    normalizeActionV7(action);
    return action;
}

void addUniqueActionV7(
    std::vector<MacroActionV7>& actions,
    MacroActionV7 action,
    std::unordered_set<uint32_t>& signatures
) {
    if (signatures.insert(action.signature).second)
        actions.push_back(std::move(action));
}

std::vector<MacroActionV7> guidedActionsV7(
    Level2& probe,
    SnapshotV7 const& snapshot,
    int precisionLevel
) {
    restoreSnapshotV7(probe, snapshot);
    Player const& p1 = probe.latestState();
    Player const& p2 = probe.latestState2();
    VehicleType mode = p1.vehicle.type;
    int duration = baseSegmentFramesV7(mode, precisionLevel, false);
    bool initial1 = snapshot.press1;
    bool initial2 = snapshot.press2;

    std::vector<MacroActionV7> actions;
    actions.reserve(36);
    std::unordered_set<uint32_t> signatures;

    auto addP1 = [&](std::vector<std::pair<int, bool>> states) {
        addUniqueActionV7(
            actions,
            makeActionV7(duration, initial1, initial2, std::move(states)),
            signatures
        );
    };

    addP1({});
    addP1({{0, false}});
    addP1({{0, true}});

    if (flightMode(mode)) {
        for (int split : {duration / 4, duration / 2, (duration * 3) / 4}) {
            addP1({{0, false}, {split, true}});
            addP1({{0, true}, {split, false}});
        }
        for (int start : {0, duration / 4, duration / 2}) {
            int width = std::max(2, duration / 3);
            addP1({{0, false}, {start, true}, {std::min(duration - 1, start + width), false}});
        }
    } else {
        int step = precisionLevel >= 4 ? 2 : precisionLevel >= 2 ? 3 : 5;
        int maxStarts = 7;
        int count = 0;
        for (int start = 0; start < duration && count < maxStarts; start += step, ++count) {
            std::array<int, 4> widths = mode == VehicleType::Robot
                ? std::array<int, 4>{2, 6, 12, std::max(14, duration - start - 1)}
                : mode == VehicleType::Spider
                    ? std::array<int, 4>{1, 2, 4, 8}
                    : std::array<int, 4>{1, 3, 7, 12};
            for (int width : widths) {
                int pressAt = start;
                int releaseAt = std::min(duration - 1, pressAt + std::max(1, width));
                std::vector<std::pair<int, bool>> states;
                if (initial1 && pressAt > 0)
                    states.push_back({0, false});
                if (initial1 && pressAt == 0)
                    pressAt = std::min(duration - 2, 1);
                states.push_back({pressAt, true});
                states.push_back({releaseAt, false});
                addP1(std::move(states));
                if (actions.size() >= 28)
                    break;
            }
            if (actions.size() >= 28)
                break;
        }
    }

    UpcomingOrbV7 orb = upcomingOrbV7(probe, duration);
    if (orb.found && orb.contactOffset >= 0) {
        int tail = (orb.type == OrbType::Blue || orb.type == OrbType::Green) ? 6 : 4;
        std::array<int, 9> deltas = {-6, -4, -2, -1, 0, 1, 2, 4, 6};
        for (int delta : deltas) {
            int pressAt = std::clamp(orb.contactOffset + delta, 0, duration - 2);
            if (initial1 && pressAt == 0)
                pressAt = 1;
            int releaseAt = std::min(
                duration - 1,
                std::max(pressAt + 2, orb.contactOffset + tail)
            );
            std::vector<std::pair<int, bool>> states;
            if (initial1)
                states.push_back({0, false});
            states.push_back({pressAt, true});
            states.push_back({releaseAt, false});
            addP1(std::move(states));
        }
    }

    if (p1.dualActive) {
        size_t baseCount = std::min<size_t>(actions.size(), 10);
        for (size_t i = 0; i < baseCount; ++i) {
            std::vector<std::pair<int, bool>> p1States;
            std::vector<std::pair<int, bool>> p2States;

            bool s1 = initial1;
            bool s2 = initial2;
            for (auto const& toggle : actions[i].toggles) {
                if (toggle.player2)
                    continue;
                s1 = !s1;
                p1States.push_back({toggle.offset, s1});
                s2 = !s2;
                p2States.push_back({toggle.offset, s2});
            }
            addUniqueActionV7(
                actions,
                makeActionV7(duration, initial1, initial2, p1States, p2States),
                signatures
            );
        }
        addUniqueActionV7(
            actions,
            makeActionV7(duration, initial1, initial2, {}, {{0, false}}),
            signatures
        );
        addUniqueActionV7(
            actions,
            makeActionV7(duration, initial1, initial2, {}, {{0, true}}),
            signatures
        );
    }

    if (actions.size() > 36)
        actions.resize(36);
    return actions;
}

std::vector<MacroActionV7> explorerActionsV7(
    SnapshotV7 const& snapshot,
    int precisionLevel,
    uint32_t seed
) {
    Player const& p1 = snapshot.p1;
    VehicleType mode = p1.vehicle.type;
    int duration = baseSegmentFramesV7(mode, precisionLevel, true);
    std::vector<MacroActionV7> actions;
    actions.reserve(18);
    std::unordered_set<uint32_t> signatures;
    std::mt19937 rng(seed);

    addUniqueActionV7(
        actions,
        makeActionV7(duration, snapshot.press1, snapshot.press2, {}),
        signatures
    );
    addUniqueActionV7(
        actions,
        makeActionV7(duration, snapshot.press1, snapshot.press2, {{0, false}}),
        signatures
    );
    addUniqueActionV7(
        actions,
        makeActionV7(duration, snapshot.press1, snapshot.press2, {{0, true}}),
        signatures
    );

    int randomCount = precisionLevel >= 3 ? 15 : 12;
    std::uniform_int_distribution<int> offsetDist(0, std::max(0, duration - 2));

    for (int i = 0; i < randomCount; ++i) {
        std::vector<std::pair<int, bool>> p1States;
        std::vector<std::pair<int, bool>> p2States;

        if (flightMode(mode)) {
            int toggles = 1 + static_cast<int>(rng() % 4u);
            bool state = snapshot.press1;
            for (int t = 0; t < toggles; ++t) {
                int offset = offsetDist(rng);
                state = !state;
                p1States.push_back({offset, state});
            }
        } else {
            int start = offsetDist(rng);
            int maxWidth = std::max(2, duration - start - 1);
            int width = 1 + static_cast<int>(rng() % static_cast<uint32_t>(maxWidth));
            if (snapshot.press1)
                p1States.push_back({0, false});
            p1States.push_back({start, true});
            p1States.push_back({std::min(duration - 1, start + width), false});

            if ((rng() & 3u) == 0u) {
                int second = offsetDist(rng);
                int secondWidth = 1 + static_cast<int>(
                    rng() % static_cast<uint32_t>(std::max(2, duration - second))
                );
                p1States.push_back({second, true});
                p1States.push_back({std::min(duration - 1, second + secondWidth), false});
            }
        }

        if (p1.dualActive && (rng() & 1u)) {
            int offset = offsetDist(rng);
            bool desired = (rng() & 1u) != 0;
            p2States.push_back({offset, desired});
            if ((rng() & 1u) != 0)
                p2States.push_back({std::min(duration - 1, offset + 4), !desired});
        }

        addUniqueActionV7(
            actions,
            makeActionV7(
                duration,
                snapshot.press1,
                snapshot.press2,
                std::move(p1States),
                std::move(p2States)
            ),
            signatures
        );
    }
    return actions;
}

float scoreStateV7(
    Level2 const& level,
    SnapshotV7 const& snapshot,
    float minClearance,
    size_t routeSize,
    float startX
) {
    Player const& p1 = snapshot.p1;
    float progressX = p1.pos.x - startX;
    float clearance = std::clamp(minClearance, 0.f, 180.f);
    float score = progressX * 100.f;
    score += clearance * (flightMode(p1.vehicle.type) ? 2.4f : 0.65f);
    score += static_cast<float>(p1.frame) * 0.02f;
    score -= static_cast<float>(routeSize) * 0.8f;

    if (p1.grounded && !flightMode(p1.vehicle.type))
        score += 8.f;
    if (p1.direction < 0)
        score -= 30.f;
    if (p1.dashing)
        score += 10.f;
    if (p1.completed || p1.pos.x >= level.length)
        score += 1.0e9f;
    return score;
}

SimResultV7 simulateActionV7(
    Level2& worker,
    SearchNodeV7 const& parent,
    MacroActionV7 const& action,
    float startX
) {
    restoreSnapshotV7(worker, parent.snapshot);

    int startFrame = worker.currentFrame();
    int endFrame = startFrame + action.duration;
    size_t toggleCursor = 0;
    float minClearance = hazardClearance(worker, worker.latestState());
    float furthestX = worker.latestState().pos.x;

    while (worker.currentFrame() < endFrame &&
           !worker.latestState().dead &&
           !reachedGoal(worker)) {
        int offset = worker.currentFrame() - startFrame;
        while (toggleCursor < action.toggles.size() &&
               action.toggles[toggleCursor].offset <= offset) {
            if (action.toggles[toggleCursor].player2)
                worker.press2 = !worker.press2;
            else
                worker.press1 = !worker.press1;
            ++toggleCursor;
        }

        worker.runFrame(worker.press1, worker.press2, 1.f / 240.f);
        furthestX = std::max(furthestX, worker.latestState().pos.x);

        int stride = flightMode(worker.latestState().vehicle.type) ? 3 : 5;
        if ((worker.currentFrame() - startFrame) % stride == 0) {
            minClearance = std::min(
                minClearance,
                hazardClearance(worker, worker.latestState())
            );
            if (worker.latestState().dualActive) {
                minClearance = std::min(
                    minClearance,
                    hazardClearance(worker, worker.latestState2())
                );
            }
        }
    }

    SimResultV7 result;
    result.dead = worker.latestState().dead;
    result.deathX = result.dead ? worker.latestState().pos.x : 0.f;

    if (!result.dead) {
        result.node.snapshot = captureSnapshotV7(worker);
        result.node.route = parent.route;
        result.node.route.reserve(parent.route.size() + action.toggles.size());
        for (auto const& toggle : action.toggles) {
            result.node.route.push_back(
                inputKey(
                    static_cast<uint32_t>(startFrame + toggle.offset),
                    toggle.player2
                )
            );
        }
        result.node.x = worker.latestState().pos.x;
        result.node.y = worker.latestState().pos.y;
        result.node.velocity = worker.latestState().velocity;
        result.node.minClearance = minClearance;
        result.node.complete = reachedGoal(worker);
        result.node.depth = parent.depth + 1;
        result.node.explorer = parent.explorer;
        result.node.score = scoreStateV7(
            worker,
            result.node.snapshot,
            minClearance,
            result.node.route.size(),
            startX
        );
    }

    return result;
}

bool nodeBetterV7(SearchNodeV7 const& a, SearchNodeV7 const& b) {
    if (a.complete != b.complete)
        return a.complete;
    if (std::abs(a.x - b.x) > 0.5f)
        return a.x > b.x;
    if (std::abs(a.score - b.score) > 0.01f)
        return a.score > b.score;
    if (std::abs(a.minClearance - b.minClearance) > 0.2f)
        return a.minClearance > b.minClearance;
    return a.route.size() < b.route.size();
}

std::vector<SearchNodeV7> selectFrontierV7(
    std::vector<SearchNodeV7> candidates
) {
    std::sort(candidates.begin(), candidates.end(), nodeBetterV7);

    std::vector<SearchNodeV7> selected;
    selected.reserve(kLogicalHelpersV7);
    std::vector<bool> taken(candidates.size(), false);

    size_t guidedCount = std::min<size_t>(kGuidedSlotsV7, candidates.size());
    for (size_t i = 0; i < guidedCount; ++i) {
        candidates[i].explorer = false;
        selected.push_back(candidates[i]);
        taken[i] = true;
    }

    std::unordered_set<CoarseKeyV7, CoarseKeyHashV7> used;
    for (auto const& node : selected)
        used.insert(coarseKeyV7(node));

    size_t explorerTarget = std::min<size_t>(
        kLogicalHelpersV7,
        kGuidedSlotsV7 + kExplorerSlotsV7
    );

    for (size_t i = guidedCount;
         i < candidates.size() && selected.size() < explorerTarget;
         ++i) {
        CoarseKeyV7 key = coarseKeyV7(candidates[i]);
        if (!used.insert(key).second)
            continue;
        candidates[i].explorer = true;
        selected.push_back(candidates[i]);
        taken[i] = true;
    }

    for (size_t i = guidedCount;
         i < candidates.size() && selected.size() < explorerTarget;
         ++i) {
        if (taken[i])
            continue;
        candidates[i].explorer = true;
        selected.push_back(candidates[i]);
    }

    return selected;
}

void mergeArchiveV7(
    std::vector<SearchNodeV7>& archive,
    std::vector<SearchNodeV7> const& frontier
) {
    archive.insert(archive.end(), frontier.begin(), frontier.end());
    std::sort(archive.begin(), archive.end(), nodeBetterV7);

    std::unordered_set<CoarseKeyV7, CoarseKeyHashV7> seen;
    std::vector<SearchNodeV7> compact;
    compact.reserve(std::min<size_t>(archive.size(), kArchiveLimitV7));
    for (auto& node : archive) {
        CoarseKeyV7 key = coarseKeyV7(node);
        if (!seen.insert(key).second)
            continue;
        compact.push_back(std::move(node));
        if (compact.size() >= kArchiveLimitV7)
            break;
    }
    archive = std::move(compact);
}

std::vector<PathfinderInput> routeToInputsV7(std::vector<SearchInput> route) {
    std::sort(route.begin(), route.end());
    route.erase(std::unique(route.begin(), route.end()), route.end());

    bool held1 = false;
    bool held2 = false;
    std::vector<PathfinderInput> inputs;
    inputs.reserve(route.size());

    for (SearchInput key : route) {
        uint32_t frame = key >> 1;
        bool player2 = (key & 1u) != 0;
        bool& held = player2 ? held2 : held1;
        held = !held;
        inputs.push_back({frame, player2, held, 1});
    }
    return inputs;
}

std::vector<uint8_t> inputsToMacroV7(std::vector<PathfinderInput> const& inputs) {
    Replay2 output;
    for (auto const& input : inputs) {
        output.inputs.push_back(
            gdr::Input(
                input.frame,
                input.button,
                input.player2,
                input.down
            )
        );
    }
    return output.exportData().unwrapOr({});
}

struct ReplayValidationV7 {
    bool complete = false;
    bool dead = false;
    float furthestX = 0.f;
    float endX = 0.f;
    int frame = 0;
};

ReplayValidationV7 validateReplayV7(
    std::string const& lvlString,
    std::vector<PathfinderInput> const& inputs,
    float trustedEndX
) {
    Level2 check(lvlString);
    pruneNoTouchPhysicsV7(check, lvlString);

    float startX = check.latestState().pos.x;
    bool hasTrustedEnd = std::isfinite(trustedEndX) &&
                         trustedEndX > startX + 30.f;
    if (hasTrustedEnd) {
        check.length = trustedEndX;
        check.lengthSource = "trusted-gd-validation";
    }

    std::vector<PathfinderInput> ordered = inputs;
    std::sort(
        ordered.begin(),
        ordered.end(),
        [](PathfinderInput const& a, PathfinderInput const& b) {
            if (a.frame != b.frame)
                return a.frame < b.frame;
            return a.player2 < b.player2;
        }
    );

    bool held1 = check.press1;
    bool held2 = check.press2;
    size_t cursor = 0;
    float furthestX = check.latestState().pos.x;
    uint32_t lastInput = ordered.empty() ? 0u : ordered.back().frame;
    double distance = std::max(0.0, static_cast<double>(check.length - startX));
    int maxFrame = std::clamp(
        std::max(
            static_cast<int>(std::ceil(distance * 2.6)) + 6000,
            static_cast<int>(std::min<uint64_t>(
                static_cast<uint64_t>(lastInput) + 30000ull,
                200000ull
            ))
        ),
        6000,
        200000
    );

    while (check.currentFrame() < maxFrame &&
           !check.latestState().dead &&
           !reachedGoal(check)) {
        uint32_t frame = static_cast<uint32_t>(check.currentFrame());
        while (cursor < ordered.size() && ordered[cursor].frame <= frame) {
            auto const& input = ordered[cursor++];
            if (input.button != 1)
                continue;
            if (input.player2)
                held2 = input.down;
            else
                held1 = input.down;
        }
        check.press1 = held1;
        check.press2 = held2;
        check.runFrame(held1, held2, 1.f / 240.f);
        furthestX = std::max(furthestX, check.latestState().pos.x);
    }

    ReplayValidationV7 validation;
    validation.dead = check.latestState().dead;
    validation.furthestX = furthestX;
    validation.endX = check.length;
    validation.frame = check.currentFrame();
    bool reachedTrusted = !hasTrustedEnd || furthestX >= trustedEndX - 30.f;
    validation.complete = !validation.dead && reachedGoal(check) && reachedTrusted;
    return validation;
}

PathfinderResult runStateSearchV7(
    std::string const& lvlString,
    std::atomic_bool& stop,
    std::function<void(PathfinderTelemetry const&)> callback,
    float trustedEndX
) {
    Level2 root(lvlString);
    int prunedNoTouch = pruneNoTouchPhysicsV7(root, lvlString);

    float startX = root.latestState().pos.x;
    float inferredLength = root.length;
    bool hasTrustedEnd = std::isfinite(trustedEndX) &&
                         trustedEndX > startX + 30.f;
    if (hasTrustedEnd) {
        root.length = trustedEndX;
        root.lengthSource = "trusted-gd";
    }

    SearchNodeV7 initial;
    initial.snapshot = captureSnapshotV7(root);
    initial.x = root.latestState().pos.x;
    initial.y = root.latestState().pos.y;
    initial.velocity = root.latestState().velocity;
    initial.minClearance = hazardClearance(root, root.latestState());
    initial.score = scoreStateV7(root, initial.snapshot, initial.minClearance, 0, startX);

    std::vector<SearchNodeV7> frontier;
    frontier.push_back(initial);
    std::vector<SearchNodeV7> archive;
    archive.push_back(initial);

    SearchNodeV7 bestNode = initial;
    float furthestX = initial.x;
    float lastDeathX = 0.f;
    uint64_t totalTrials = 0;
    int precisionLevel = 0;
    int stallLayers = 0;
    int recoveryCount = 0;
    int layer = 0;

    std::unordered_map<StateKeyV7, float, StateKeyHashV7> visited;
    visited[stateKeyV7(initial.snapshot, precisionLevel)] = initial.score;

    std::random_device rd;
    uint32_t baseSeed = rd() ^
        static_cast<uint32_t>(std::hash<std::string>{}(lvlString));

    auto emit = [&](int phase, char const* reason, int segmentFrames, int candidateCount) {
        if (!callback)
            return;
        PathfinderTelemetry telemetry;
        telemetry.progress = progressFor(
            furthestX,
            startX,
            root.length,
            bestNode.complete
        );
        telemetry.startX = startX;
        telemetry.currentX = bestNode.x;
        telemetry.furthestX = furthestX;
        telemetry.trustedEndX = hasTrustedEnd ? trustedEndX : 0.f;
        telemetry.inferredLength = inferredLength;
        telemetry.checkpointX = bestNode.x;
        telemetry.deathX = lastDeathX;
        telemetry.deathProgress = static_cast<float>(
            progressFor(lastDeathX, startX, root.length, false)
        );
        telemetry.bestClearance = bestNode.minClearance;
        telemetry.frame = bestNode.snapshot.p1.frame;
        telemetry.checkpointFrame = bestNode.snapshot.p1.frame;
        telemetry.vehicleType = static_cast<int>(bestNode.snapshot.p1.vehicle.type);
        telemetry.searchLevel = precisionLevel;
        telemetry.horizonFrames = segmentFrames;
        telemetry.candidateCount = candidateCount;
        telemetry.workerCount = kLogicalHelpersV7;
        telemetry.phase = phase;
        telemetry.totalTrials = totalTrials;
        telemetry.mode = "state-space-beam100-v7";
        telemetry.recoveryReason = reason;
        callback(telemetry);
    };

    emit(0, "state-space-start", baseSegmentFramesV7(root.latestState().vehicle.type, 0, false), 1);

    while (!frontier.empty() && !stop.load() && !bestNode.complete) {
        ++layer;

        struct TaskV7 {
            size_t parent = 0;
            MacroActionV7 action;
        };

        std::vector<TaskV7> tasks;
        tasks.reserve(frontier.size() * 20);
        Level2 actionProbe = compactWorkerV7(root);

        for (size_t i = 0; i < frontier.size(); ++i) {
            auto actions = frontier[i].explorer
                ? explorerActionsV7(
                    frontier[i].snapshot,
                    precisionLevel,
                    baseSeed ^
                        static_cast<uint32_t>(layer * 104729u) ^
                        static_cast<uint32_t>(i * 2654435761u)
                )
                : guidedActionsV7(actionProbe, frontier[i].snapshot, precisionLevel);

            for (auto& action : actions)
                tasks.push_back({i, std::move(action)});
        }

        if (tasks.empty()) {
            ++recoveryCount;
            ++precisionLevel;
            precisionLevel = std::min(precisionLevel, 6);
            frontier = selectFrontierV7(archive);
            visited.clear();
            for (auto const& node : frontier)
                visited[stateKeyV7(node.snapshot, precisionLevel)] = node.score;
            emit(2, "reseed-empty-frontier", 0, 0);
            continue;
        }

        emit(
            1,
            "state-expansion",
            tasks.front().action.duration,
            static_cast<int>(tasks.size())
        );

        std::atomic<size_t> nextTask {0};
        std::atomic<uint64_t> completedTasks {0};
        std::atomic<uint64_t> nextReport {128};
        std::atomic<float> liveBestX {furthestX};
        std::mutex reportMutex;
        std::mutex resultMutex;

        std::vector<SearchNodeV7> produced;
        produced.reserve(tasks.size());
        float layerDeathX = 0.f;

        int threadCount = std::min<int>(
            kPhysicalThreadsV7,
            static_cast<int>(tasks.size())
        );
        std::vector<std::thread> threads;
        threads.reserve(threadCount);

        for (int threadIndex = 0; threadIndex < threadCount; ++threadIndex) {
            threads.emplace_back([&] {
                Level2 worker = compactWorkerV7(root);
                std::vector<SearchNodeV7> localProduced;
                localProduced.reserve(tasks.size() / std::max(1, threadCount) + 8);
                float localDeathX = 0.f;

                while (!stop.load()) {
                    size_t taskIndex = nextTask.fetch_add(1, std::memory_order_relaxed);
                    if (taskIndex >= tasks.size())
                        break;

                    TaskV7 const& task = tasks[taskIndex];
                    SimResultV7 result = simulateActionV7(
                        worker,
                        frontier[task.parent],
                        task.action,
                        startX
                    );

                    if (result.dead) {
                        localDeathX = std::max(localDeathX, result.deathX);
                    } else {
                        float current = liveBestX.load(std::memory_order_relaxed);
                        while (result.node.x > current &&
                               !liveBestX.compare_exchange_weak(
                                   current,
                                   result.node.x,
                                   std::memory_order_relaxed
                               )) {}
                        localProduced.push_back(std::move(result.node));
                    }

                    uint64_t done = completedTasks.fetch_add(1, std::memory_order_relaxed) + 1;
                    uint64_t target = nextReport.load(std::memory_order_relaxed);
                    if (callback && done >= target &&
                        nextReport.compare_exchange_strong(
                            target,
                            done + 128,
                            std::memory_order_relaxed
                        )) {
                        std::lock_guard<std::mutex> guard(reportMutex);
                        PathfinderTelemetry telemetry;
                        float liveX = std::max(
                            furthestX,
                            liveBestX.load(std::memory_order_relaxed)
                        );
                        telemetry.progress = progressFor(
                            liveX,
                            startX,
                            root.length,
                            false
                        );
                        telemetry.startX = startX;
                        telemetry.currentX = liveX;
                        telemetry.furthestX = liveX;
                        telemetry.trustedEndX = hasTrustedEnd ? trustedEndX : 0.f;
                        telemetry.inferredLength = inferredLength;
                        telemetry.checkpointX = bestNode.x;
                        telemetry.deathX = lastDeathX;
                        telemetry.bestClearance = bestNode.minClearance;
                        telemetry.frame = bestNode.snapshot.p1.frame;
                        telemetry.checkpointFrame = bestNode.snapshot.p1.frame;
                        telemetry.vehicleType = static_cast<int>(bestNode.snapshot.p1.vehicle.type);
                        telemetry.searchLevel = precisionLevel;
                        telemetry.horizonFrames = tasks[taskIndex].action.duration;
                        telemetry.candidateCount = static_cast<int>(tasks.size());
                        telemetry.workerCount = kLogicalHelpersV7;
                        telemetry.phase = 1;
                        telemetry.totalTrials = totalTrials + done;
                        telemetry.mode = "state-space-beam100-v7";
                        telemetry.recoveryReason = "live-state-expansion";
                        callback(telemetry);
                    }
                }

                std::lock_guard<std::mutex> guard(resultMutex);
                layerDeathX = std::max(layerDeathX, localDeathX);
                produced.insert(
                    produced.end(),
                    std::make_move_iterator(localProduced.begin()),
                    std::make_move_iterator(localProduced.end())
                );
            });
        }

        for (auto& thread : threads)
            thread.join();

        totalTrials += completedTasks.load(std::memory_order_relaxed);
        if (layerDeathX > 0.f)
            lastDeathX = layerDeathX;

        std::unordered_map<StateKeyV7, SearchNodeV7, StateKeyHashV7> unique;
        unique.reserve(produced.size());

        for (auto& node : produced) {
            StateKeyV7 key = stateKeyV7(node.snapshot, precisionLevel);
            auto visitedIt = visited.find(key);
            if (visitedIt != visited.end() && visitedIt->second >= node.score - 0.01f)
                continue;
            visited[key] = node.score;

            auto it = unique.find(key);
            if (it == unique.end() || nodeBetterV7(node, it->second))
                unique[key] = std::move(node);
        }

        std::vector<SearchNodeV7> candidates;
        candidates.reserve(unique.size());
        for (auto& [_, node] : unique)
            candidates.push_back(std::move(node));

        bool improved = false;
        for (auto const& node : candidates) {
            if (node.complete || nodeBetterV7(node, bestNode)) {
                if (node.x > furthestX + 1.f || node.complete)
                    improved = true;
                if (nodeBetterV7(node, bestNode))
                    bestNode = node;
                furthestX = std::max(furthestX, node.x);
            }
        }

        if (bestNode.complete)
            break;

        frontier = selectFrontierV7(std::move(candidates));

        if (!frontier.empty())
            mergeArchiveV7(archive, frontier);

        if (improved) {
            stallLayers = 0;
            if (precisionLevel > 0 && (layer % 5) == 0)
                --precisionLevel;
            emit(
                3,
                "state-frontier-advance",
                baseSegmentFramesV7(bestNode.snapshot.p1.vehicle.type, precisionLevel, false),
                static_cast<int>(frontier.size())
            );
        } else {
            ++stallLayers;
        }

        int stallLimit = precisionLevel >= 3 ? 4 : 6;
        if (frontier.empty() || stallLayers >= stallLimit) {
            ++recoveryCount;
            precisionLevel = std::min(6, precisionLevel + 1);
            stallLayers = 0;

            std::vector<SearchNodeV7> reseed;
            reseed.reserve(kLogicalHelpersV7);
            float backWindow = 500.f + static_cast<float>(precisionLevel) * 220.f;

            for (auto const& node : archive) {
                if (node.x + backWindow < furthestX)
                    continue;
                reseed.push_back(node);
                if (reseed.size() >= kLogicalHelpersV7)
                    break;
            }
            if (reseed.empty() && !archive.empty())
                reseed.push_back(archive.front());
            if (reseed.empty())
                reseed.push_back(initial);

            frontier = selectFrontierV7(std::move(reseed));
            visited.clear();
            for (auto const& node : frontier)
                visited[stateKeyV7(node.snapshot, precisionLevel)] = node.score;

            emit(
                2,
                "diverse-backtrack",
                baseSegmentFramesV7(bestNode.snapshot.p1.vehicle.type, precisionLevel, false),
                static_cast<int>(frontier.size())
            );
        }
    }

    PathfinderResult result;
    result.inputs = routeToInputsV7(bestNode.route);
    result.macro = inputsToMacroV7(result.inputs);
    result.complete = bestNode.complete;
    result.progress = progressFor(
        furthestX,
        startX,
        root.length,
        result.complete
    );

    std::ostringstream diagnostics;
    diagnostics
        << "solver=state-space-beam100-v7"
        << " progress=" << result.progress
        << " routeX=" << bestNode.x
        << " furthestX=" << furthestX
        << " endX=" << root.length
        << " layers=" << layer
        << " precisionLevel=" << precisionLevel
        << " recoveryCount=" << recoveryCount
        << " totalTrials=" << totalTrials
        << " logicalHelpers=" << kLogicalHelpersV7
        << " physicalThreads=" << kPhysicalThreadsV7
        << " archive=" << archive.size()
        << " prunedNoTouch=" << prunedNoTouch
        << " inputs=" << result.inputs.size()
        << " complete=" << (result.complete ? 1 : 0)
        << " stopped=" << (stop.load() ? 1 : 0);
    result.diagnostics = diagnostics.str();
    return result;
}

void addValidatedIdleInputV7(PathfinderResult& result) {
    result.inputs.push_back({1u, false, false, 1u});
    result.macro = inputsToMacroV7(result.inputs);
}

} // namespace

PathfinderResult pathfind_v7(
    std::string const& lvlString,
    std::atomic_bool& stop,
    std::function<void(PathfinderTelemetry const&)> callback,
    float trustedEndX
) {
    std::function<void(PathfinderTelemetry const&)> guarded;
    if (callback) {
        guarded = [callback](PathfinderTelemetry const& incoming) {
            PathfinderTelemetry telemetry = incoming;
            if (telemetry.progress >= 100.0) {
                telemetry.progress = 99.99;
                telemetry.phase = 1;
                telemetry.recoveryReason = "awaiting-replay-validation";
            }
            callback(telemetry);
        };
    }

    PathfinderResult result = runStateSearchV7(
        lvlString,
        stop,
        guarded,
        trustedEndX
    );

    if (!result.complete) {
        result.diagnostics += " replayValidation=not-needed";
        return result;
    }

    bool zeroInput = result.inputs.empty();
    ReplayValidationV7 validation = validateReplayV7(
        lvlString,
        result.inputs,
        trustedEndX
    );

    if (!validation.complete) {
        result.complete = false;
        Level2 progressLevel(lvlString);
        float startX = progressLevel.latestState().pos.x;
        float endX = (
            std::isfinite(trustedEndX) &&
            trustedEndX > startX + 30.f
        ) ? trustedEndX : progressLevel.length;
        result.progress = progressFor(
            validation.furthestX,
            startX,
            endX,
            false
        );
        result.diagnostics += " replayValidation=FAILED";
        result.diagnostics += " validatedFrame=" + std::to_string(validation.frame);
        result.diagnostics += " validatedX=" + std::to_string(validation.furthestX);
        result.diagnostics += " validationDead=" + std::to_string(validation.dead ? 1 : 0);
        return result;
    }

    if (zeroInput)
        addValidatedIdleInputV7(result);

    result.complete = true;
    result.progress = 100.0;
    result.diagnostics += " replayValidation=PASS";
    result.diagnostics += " validatedFrame=" + std::to_string(validation.frame);
    result.diagnostics += " validatedX=" + std::to_string(validation.furthestX);
    return result;
}
