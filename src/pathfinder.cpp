#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <limits>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Level.hpp>
#include <gdr/gdr.hpp>

#include "pathfinder.hpp"

class Replay2 : public gdr::Replay<Replay2, gdr::Input<"">> {
public:
    Replay2() : Replay("Pathfinder", 1) {}
};

namespace {

using ObjectFields = std::unordered_map<int, std::string>;

ObjectFields parseFields(std::string const& text) {
    ObjectFields fields;
    std::stringstream stream(text);
    std::string key;
    std::string value;
    while (std::getline(stream, key, ',')) {
        if (!std::getline(stream, value, ','))
            break;
        int id = std::atoi(key.c_str());
        if (id > 0)
            fields[id] = value;
    }
    return fields;
}

int intField(ObjectFields const& fields, int key, int fallback = 0) {
    auto it = fields.find(key);
    return it == fields.end() || it->second.empty()
        ? fallback
        : std::atoi(it->second.c_str());
}

float floatField(ObjectFields const& fields, int key, float fallback = 0.f) {
    auto it = fields.find(key);
    return it == fields.end() || it->second.empty()
        ? fallback
        : static_cast<float>(std::atof(it->second.c_str()));
}

bool boolField(ObjectFields const& fields, int key) {
    return intField(fields, key, 0) != 0;
}

std::vector<int> objectGroups(ObjectFields const& fields) {
    std::vector<int> result;
    auto add = [&result](int group) {
        if (group <= 0)
            return;
        if (std::find(result.begin(), result.end(), group) == result.end())
            result.push_back(group);
    };

    add(intField(fields, 33));
    if (auto it = fields.find(57); it != fields.end()) {
        std::stringstream stream(it->second);
        std::string group;
        while (std::getline(stream, group, '.'))
            add(std::atoi(group.c_str()));
    }
    return result;
}

float bounceOut(float t) {
    constexpr float n1 = 7.5625f;
    constexpr float d1 = 2.75f;
    if (t < 1.f / d1)
        return n1 * t * t;
    if (t < 2.f / d1) {
        t -= 1.5f / d1;
        return n1 * t * t + 0.75f;
    }
    if (t < 2.5f / d1) {
        t -= 2.25f / d1;
        return n1 * t * t + 0.9375f;
    }
    t -= 2.625f / d1;
    return n1 * t * t + 0.984375f;
}

float moveEase(int type, float t, float rate) {
    t = std::clamp(t, 0.f, 1.f);
    constexpr float pi = 3.14159265358979323846f;
    float power = std::max(1.f, rate <= 0.f ? 2.f : rate);

    switch (type) {
        case 0: return t;
        case 1:
            return t < 0.5f
                ? 0.5f * std::pow(t * 2.f, power)
                : 1.f - 0.5f * std::pow((1.f - t) * 2.f, power);
        case 2: return std::pow(t, power);
        case 3: return 1.f - std::pow(1.f - t, power);
        case 4: {
            if (t == 0.f || t == 1.f) return t;
            float x = t * 2.f - 1.f;
            float c = (2.f * pi) / 4.5f;
            return x < 0.f
                ? -(std::pow(2.f, 10.f * x) * std::sin((x * 10.f - 0.75f) * c)) / 2.f
                : (std::pow(2.f, -10.f * x) * std::sin((x * 10.f - 0.75f) * c)) / 2.f + 1.f;
        }
        case 5: {
            if (t == 0.f || t == 1.f) return t;
            float c = (2.f * pi) / 3.f;
            return -std::pow(2.f, 10.f * t - 10.f) * std::sin((t * 10.f - 10.75f) * c);
        }
        case 6: {
            if (t == 0.f || t == 1.f) return t;
            float c = (2.f * pi) / 3.f;
            return std::pow(2.f, -10.f * t) * std::sin((t * 10.f - 0.75f) * c) + 1.f;
        }
        case 7:
            return t < 0.5f
                ? (1.f - bounceOut(1.f - 2.f * t)) * 0.5f
                : (1.f + bounceOut(2.f * t - 1.f)) * 0.5f;
        case 8: return 1.f - bounceOut(1.f - t);
        case 9: return bounceOut(t);
        case 10:
            if (t == 0.f || t == 1.f) return t;
            return t < 0.5f
                ? std::pow(2.f, 20.f * t - 10.f) * 0.5f
                : (2.f - std::pow(2.f, -20.f * t + 10.f)) * 0.5f;
        case 11: return t == 0.f ? 0.f : std::pow(2.f, 10.f * t - 10.f);
        case 12: return t == 1.f ? 1.f : 1.f - std::pow(2.f, -10.f * t);
        case 13: return -(std::cos(pi * t) - 1.f) * 0.5f;
        case 14: return 1.f - std::cos((t * pi) * 0.5f);
        case 15: return std::sin((t * pi) * 0.5f);
        case 16: {
            constexpr float c1 = 1.70158f;
            constexpr float c2 = c1 * 1.525f;
            return t < 0.5f
                ? (std::pow(2.f * t, 2.f) * ((c2 + 1.f) * 2.f * t - c2)) * 0.5f
                : (std::pow(2.f * t - 2.f, 2.f) * ((c2 + 1.f) * (t * 2.f - 2.f) + c2) + 2.f) * 0.5f;
        }
        case 17: {
            constexpr float c1 = 1.70158f;
            return (c1 + 1.f) * t * t * t - c1 * t * t;
        }
        case 18: {
            constexpr float c1 = 1.70158f;
            float x = t - 1.f;
            return 1.f + (c1 + 1.f) * x * x * x + c1 * x * x;
        }
        default: return t;
    }
}

} // namespace

struct Level2 : public Level {
    struct MoveTriggerState {
        float x = 0.f;
        float moveX = 0.f;
        float moveY = 0.f;
        float duration = 0.f;
        float easingRate = 2.f;
        int targetGroup = 0;
        int easing = 0;
        int activationFrame = -1;
        bool unsupportedMode = false;
    };

    bool press1 = false;
    bool press2 = false;
    float highestY = 0.f;
    std::vector<MoveTriggerState> moveTriggers;
    std::unordered_map<int, std::vector<int>> groupObjects;
    std::unordered_map<int, Vec2D> baseObjectPositions;
    std::unordered_map<int, int> objectSections;
    std::unordered_set<int> movingObjectIDs;
    int supportedMoveTriggers = 0;
    int unsupportedMoveTriggers = 0;

    using Level::Level;

    explicit Level2(std::string const& lvlString) : Level(lvlString) {
        for (auto const& [sectionIndex, section] : sections) {
            for (auto const& object : section) {
                highestY = std::max(highestY, object->pos.y);
                baseObjectPositions[object->id] = object->pos;
                objectSections[object->id] = sectionIndex;
            }
        }
        parseDynamicGeometry(lvlString);
        rebuildMoveActivations();
        applyMoveGeometry(currentFrame());
    }

    void parseDynamicGeometry(std::string const& lvlString) {
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

            if (objectID == 901) {
                MoveTriggerState trigger;
                trigger.x = floatField(fields, 2);
                trigger.moveX = floatField(fields, 28) * 3.f;
                trigger.moveY = floatField(fields, 29) * 3.f;
                trigger.duration = floatField(fields, 10, 0.f);
                trigger.targetGroup = intField(fields, 51);
                trigger.easing = intField(fields, 30);
                trigger.easingRate = floatField(fields, 85, 2.f);

                trigger.unsupportedMode =
                    boolField(fields, 11) ||
                    boolField(fields, 62) ||
                    boolField(fields, 58) ||
                    boolField(fields, 59) ||
                    boolField(fields, 100) ||
                    trigger.targetGroup <= 0;

                if (trigger.unsupportedMode)
                    ++unsupportedMoveTriggers;
                else
                    ++supportedMoveTriggers;

                moveTriggers.push_back(trigger);
                continue;
            }

            auto groups = objectGroups(fields);
            ObjectFields probe = fields;
            if (Object::create(std::move(probe))) {
                for (int group : groups)
                    groupObjects[group].push_back(supportedIndex);
                ++supportedIndex;
            }
        }

        for (auto const& trigger : moveTriggers) {
            if (trigger.unsupportedMode)
                continue;
            auto members = groupObjects.find(trigger.targetGroup);
            if (members == groupObjects.end())
                continue;
            for (int objectID : members->second) {
                if (baseObjectPositions.contains(objectID))
                    movingObjectIDs.insert(objectID);
            }
        }
    }

    void moveObjectTo(int objectID, Vec2D const& position) {
        auto sectionRef = objectSections.find(objectID);
        if (sectionRef == objectSections.end())
            return;

        int currentSection = sectionRef->second;
        int desiredSection = static_cast<int>(std::floor(position.x / sectionSize));
        auto current = sections.find(currentSection);
        if (current == sections.end())
            return;

        auto& objects = current->second;
        auto object = std::find_if(
            objects.begin(),
            objects.end(),
            [objectID](ObjectContainer const& value) {
                return value->id == objectID;
            }
        );
        if (object == objects.end())
            return;

        if (desiredSection == currentSection) {
            object->operator->()->pos = position;
            return;
        }

        ObjectContainer moving = std::move(*object);
        objects.erase(object);
        moving->pos = position;
        sections[desiredSection].push_back(std::move(moving));
        objectSections[objectID] = desiredSection;
    }

    void activateMoveTriggers(Player const& state) {
        for (auto& trigger : moveTriggers) {
            if (trigger.unsupportedMode || trigger.activationFrame >= 0)
                continue;
            if (state.pos.x + 0.01f >= trigger.x)
                trigger.activationFrame = state.frame;
        }
    }

    void rebuildMoveActivations() {
        for (auto& trigger : moveTriggers) {
            trigger.activationFrame = -1;
            if (trigger.unsupportedMode)
                continue;

            for (auto const& state : gameStates) {
                if (state.pos.x + 0.01f >= trigger.x) {
                    trigger.activationFrame = state.frame;
                    break;
                }
            }
        }
    }

    void applyMoveGeometry(int frame) {
        if (movingObjectIDs.empty())
            return;

        std::unordered_map<int, Vec2D> offsets;
        offsets.reserve(movingObjectIDs.size());

        for (auto const& trigger : moveTriggers) {
            if (trigger.unsupportedMode || trigger.activationFrame < 0)
                continue;

            float progress = 1.f;
            if (trigger.duration > 0.f) {
                float elapsed = static_cast<float>(frame - trigger.activationFrame) / 240.f;
                progress = std::clamp(elapsed / trigger.duration, 0.f, 1.f);
            } else if (trigger.duration < 0.f) {
                progress = 0.f;
            }

            float eased = moveEase(trigger.easing, progress, trigger.easingRate);
            Vec2D delta {trigger.moveX * eased, trigger.moveY * eased};

            auto members = groupObjects.find(trigger.targetGroup);
            if (members == groupObjects.end())
                continue;

            for (int objectID : members->second) {
                if (!movingObjectIDs.contains(objectID))
                    continue;
                offsets[objectID] += delta;
            }
        }

        for (int objectID : movingObjectIDs) {
            auto base = baseObjectPositions.find(objectID);
            if (base == baseObjectPositions.end())
                continue;
            Vec2D position = base->second;
            if (auto offset = offsets.find(objectID); offset != offsets.end())
                position += offset->second;
            moveObjectTo(objectID, position);
        }
    }

    Player& runFrame(bool pressed, float dt = 1.f / 240.f) {
        return runFrame(pressed, pressed, dt);
    }

    Player& runFrame(bool player1Pressed, bool player2Pressed, float dt) {
        activateMoveTriggers(latestState());
        applyMoveGeometry(currentFrame());
        Player& result = Level::runFrame(player1Pressed, player2Pressed, dt);
        activateMoveTriggers(latestState());
        return result;
    }

    void rollback(int frame) {
        Level::rollback(frame);
        rebuildMoveActivations();
        applyMoveGeometry(currentFrame());
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
        rebuildMoveActivations();
        applyMoveGeometry(currentFrame());
    }
};

using SearchInput = uint32_t;

static SearchInput inputKey(uint32_t frame, bool player2) {
    return (frame << 1) | static_cast<uint32_t>(player2);
}

static bool reachedGoal(Level2 const& lvl) {
    return !lvl.gameStates.empty() && lvl.gameStates.back().completed;
}

static bool flightMode(VehicleType type) {
    return type == VehicleType::Ship ||
           type == VehicleType::Wave ||
           type == VehicleType::Swing;
}

struct TrialResult {
    int frame = 0;
    float x = 0.f;
    float deathX = 0.f;
    float minClearance = 10000.f;
    bool dead = false;
    bool complete = false;
};

struct Timeline {
    std::vector<Player> p1;
    std::vector<Player> p2;
    bool press1 = false;
    bool press2 = false;

    explicit Timeline(Level2 const& lvl) {
        capture(lvl);
    }

    void capture(Level2 const& lvl) {
        p1 = lvl.gameStates;
        p2 = lvl.gameStates2;
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

    int frame() const {
        return p1.empty() ? 0 : p1.back().frame;
    }

    float x() const {
        return p1.empty() ? 0.f : p1.back().pos.x;
    }

    bool complete() const {
        return !p1.empty() && !p1.back().dead && p1.back().completed;
    }
};

static int maxToggleBudget(VehicleType type) {
    switch (type) {
        case VehicleType::Cube:   return 10;
        case VehicleType::Ship:   return 18;
        case VehicleType::Ball:   return 10;
        case VehicleType::Ufo:    return 14;
        case VehicleType::Wave:   return 24;
        case VehicleType::Robot:  return 12;
        case VehicleType::Spider: return 8;
        case VehicleType::Swing:  return 20;
    }
    return 12;
}

static float hazardClearance(Level2 const& lvl, Player const& player) {
    Entity hitbox = player.unrotatedHitbox();
    int section = static_cast<int>(std::floor(player.pos.x / Level::sectionSize));
    float best = 10000.f;

    for (int s = section - 1; s <= section + 1; ++s) {
        auto it = lvl.sections.find(s);
        if (it == lvl.sections.end())
            continue;

        for (auto const& object : it->second) {
            if (object->prio != 2)
                continue;

            float dx = std::max({
                object->getLeft() - hitbox.getRight(),
                hitbox.getLeft() - object->getRight(),
                0.f
            });
            float dy = std::max({
                object->getBottom() - hitbox.getTop(),
                hitbox.getBottom() - object->getTop(),
                0.f
            });
            best = std::min(best, std::sqrt(dx * dx + dy * dy));
        }
    }

    return best;
}

static TrialResult tryInputs(
    Level2& lvl,
    std::set<SearchInput> const& inputs,
    int horizonFrames
) {
    int startFrame = lvl.currentFrame();
    float startX = lvl.latestState().pos.x;
    bool press1Before = lvl.press1;
    bool press2Before = lvl.press2;
    int endFrame = startFrame + horizonFrames;
    VehicleType startMode = lvl.latestState().vehicle.type;
    int clearanceStride = flightMode(startMode)
        ? 6
        : startMode == VehicleType::Robot ? 8 : 12;
    float minClearance = hazardClearance(lvl, lvl.latestState());

    while (lvl.currentFrame() < endFrame &&
           !lvl.latestState().dead &&
           !reachedGoal(lvl)) {
        uint32_t current = static_cast<uint32_t>(lvl.currentFrame());
        if (inputs.contains(inputKey(current, false)))
            lvl.press1 = !lvl.press1;
        if (inputs.contains(inputKey(current, true)))
            lvl.press2 = !lvl.press2;

        lvl.runFrame(lvl.press1, lvl.press2, 1.f / 240.f);

        if (!lvl.latestState().dead &&
            (lvl.currentFrame() - startFrame) % clearanceStride == 0) {
            minClearance = std::min(minClearance, hazardClearance(lvl, lvl.latestState()));
            if (lvl.latestState().dualActive)
                minClearance = std::min(minClearance, hazardClearance(lvl, lvl.latestState2()));
        }
    }

    TrialResult result {
        lvl.currentFrame(),
        lvl.latestState().pos.x,
        lvl.latestState().dead ? lvl.latestState().pos.x : 0.f,
        minClearance,
        lvl.latestState().dead,
        !lvl.latestState().dead && reachedGoal(lvl)
    };

    float y = lvl.latestState().pos.y;
    if (!result.complete &&
        (y > std::max(1300.f, lvl.highestY + 600.f) || y < -600.f)) {
        result.frame = startFrame;
        result.x = startX;
        result.deathX = startX;
        result.dead = true;
    }

    lvl.rollback(startFrame);
    lvl.press1 = press1Before;
    lvl.press2 = press2Before;
    return result;
}

static double progressFor(float x, float startX, float endX, bool complete) {
    if (complete)
        return 100.0;
    double span = static_cast<double>(endX - startX);
    if (span <= 1.0)
        return 0.0;
    return std::clamp(
        ((static_cast<double>(x) - startX) / span) * 100.0,
        0.0,
        99.99
    );
}

static bool betterTrial(
    TrialResult const& trial,
    size_t toggleCount,
    bool haveBest,
    TrialResult const& best,
    size_t bestToggleCount,
    VehicleType mode
) {
    if (!haveBest)
        return true;

    if (trial.complete != best.complete)
        return trial.complete;
    if (trial.dead != best.dead)
        return !trial.dead;

    if (trial.dead && best.dead) {
        if (std::abs(trial.x - best.x) > 0.01f)
            return trial.x > best.x;
        if (trial.frame != best.frame)
            return trial.frame > best.frame;
        return toggleCount < bestToggleCount;
    }

    float progressBand = flightMode(mode)
        ? 42.f
        : mode == VehicleType::Robot ? 28.f
        : mode == VehicleType::Spider ? 20.f
        : 16.f;
    float xDelta = trial.x - best.x;
    if (std::abs(xDelta) > progressBand)
        return xDelta > 0.f;

    float clearanceDelta = trial.minClearance - best.minClearance;
    float clearanceBand = flightMode(mode)
        ? 1.5f
        : mode == VehicleType::Robot ? 1.0f
        : 0.75f;
    if (std::abs(clearanceDelta) > clearanceBand)
        return clearanceDelta > 0.f;

    if (std::abs(xDelta) > 0.01f)
        return xDelta > 0.f;
    if (trial.frame != best.frame)
        return trial.frame > best.frame;
    return toggleCount < bestToggleCount;
}

static void addToggle(
    std::set<SearchInput>& inputs,
    int frame,
    int horizonFrames,
    int offset,
    bool player2
) {
    if (offset < 0 || offset >= horizonFrames)
        return;
    inputs.insert(inputKey(static_cast<uint32_t>(frame + offset), player2));
}

static std::vector<std::set<SearchInput>> structuredCandidates(
    int frame,
    int horizonFrames,
    VehicleType mode,
    bool dual,
    bool heldP1
) {
    std::vector<std::set<SearchInput>> candidates;
    candidates.reserve(340);
    candidates.emplace_back();

    static constexpr int offsets[] = {
        0, 1, 2, 4, 6, 8, 12, 16, 24, 36, 48, 72, 96, 144, 192
    };
    static constexpr int windows[] = {
        2, 4, 6, 8, 12, 16, 24, 36, 48, 72
    };

    for (int offset : offsets) {
        std::set<SearchInput> one;
        addToggle(one, frame, horizonFrames, offset, false);
        candidates.push_back(std::move(one));
    }

    for (int start : offsets) {
        if (start > 96)
            break;
        for (int width : windows) {
            if (start + width >= horizonFrames)
                continue;
            std::set<SearchInput> pulse;
            addToggle(pulse, frame, horizonFrames, start, false);
            addToggle(pulse, frame, horizonFrames, start + width, false);
            candidates.push_back(std::move(pulse));
        }
    }

    if (flightMode(mode)) {
        static constexpr int halfPeriods[] = {
            4, 6, 8, 10, 12, 16, 20, 24, 30, 36, 48
        };
        int patternWindow = std::min(horizonFrames, 600);

        for (int halfPeriod : halfPeriods) {
            for (int phase : {0, halfPeriod / 2}) {
                std::set<SearchInput> rhythm;
                int toggles = 0;
                for (int t = phase; t < patternWindow && toggles < 28; t += halfPeriod) {
                    addToggle(rhythm, frame, horizonFrames, t, false);
                    ++toggles;
                }
                candidates.push_back(std::move(rhythm));
            }
        }

        for (int start : {0, 8, 16, 24, 36, 48, 72, 96}) {
            for (int width : {12, 18, 24, 30, 36, 48, 60, 84}) {
                if (start + width >= horizonFrames)
                    continue;
                std::set<SearchInput> correction;
                addToggle(correction, frame, horizonFrames, start, false);
                addToggle(correction, frame, horizonFrames, start + width, false);
                candidates.push_back(std::move(correction));
            }
        }
    } else if (mode == VehicleType::Robot) {
        for (int start : {0, 1, 2, 4, 6, 8, 12, 16, 24, 36, 48, 72, 96, 120}) {
            for (int hold : {1, 2, 4, 6, 8, 12, 16, 20, 24, 30, 36}) {
                if (start + hold >= horizonFrames)
                    continue;
                std::set<SearchInput> jump;
                addToggle(jump, frame, horizonFrames, start, false);
                addToggle(jump, frame, horizonFrames, start + hold, false);
                candidates.push_back(std::move(jump));
            }
        }
    } else if (mode == VehicleType::Spider) {
        // Spider only teleports on fresh press edges. Generate explicit click chains
        // with a release after every press, and normalize a held button first so a
        // portal entered while holding cannot poison the whole search window.
        static constexpr int periods[] = {12, 18, 24, 30, 36, 48, 60, 72, 96, 120};
        static constexpr int pulseWidths[] = {1, 2, 3, 4, 6};
        int patternWindow = std::min(horizonFrames, 720);

        for (int period : periods) {
            for (int pulse : pulseWidths) {
                for (int phase : {0, period / 3}) {
                    std::set<SearchInput> clicks;
                    if (heldP1)
                        addToggle(clicks, frame, horizonFrames, 0, false);

                    int start = phase + (heldP1 ? 2 : 0);
                    for (int t = start; t + pulse < patternWindow; t += period) {
                        addToggle(clicks, frame, horizonFrames, t, false);
                        addToggle(clicks, frame, horizonFrames, t + pulse, false);
                    }
                    candidates.push_back(std::move(clicks));
                }
            }
        }

        for (int start : {0, 1, 2, 4, 6, 8, 12, 16, 24, 36, 48, 72, 96}) {
            std::set<SearchInput> click;
            if (heldP1) {
                addToggle(click, frame, horizonFrames, 0, false);
                start = std::max(start, 2);
            }
            addToggle(click, frame, horizonFrames, start, false);
            addToggle(click, frame, horizonFrames, start + 1, false);
            candidates.push_back(std::move(click));
        }
    }

    if (dual) {
        size_t mirrorCount = std::min<size_t>(candidates.size(), 48);
        for (size_t i = 0; i < mirrorCount; ++i) {
            std::set<SearchInput> p2;
            for (SearchInput key : candidates[i]) {
                uint32_t inputFrame = key >> 1;
                p2.insert(inputKey(inputFrame, true));
            }
            candidates.push_back(std::move(p2));
        }
    }

    return candidates;
}

static int workerCountFor(size_t candidateCount) {
    if (candidateCount <= 1)
        return 1;
    unsigned int reported = std::thread::hardware_concurrency();
    if (reported == 0)
        reported = 4;
    return static_cast<int>(std::min<size_t>(
        candidateCount,
        std::clamp<unsigned int>(reported, 2, 6)
    ));
}

static std::vector<TrialResult> evaluateCandidatesParallel(
    Level2 const& base,
    std::vector<std::set<SearchInput>> const& candidates,
    int horizonFrames,
    std::atomic_bool& stop,
    int& workersUsed
) {
    std::vector<TrialResult> results(candidates.size());
    if (candidates.empty()) {
        workersUsed = 1;
        return results;
    }

    workersUsed = workerCountFor(candidates.size());
    if (workersUsed <= 1) {
        Level2 worker = base;
        worker.fixStatePointers();
        for (size_t i = 0; i < candidates.size() && !stop.load(); ++i)
            results[i] = tryInputs(worker, candidates[i], horizonFrames);
        return results;
    }

    std::atomic<size_t> next {0};
    std::vector<std::thread> threads;
    threads.reserve(static_cast<size_t>(workersUsed));

    for (int workerIndex = 0; workerIndex < workersUsed; ++workerIndex) {
        threads.emplace_back([&] {
            Level2 worker = base;
            worker.fixStatePointers();

            while (!stop.load()) {
                size_t index = next.fetch_add(1, std::memory_order_relaxed);
                if (index >= candidates.size())
                    break;
                results[index] = tryInputs(worker, candidates[index], horizonFrames);
            }
        });
    }

    for (auto& thread : threads)
        thread.join();

    return results;
}

PathfinderResult pathfind(
    std::string const& lvlString,
    std::atomic_bool& stop,
    std::function<void(PathfinderTelemetry const&)> callback,
    float trustedEndX
) {
    Level2 lvl(lvlString);

    float solveStartX = lvl.latestState().pos.x;
    float simulatorLength = lvl.length;
    bool hasTrustedEnd = std::isfinite(trustedEndX) &&
                         trustedEndX > solveStartX + 30.f;
    if (hasTrustedEnd) {
        lvl.length = trustedEndX;
        lvl.lengthSource = "trusted-gd";
    }

    std::random_device rd;
    uint32_t baseSeed = rd() ^
        static_cast<uint32_t>(std::hash<std::string>{}(lvlString));
    std::mt19937 rng(baseSeed);

    int trueBestFrame = lvl.currentFrame();
    int fail = 1;
    int numAway = 1000;
    int stagnantRounds = 0;
    int recoveryCount = 0;
    int fullRestarts = 0;
    int hardestSearchLevel = 0;
    int recoveredExceptions = 0;
    int deadCandidatesRejected = 0;
    int randomFallbacks = 0;
    int structuredWins = 0;
    int debugSearchLevel = 0;
    int debugHorizon = 0;
    int debugCandidateCount = 0;
    int debugWorkers = 1;
    int debugVehicleType = static_cast<int>(lvl.latestState().vehicle.type);
    uint64_t totalTrials = 0;

    float furthestX = lvl.latestState().pos.x;
    float lastDeathX = 0.f;
    float debugClearance = 0.f;
    Timeline bestPlayable(lvl);

    auto emitTelemetry = [&](int phase, char const* reason) {
        if (!callback)
            return;

        PathfinderTelemetry telemetry;
        telemetry.progress = progressFor(furthestX, solveStartX, lvl.length, reachedGoal(lvl));
        telemetry.startX = solveStartX;
        telemetry.currentX = lvl.latestState().pos.x;
        telemetry.furthestX = furthestX;
        telemetry.trustedEndX = hasTrustedEnd ? trustedEndX : 0.f;
        telemetry.inferredLength = simulatorLength;
        telemetry.checkpointX = bestPlayable.x();
        telemetry.deathX = lastDeathX;
        telemetry.deathProgress = static_cast<float>(progressFor(lastDeathX, solveStartX, lvl.length, false));
        telemetry.bestClearance = debugClearance;
        telemetry.frame = lvl.currentFrame();
        telemetry.checkpointFrame = bestPlayable.frame();
        telemetry.vehicleType = debugVehicleType;
        telemetry.searchLevel = debugSearchLevel;
        telemetry.horizonFrames = debugHorizon;
        telemetry.candidateCount = debugCandidateCount;
        telemetry.workerCount = debugWorkers;
        telemetry.phase = phase;
        telemetry.totalTrials = totalTrials;
        telemetry.mode = "parallel-safety";
        telemetry.recoveryReason = reason;
        callback(telemetry);
    };

    auto recoverFromBest = [&](int retreatFrames) {
        bestPlayable.restore(lvl);
        int target = std::max(1, bestPlayable.frame() - retreatFrames);
        lvl.rollback(target);
        lvl.syncPresses();
        ++recoveryCount;
        rng.seed(baseSeed ^
                 static_cast<uint32_t>(recoveryCount * 7919) ^
                 static_cast<uint32_t>(lvl.currentFrame()));
    };

    while (!reachedGoal(lvl) && !stop.load()) {
        try {
            if (lvl.latestState().dead) {
                recoverFromBest(std::min(2880, 240 + recoveryCount * 240));
                emitTelemetry(2, "recover-dead-state");
                continue;
            }

            int frame = lvl.currentFrame();
            VehicleType mode = lvl.latestState().vehicle.type;
            int searchLevel = std::min(12, recoveryCount + stagnantRounds / 4);
            hardestSearchLevel = std::max(hardestSearchLevel, searchLevel);

            int horizonFrames = 720 + searchLevel * 80;
            if (flightMode(mode))
                horizonFrames += 240;
            else if (mode == VehicleType::Robot)
                horizonFrames += 120;
            else if (mode == VehicleType::Spider)
                horizonFrames = std::min(horizonFrames, 1200);
            horizonFrames = std::min(horizonFrames, 1920);

            bool dual = lvl.latestState().dualActive;
            auto candidates = structuredCandidates(
                frame,
                horizonFrames,
                mode,
                dual,
                lvl.press1
            );

            debugVehicleType = static_cast<int>(mode);
            debugSearchLevel = searchLevel;
            debugHorizon = horizonFrames;
            debugCandidateCount = static_cast<int>(candidates.size());
            debugClearance = 0.f;
            emitTelemetry(0, "structured-search");

            int workersUsed = 1;
            auto results = evaluateCandidatesParallel(lvl, candidates, horizonFrames, stop, workersUsed);
            debugWorkers = workersUsed;
            totalTrials += results.size();

            std::set<SearchInput> bestInputs;
            TrialResult bestTrial {frame, lvl.latestState().pos.x, 0.f, 0.f, false, false};
            bool haveBest = false;
            size_t bestToggleCount = std::numeric_limits<size_t>::max();

            auto consumeBatch = [&](auto const& batchCandidates, auto const& batchResults) {
                size_t count = std::min(batchCandidates.size(), batchResults.size());
                for (size_t i = 0; i < count; ++i) {
                    auto const& trial = batchResults[i];
                    if (trial.dead)
                        lastDeathX = trial.deathX;

                    if (!betterTrial(
                            trial,
                            batchCandidates[i].size(),
                            haveBest,
                            bestTrial,
                            bestToggleCount,
                            mode
                        )) {
                        if (trial.dead && haveBest && !bestTrial.dead)
                            ++deadCandidatesRejected;
                        continue;
                    }

                    bestTrial = trial;
                    bestToggleCount = batchCandidates[i].size();
                    bestInputs = batchCandidates[i];
                    haveBest = true;
                }
            };

            consumeBatch(candidates, results);

            float strongClearance = flightMode(mode)
                ? 5.5f
                : mode == VehicleType::Spider ? 1.5f
                : 2.5f;
            bool structuredStrong =
                haveBest &&
                !bestTrial.dead &&
                bestTrial.frame >= frame + horizonFrames - 2 &&
                (bestTrial.complete || bestTrial.minClearance >= strongClearance);

            if (structuredStrong) {
                ++structuredWins;
            } else if (!stop.load()) {
                ++randomFallbacks;
                int randomCount = std::min(620, 90 + searchLevel * 45);
                if (mode == VehicleType::Spider)
                    randomCount = std::min(randomCount, 180);

                std::vector<std::set<SearchInput>> randomCandidates;
                randomCandidates.reserve(static_cast<size_t>(randomCount));

                int nearWindow = std::min(horizonFrames, 160 + searchLevel * 45);
                std::uniform_int_distribution<int> farFrame(0, horizonFrames - 1);
                std::uniform_int_distribution<int> nearFrame(0, std::max(0, nearWindow - 1));
                int extraToggles = searchLevel * 2;
                int maxP1 = maxToggleBudget(mode) + extraToggles;
                int maxP2 = dual
                    ? maxToggleBudget(lvl.latestState2().vehicle.type) + extraToggles
                    : 0;
                std::uniform_int_distribution<int> p1Budget(0, maxP1);
                std::uniform_int_distribution<int> p2Budget(0, maxP2);

                for (int attempt = 0; attempt < randomCount; ++attempt) {
                    std::set<SearchInput> inputs;
                    int p1Candidates = p1Budget(rng);
                    int p2Candidates = p2Budget(rng);

                    auto candidateFrame = [&](int index) -> uint32_t {
                        bool useNear = searchLevel > 0 && ((attempt + index) % 5 != 0);
                        int offset = useNear ? nearFrame(rng) : farFrame(rng);
                        return static_cast<uint32_t>(frame + offset);
                    };

                    for (int i = 0; i < p1Candidates; ++i)
                        inputs.insert(inputKey(candidateFrame(i), false));
                    for (int i = 0; i < p2Candidates; ++i)
                        inputs.insert(inputKey(candidateFrame(i + p1Candidates), true));
                    randomCandidates.push_back(std::move(inputs));
                }

                debugCandidateCount = static_cast<int>(candidates.size() + randomCandidates.size());
                emitTelemetry(1, "random-fallback");

                int randomWorkers = 1;
                auto randomResults = evaluateCandidatesParallel(
                    lvl,
                    randomCandidates,
                    horizonFrames,
                    stop,
                    randomWorkers
                );
                debugWorkers = std::max(debugWorkers, randomWorkers);
                totalTrials += randomResults.size();
                consumeBatch(randomCandidates, randomResults);
            }

            if (stop.load())
                break;

            debugClearance = haveBest ? bestTrial.minClearance : 0.f;
            emitTelemetry(1, "candidate-selected");

            if (!haveBest || bestTrial.frame <= frame) {
                ++recoveryCount;
                int preferred = std::max(frame - fail, trueBestFrame - numAway);
                int target = std::clamp(preferred, 1, std::max(1, frame - 1));
                lvl.rollback(target);
                lvl.syncPresses();

                fail += 5;
                if (fail > numAway + 1000) {
                    numAway += 1000;
                    fail = 1;
                    if (numAway > 10000) {
                        numAway = 1000;
                        trueBestFrame = 1;
                        lvl.rollback(1);
                        lvl.syncPresses();
                        ++fullRestarts;
                        rng.seed(baseSeed ^
                                 static_cast<uint32_t>(fullRestarts * 104729) ^
                                 static_cast<uint32_t>(recoveryCount * 7919));
                    }
                } else if (fail > 100) {
                    fail += 50;
                }
                emitTelemetry(2, "rollback-no-advance");
                continue;
            }

            int applyUntil = frame;
            if (bestTrial.complete) {
                applyUntil = bestTrial.frame;
            } else if (!bestTrial.dead) {
                int advance = bestTrial.frame - frame;
                int safetyDivisor = flightMode(mode) ||
                    mode == VehicleType::Robot ||
                    mode == VehicleType::Spider
                    ? 4 : 5;
                int safetyTail = std::max(12, advance / safetyDivisor);
                applyUntil = std::max(frame + 1, bestTrial.frame - safetyTail);
            } else {
                int deathBuffer = std::min(300, 110 + searchLevel * 12);
                int safeEnd = std::max(frame, bestTrial.frame - deathBuffer);
                int safeAdvance = safeEnd - frame;
                applyUntil = frame + safeAdvance / 2;
                ++deadCandidatesRejected;
            }

            if (applyUntil <= frame) {
                recoverFromBest(std::min(2880, 240 + recoveryCount * 240));
                emitTelemetry(2, "recover-zero-prefix");
                continue;
            }

            while (lvl.currentFrame() < applyUntil &&
                   !lvl.latestState().dead &&
                   !reachedGoal(lvl)) {
                uint32_t current = static_cast<uint32_t>(lvl.currentFrame());
                if (bestInputs.contains(inputKey(current, false)))
                    lvl.press1 = !lvl.press1;
                if (bestInputs.contains(inputKey(current, true)))
                    lvl.press2 = !lvl.press2;
                lvl.runFrame(lvl.press1, lvl.press2, 1.f / 240.f);
            }

            if (lvl.latestState().dead) {
                lastDeathX = lvl.latestState().pos.x;
                recoverFromBest(std::min(2880, 360 + recoveryCount * 240));
                emitTelemetry(2, "recover-applied-death");
                continue;
            }

            if (lvl.currentFrame() > trueBestFrame) {
                trueBestFrame = lvl.currentFrame();
                fail = 0;
                numAway = 1000;
            }

            bool advancedX = lvl.latestState().pos.x > furthestX + 1.f;
            if (advancedX || reachedGoal(lvl)) {
                furthestX = std::max(furthestX, lvl.latestState().pos.x);
                bestPlayable.capture(lvl);
                stagnantRounds = 0;
                recoveryCount = std::max(0, recoveryCount - 2);
                emitTelemetry(3, reachedGoal(lvl) ? "complete" : "advance");
            } else if (lvl.latestState().direction < 0) {
                stagnantRounds = 0;
            } else {
                ++stagnantRounds;
            }

            if (stagnantRounds >= 10 &&
                bestPlayable.frame() > 2 &&
                lvl.latestState().direction >= 0) {
                int retreat = std::min(
                    bestPlayable.frame() - 1,
                    420 * (1 + std::min(recoveryCount, 10))
                );
                recoverFromBest(retreat);
                stagnantRounds = 0;
                fail = 1;
                numAway = std::min(8000, 1000 + recoveryCount * 500);
                emitTelemetry(2, "recover-stagnation");
            }
        } catch (std::exception const&) {
            ++recoveredExceptions;
            recoverFromBest(std::min(3600, 480 + recoveredExceptions * 240));
            emitTelemetry(2, "recover-exception");
        } catch (...) {
            ++recoveredExceptions;
            recoverFromBest(std::min(3600, 480 + recoveredExceptions * 240));
            emitTelemetry(2, "recover-unknown-exception");
        }
    }

    if (reachedGoal(lvl) && !lvl.latestState().dead) {
        furthestX = std::max(furthestX, lvl.latestState().pos.x);
        bestPlayable.capture(lvl);
    }

    PathfinderResult result;
    Replay2 output;

    for (size_t i = 1; i < bestPlayable.p1.size(); ++i) {
        auto const& p1 = bestPlayable.p1[i];
        auto const& previousP1 = bestPlayable.p1[i - 1];

        if (p1.frame > 1 && p1.button != previousP1.button) {
            output.inputs.push_back(gdr::Input(p1.frame, 1, false, p1.button));
            result.inputs.push_back({static_cast<uint32_t>(p1.frame), false, p1.button, 1});
        }

        if (i < bestPlayable.p2.size()) {
            auto const& p2 = bestPlayable.p2[i];
            auto const& previousP2 = bestPlayable.p2[i - 1];
            if (p2.dualActive && p2.frame > 1 && p2.button != previousP2.button) {
                output.inputs.push_back(gdr::Input(p2.frame, 1, true, p2.button));
                result.inputs.push_back({static_cast<uint32_t>(p2.frame), true, p2.button, 1});
            }
        }
    }

    std::sort(
        result.inputs.begin(),
        result.inputs.end(),
        [](auto const& a, auto const& b) {
            if (a.frame != b.frame)
                return a.frame < b.frame;
            return a.player2 < b.player2;
        }
    );

    result.macro = output.exportData().unwrapOr({});
    result.complete = bestPlayable.complete();
    result.progress = progressFor(furthestX, solveStartX, lvl.length, result.complete);

    std::ostringstream diagnostics;
    diagnostics
        << "solver=parallel-safety"
        << " progress=" << result.progress
        << " frame=" << bestPlayable.frame()
        << " routeX=" << bestPlayable.x()
        << " furthestX=" << furthestX
        << " endX=" << lvl.length
        << " recoveryCount=" << recoveryCount
        << " fullRestarts=" << fullRestarts
        << " hardestSearch=" << hardestSearchLevel
        << " totalTrials=" << totalTrials
        << " workers=" << debugWorkers
        << " structuredWins=" << structuredWins
        << " randomFallbacks=" << randomFallbacks
        << " deadCandidatesRejected=" << deadCandidatesRejected
        << " lastDeathX=" << lastDeathX
        << " bestClearance=" << debugClearance
        << " recoveredExceptions=" << recoveredExceptions
        << " moveTriggers=" << lvl.supportedMoveTriggers
        << " unsupportedMoves=" << lvl.unsupportedMoveTriggers
        << " movingObjects=" << lvl.movingObjectIDs.size()
        << " inputs=" << result.inputs.size()
        << " complete=" << (result.complete ? 1 : 0)
        << " stopped=" << (stop.load() ? 1 : 0);
    result.diagnostics = diagnostics.str();

    return result;
}
