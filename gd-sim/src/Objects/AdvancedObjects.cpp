#include <AdvancedObjects.hpp>
#include <Portals.hpp>
#include <Player.hpp>
#include <Level.hpp>
#include <cmath>
#include <algorithm>

namespace {
bool crossesTriggerX(Object const& trigger, Player const& p) {
    if (p.usedEffects.contains(trigger.id))
        return false;

    float a = p.prevPlayer().pos.x;
    float b = p.pos.x;
    float low = std::min(a, b) - 1.0f;
    float high = std::max(a, b) + 1.0f;
    return trigger.pos.x >= low && trigger.pos.x <= high;
}

bool boolField(std::unordered_map<int, std::string> const& fields, int key) {
    auto it = fields.find(key);
    return it != fields.end() && !it->second.empty() && atoi(it->second.c_str()) != 0;
}
}

ModifierBlock::ModifierBlock(Vec2D size, std::unordered_map<int, std::string>&& fields)
    : Object(size, std::move(fields)) {
    prio = 0;
    switch (std::stoi(fields[1])) {
        case 1755: type = ModifierType::D; break;
        case 1813: type = ModifierType::J; break;
        case 1829: type = ModifierType::S; break;
        case 1859: type = ModifierType::H; break;
        case 2866: type = ModifierType::F; break;
        default: type = ModifierType::D; break;
    }
}

void ModifierBlock::collide(Player& p) const {
    switch (type) {
        case ModifierType::D: p.dBlock = true; break;
        case ModifierType::J:
            p.jBlock = true;
            p.buffer = false;
            break;
        case ModifierType::S: p.dashing = false; break;
        case ModifierType::H: p.hBlock = true; break;
        case ModifierType::F: p.fBlock = true; break;
    }
}

ForceBlock::ForceBlock(Vec2D size, std::unordered_map<int, std::string>&& fields)
    : Object(size, std::move(fields)) {
    prio = 0;
    force = stod_def(fields[10], 1.0f);
    if (std::abs(force) < 0.001f) force = 1.0f;
    direction = rotation;
}

void ForceBlock::collide(Player& p) const {
    p.velocity += std::sin(deg2rad(direction)) * 900.0f * force * p.dt;
}

DualPortal::DualPortal(Vec2D size, std::unordered_map<int, std::string>&& fields)
    : EffectObject(size, std::move(fields)), enable(std::stoi(fields[1]) == 286) {}

void DualPortal::collide(Player& p) const {
    EffectObject::collide(p);
    p.dualActive = enable;
}

TeleportPortal::TeleportPortal(Vec2D size, std::unordered_map<int, std::string>&& fields)
    : EffectObject(size, std::move(fields)) {
    yOffset = stod_def(fields[54], 0.0f);
    smooth = !fields[55].empty() && std::stoi(fields[55]) != 0;
}

void TeleportPortal::collide(Player& p) const {
    EffectObject::collide(p);
    p.pos.y += yOffset;
}

GameplayTrigger::GameplayTrigger(Vec2D size, std::unordered_map<int, std::string>&& fields)
    : EffectObject(size, std::unordered_map<int, std::string>(fields)) {
    touchTriggered = boolField(fields, 11);
    spawnTriggered = boolField(fields, 62);

    switch (std::stoi(fields[1])) {
        case 2066:
            type = GameplayTriggerType::Gravity;
            value = stod_def(fields[148], 1.0f);
            break;
        case 1935:
            type = GameplayTriggerType::TimeWarp;
            value = stod_def(fields[120], 1.0f);
            break;
        case 1917:
            type = GameplayTriggerType::Reverse;
            value = 0;
            break;
        case 1931:
        case 3600:
            type = GameplayTriggerType::End;
            value = 0;
            break;
        case 2900:
            type = GameplayTriggerType::GameplayRotation;
            value = rotation;
            break;
        default:
            type = GameplayTriggerType::Gravity;
            value = 1.0f;
            break;
    }
}

bool GameplayTrigger::touching(Player const& p) const {
    // Spawn-triggered effects are remote/group-controlled in real GD. Until the
    // simulator has a full spawn graph, auto-firing them on X-crossing is much
    // worse than leaving them dormant because it can cause fake teleports/endings.
    if (spawnTriggered)
        return false;

    // Touch-triggered effects care about actual player overlap, including Y.
    if (touchTriggered)
        return EffectObject::touching(p);

    // Normal gameplay triggers fire when the player crosses their X position.
    return crossesTriggerX(*this, p);
}

void GameplayTrigger::collide(Player& p) const {
    EffectObject::collide(p);

    switch (type) {
        case GameplayTriggerType::Gravity:
            p.gravityScale = std::clamp(value, 0.0f, 10.0f);
            break;
        case GameplayTriggerType::TimeWarp:
            p.timeWarp = std::clamp(value, 0.05f, 4.0f);
            break;
        case GameplayTriggerType::Reverse:
            p.direction *= -1;
            break;
        case GameplayTriggerType::End:
            p.pos.x = p.level->length + 1.0f;
            break;
        case GameplayTriggerType::GameplayRotation: {
            // Full vertical channels need 2D horizontal velocity. Horizontal rotations
            // are exact here and vertical channels remain intentionally conservative.
            float horizontal = std::cos(deg2rad(value));
            if (std::abs(horizontal) >= 0.7f)
                p.direction = horizontal >= 0 ? 1 : -1;
            break;
        }
    }
}

PlayerControlTrigger::PlayerControlTrigger(Vec2D size, std::unordered_map<int, std::string>&& fields)
    : EffectObject(size, std::unordered_map<int, std::string>(fields)) {
    targetP1 = boolField(fields, 138);
    targetP2 = boolField(fields, 200);
    if (!targetP1 && !targetP2) {
        targetP1 = true;
        targetP2 = true;
    }

    stopJump = boolField(fields, 540);
    stopMove = boolField(fields, 541);
    stopRotation = boolField(fields, 542);
    stopSlide = boolField(fields, 543);
    touchTriggered = boolField(fields, 11);
    spawnTriggered = boolField(fields, 62);
}

bool PlayerControlTrigger::touching(Player const& p) const {
    if ((p.player2 && !targetP2) || (!p.player2 && !targetP1))
        return false;
    if (spawnTriggered)
        return false;
    if (touchTriggered)
        return EffectObject::touching(p);
    return crossesTriggerX(*this, p);
}

void PlayerControlTrigger::collide(Player& p) const {
    EffectObject::collide(p);

    if (stopJump) {
        p.buffer = false;
        p.vehicleBuffer = false;
        p.input = false;
    }

    // stopMove is a platformer control and Pathfinder's classic solver has no manual
    // horizontal axis, so it must not stop automatic level progression.
    (void)stopMove;

    if (stopRotation)
        p.rotation = 0;

    if (stopSlide) {
        p.slopeData.slope = {};
        p.slopeData.elapsed = 0;
        p.slopeData.snapDown = false;
    }
}
