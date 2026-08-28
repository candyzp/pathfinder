#include <AdvancedObjects.hpp>
#include <Portals.hpp>
#include <Player.hpp>
#include <Level.hpp>
#include <cmath>
#include <algorithm>

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
    : EffectObject(size, std::move(fields)) {
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
    if (p.usedEffects.contains(id))
        return false;

    float a = p.prevPlayer().pos.x;
    float b = p.pos.x;
    float low = std::min(a, b) - 1.0f;
    float high = std::max(a, b) + 1.0f;
    return pos.x >= low && pos.x <= high;
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
            // Full 90-degree gameplay channels require 2D horizontal velocity, which this
            // simulator does not store yet. Horizontal arrow directions are still exact.
            float horizontal = std::cos(deg2rad(value));
            if (std::abs(horizontal) >= 0.7f)
                p.direction = horizontal >= 0 ? 1 : -1;
            break;
        }
    }
}
