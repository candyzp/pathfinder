#include <AdvancedObjects.hpp>
#include <Portals.hpp>
#include <Player.hpp>
#include <cmath>

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
        case ModifierType::D:
            p.dBlock = true;
            break;
        case ModifierType::J:
            p.jBlock = true;
            // J blocks specifically kill a carried/buffered jump.
            p.buffer = false;
            break;
        case ModifierType::S:
            p.dashing = false;
            break;
        case ModifierType::H:
            p.hBlock = true;
            break;
        case ModifierType::F:
            p.fBlock = true;
            break;
    }
}

ForceBlock::ForceBlock(Vec2D size, std::unordered_map<int, std::string>&& fields)
    : Object(size, std::move(fields)) {
    prio = 0;
    // 2.2 force blocks expose multiple editor modes. Preserve the authored rotation
    // and use a conservative default strength when an older/unknown serialization omits it.
    force = stod_def(fields[10], 1.0f);
    if (std::abs(force) < 0.001f)
        force = 1.0f;
    direction = rotation;
}

void ForceBlock::collide(Player& p) const {
    // The simulator stores only vertical inertia; horizontal force is represented by
    // forward progress, so project the force onto Y. This still makes vertical/angled
    // force blocks affect path viability without corrupting the level's base speed.
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
    // Linked teleport portals serialize the exit displacement on key 54.
    yOffset = stod_def(fields[54], 0.0f);
    smooth = std::stoi(fields[55].empty() ? "0" : fields[55]) != 0;
}

void TeleportPortal::collide(Player& p) const {
    EffectObject::collide(p);
    p.pos.y += yOffset;
    // Smooth teleports preserve velocity; non-smooth ones in GD still preserve the
    // vertical component for standard gameplay, so no velocity reset is needed here.
}
