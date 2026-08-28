#include <Teleport.hpp>
#include <Player.hpp>
#include <Level.hpp>

#include <algorithm>
#include <cmath>

namespace {
bool fieldBool(std::unordered_map<int, std::string> const& fields, int key) {
    auto it = fields.find(key);
    return it != fields.end() && !it->second.empty() && atoi(it->second.c_str()) != 0;
}

float fieldFloat(std::unordered_map<int, std::string> const& fields, int key, float fallback = 0.f) {
    auto it = fields.find(key);
    return it == fields.end() ? fallback : stod_def(it->second, fallback);
}

int fieldInt(std::unordered_map<int, std::string> const& fields, int key, int fallback = 0) {
    auto it = fields.find(key);
    return it == fields.end() || it->second.empty() ? fallback : atoi(it->second.c_str());
}
}

TeleportConfig::TeleportConfig(std::unordered_map<int, std::string> const& fields) {
    targetGroup = fieldInt(fields, 51);
    smoothEase = fieldBool(fields, 55);

    staticForce = fieldBool(fields, 345);
    staticForceValue = fieldFloat(fields, 346);
    redirectForce = fieldBool(fields, 347);
    redirectForceMin = fieldFloat(fields, 348);
    redirectForceMax = fieldFloat(fields, 349);
    redirectForceMod = fieldFloat(fields, 350, 1.f);
    if (std::abs(redirectForceMod) < 0.0001f)
        redirectForceMod = 1.f;

    saveOffset = fieldBool(fields, 351);
    ignoreX = fieldBool(fields, 352);
    ignoreY = fieldBool(fields, 353);
    exitGravity = std::clamp(fieldInt(fields, 354), 0, 3);
    staticForceAdditive = fieldBool(fields, 443);
    snapGround = fieldBool(fields, 510);
    redirectDash = fieldBool(fields, 591);
}

bool TeleportConfig::apply(Player& p, Vec2D const& sourcePos) const {
    // A 2.2 Teleport Trigger target is supposed to resolve to one target object.
    // The old simulator used Level::getGroupTarget(), which picks an arbitrary
    // member when a group contains several objects. On decorated levels that can
    // fling the simulated player thousands of units forward and permanently poison
    // Pathfinder's furthest-X progress/checkpoint. Fail closed until a real dynamic
    // group/parent resolver exists instead of pretending an arbitrary object is valid.
    if (targetGroup <= 0)
        return false;

    auto groupIt = p.level->groupTargets.find(targetGroup);
    if (groupIt == p.level->groupTargets.end() || groupIt->second.size() != 1)
        return false;

    Entity target = groupIt->second.front();
    Vec2D before = p.pos;
    Vec2D destination = target.pos;

    if (saveOffset)
        destination += before - sourcePos;
    if (ignoreX)
        destination.x = before.x;
    if (ignoreY)
        destination.y = before.y;

    p.pos = destination;

    switch (exitGravity) {
        case 1:
            if (p.upsideDown) {
                p.upsideDown = false;
                p.velocity = -p.velocity;
            }
            break;
        case 2:
            if (!p.upsideDown) {
                p.upsideDown = true;
                p.velocity = -p.velocity;
            }
            break;
        case 3:
            p.upsideDown = !p.upsideDown;
            p.velocity = -p.velocity;
            break;
        default:
            break;
    }

    float targetRadians = deg2rad(target.rotation);
    float verticalDirection = std::sin(targetRadians);

    if (staticForce) {
        double authoredVelocity = staticForceValue * 54.0 * verticalDirection;
        if (staticForceAdditive)
            p.velocity += authoredVelocity;
        else
            p.setVelocity(authoredVelocity, true);
    } else if (redirectForce) {
        double redirected = p.velocity * redirectForceMod;
        if (redirectForceMin != 0.f || redirectForceMax != 0.f) {
            double low = std::min<double>(redirectForceMin, redirectForceMax) * 54.0;
            double high = std::max<double>(redirectForceMin, redirectForceMax) * 54.0;
            if (low != high)
                redirected = std::clamp(redirected, low, high);
        }
        if (std::abs(verticalDirection) > 0.001f)
            redirected = std::abs(redirected) * verticalDirection;
        p.setVelocity(redirected, true);
    }

    if (redirectDash && p.dashing)
        p.dashAngle = target.rotation;

    if (snapGround) {
        bool towardsCeiling = p.upsideDown;
        float surface = p.level->findOppositeSurface(p, towardsCeiling);
        if (std::isfinite(surface)) {
            p.pos.y = towardsCeiling
                ? surface - p.size.y / 2.f
                : surface + p.size.y / 2.f;
            p.velocity = 0;
            p.grounded = true;
            p.velocityOverride = true;
        }
    }

    return true;
}

TeleportTrigger::TeleportTrigger(Vec2D size, std::unordered_map<int, std::string>&& fields)
    : EffectObject(size, std::unordered_map<int, std::string>(fields)), config(fields) {
    touchTriggered = fieldBool(fields, 11);
    spawnTriggered = fieldBool(fields, 62);
}

bool TeleportTrigger::touching(Player const& p) const {
    if (p.usedEffects.contains(id))
        return false;

    if (spawnTriggered)
        return false;

    if (touchTriggered)
        return EffectObject::touching(p);

    float previousX = p.prevPlayer().pos.x;
    float currentX = p.pos.x;
    float low = std::min(previousX, currentX) - 1.f;
    float high = std::max(previousX, currentX) + 1.f;
    return pos.x >= low && pos.x <= high;
}

void TeleportTrigger::collide(Player& p) const {
    if (config.apply(p, pos))
        EffectObject::collide(p);
}

UnlinkedTeleportPortal::UnlinkedTeleportPortal(Vec2D size, std::unordered_map<int, std::string>&& fields)
    : EffectObject(size, std::unordered_map<int, std::string>(fields)), config(fields) {
}

void UnlinkedTeleportPortal::collide(Player& p) const {
    if (config.apply(p, pos))
        EffectObject::collide(p);
}
