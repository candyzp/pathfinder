#pragma once

#include <EffectObject.hpp>
#include <util.hpp>

struct Player;

/// Shared Geometry Dash 2.2 teleport settings used by Teleport Triggers,
/// unlinked blue teleport portals, and Teleport Orbs.
struct TeleportConfig {
    int targetGroup = 0;
    int exitGravity = 0; // 0 unset, 1 normal, 2 flipped, 3 toggle

    float staticForceValue = 0.f;
    float redirectForceMin = 0.f;
    float redirectForceMax = 0.f;
    float redirectForceMod = 1.f;

    bool smoothEase = false;
    bool staticForce = false;
    bool staticForceAdditive = false;
    bool redirectForce = false;
    bool saveOffset = false;
    bool ignoreX = false;
    bool ignoreY = false;
    bool snapGround = false;
    bool redirectDash = false;

    TeleportConfig() = default;
    explicit TeleportConfig(std::unordered_map<int, std::string> const& fields);

    /// Apply the authored teleport. sourcePos is the trigger/orb/portal position and
    /// is used by Save Offset. Returns false when the target group cannot be resolved.
    bool apply(Player& player, Vec2D const& sourcePos) const;
};

struct TeleportTrigger : public EffectObject {
    TeleportConfig config;

    TeleportTrigger(Vec2D size, std::unordered_map<int, std::string>&& fields);
    bool touching(Player const&) const override;
    void collide(Player&) const override;
};

struct UnlinkedTeleportPortal : public EffectObject {
    TeleportConfig config;

    UnlinkedTeleportPortal(Vec2D size, std::unordered_map<int, std::string>&& fields);
    void collide(Player&) const override;
};
