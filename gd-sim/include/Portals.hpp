#pragma once
#include <EffectObject.hpp>
#include <Vehicle.hpp>


struct VehiclePortal : public EffectObject {
    VehicleType type;
    VehiclePortal(Vec2D size, std::unordered_map<int, std::string>&& fields);
    void collide(Player&) const override;
};

struct GravityPortal : public EffectObject {
    // 0 = normal gravity, 1 = flipped gravity, 2 = toggle current gravity.
    int mode;
    GravityPortal(Vec2D size, std::unordered_map<int, std::string>&& fields);
    void collide(Player&) const override;
};

struct SizePortal : public EffectObject {
    bool small;
    SizePortal(Vec2D size, std::unordered_map<int, std::string>&& fields);
    void collide(Player&) const override;
};

struct SpeedPortal : public EffectObject {
    int speed;
    SpeedPortal(Vec2D size, std::unordered_map<int, std::string>&& fields);
    void collide(Player&) const override;
};

struct DualPortal : public EffectObject {
    bool enable;
    DualPortal(Vec2D size, std::unordered_map<int, std::string>&& fields);
    void collide(Player&) const override;
};

struct TeleportPortal : public EffectObject {
    float yOffset;
    bool smooth;
    TeleportPortal(Vec2D size, std::unordered_map<int, std::string>&& fields);
    void collide(Player&) const override;
};
