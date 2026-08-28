#pragma once
#include <EffectObject.hpp>
#include <Teleport.hpp>

enum class OrbType {
	Yellow,
	Blue,
	Pink,
	Red,
	Green,
	Black,
	Dash,
	GravityDash,
	Spider,
	Teleport,
};

struct Orb : public EffectObject {
	OrbType type;
	TeleportConfig teleport;

	Orb(Vec2D size, std::unordered_map<int, std::string>&& fields);
	bool touching(Player const&) const override;
	void collide(Player&) const override;
};
