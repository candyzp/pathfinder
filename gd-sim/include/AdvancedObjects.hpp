#pragma once

#include <Object.hpp>


enum class ModifierType {
	D,
	J,
	S,
	H,
	F
};

/// Invisible letter blocks that alter collision behavior while the player overlaps them.
struct ModifierBlock : public Object {
	ModifierType type;
	ModifierBlock(Vec2D size, std::unordered_map<int, std::string>&& fields);
	void collide(Player&) const override;
};

/// 2.2 force block. Range/relative modes are intentionally approximated by the object's
/// normal direction while still respecting authored force strength where available.
struct ForceBlock : public Object {
	float force;
	float direction;
	ForceBlock(Vec2D size, std::unordered_map<int, std::string>&& fields);
	void collide(Player&) const override;
};
