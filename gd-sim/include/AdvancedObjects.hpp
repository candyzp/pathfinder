#pragma once

#include <Object.hpp>
#include <EffectObject.hpp>

enum class ModifierType {
	D,
	J,
	S,
	H,
	F
};

struct ModifierBlock : public Object {
	ModifierType type;
	ModifierBlock(Vec2D size, std::unordered_map<int, std::string>&& fields);
	void collide(Player&) const override;
};

struct ForceBlock : public Object {
	float force;
	float direction;
	ForceBlock(Vec2D size, std::unordered_map<int, std::string>&& fields);
	void collide(Player&) const override;
};

enum class GameplayTriggerType {
	Gravity,
	TimeWarp,
	Reverse,
	End,
	GameplayRotation
};

struct GameplayTrigger : public EffectObject {
	GameplayTriggerType type;
	float value;
	GameplayTrigger(Vec2D size, std::unordered_map<int, std::string>&& fields);
	bool touching(Player const&) const override;
	void collide(Player&) const override;
};
