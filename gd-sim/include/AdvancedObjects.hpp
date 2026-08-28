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
	bool touchTriggered = false;
	bool spawnTriggered = false;

	GameplayTrigger(Vec2D size, std::unordered_map<int, std::string>&& fields);
	bool touching(Player const&) const override;
	void collide(Player&) const override;
};

/// 2.2 Player Control Trigger. Platformer-only movement controls are retained in the
/// parsed state, while classic Pathfinder uses the jump/slide parts that affect survival.
struct PlayerControlTrigger : public EffectObject {
	bool targetP1;
	bool targetP2;
	bool stopJump;
	bool stopMove;
	bool stopRotation;
	bool stopSlide;
	bool touchTriggered = false;
	bool spawnTriggered = false;

	PlayerControlTrigger(Vec2D size, std::unordered_map<int, std::string>&& fields);
	bool touching(Player const&) const override;
	void collide(Player&) const override;
};
