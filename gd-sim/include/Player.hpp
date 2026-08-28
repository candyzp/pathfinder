#pragma once

#undef small
#include <util.hpp>
#include <Vehicle.hpp>
#include <Slope.hpp>
#include <vector>
#include <functional>
#include <optional>
#include <algorithm>

/// Player X velocity per speed
inline double player_speeds[5] = {
	251.16007972276924,
	311.580093712804,
	387.42014039710523,
	468.0001388338566,
	576.00020058307177
};

inline float player_speedmults[5] = {
	0.7,
	0.9,
	1.1,
	1.3,
	1.6
};

double roundVel(double velocity, bool upsideDown);

struct Object;
class Level;
struct Slope;

struct Player : public Entity {
	Vehicle vehicle;
	Level* level;

	double timeElapsed;
	double acceleration;
	double velocity;

	/// 2.2 gameplay state.
	double robotBoostTime;
	float dashAngle;
	float dashSpeed;
	float gravityScale;
	float timeWarp;
	int direction;

	cow_set<int> usedEffects;
	std::vector<Slope const*> potentialSlopes;
	std::vector<std::function<void(Player&)>> actions;

	struct {
		std::optional<Slope> slope;
		double elapsed;
		bool snapDown;
	} slopeData;

	struct {
		Entity object;
		int playerFrame = 0;
	} snapData;

	float ceiling;
	float floor;
	float dt;

	unsigned int coyoteFrames;
	int speed;
	int frame;

	bool dead;
	bool completed;
	bool grounded;
	bool velocityOverride;
	bool button, input;
	bool buffer;
	bool vehicleBuffer;
	bool upsideDown;
	bool small;
	bool gravityPortal;
	bool roundVelocity;
	bool dashing;
	bool dualActive;
	bool player2;

	/// Letter blocks are evaluated before solid blocks each frame.
	bool dBlock;
	bool jBlock;
	bool hBlock;
	bool fBlock;

	Player();

	void preCollision(bool input);
	void postCollision();

	Entity unrotatedHitbox() const;
	Entity innerHitbox() const;

	Player const& prevPlayer() const;
	Player const* nextPlayer() const;

	template <typename T>
	T grav(T value) const { return upsideDown ? -value : value; }
	inline float gravBottom(Entity const& e) const { return upsideDown ? -e.getTop() : e.getBottom(); }
	inline float gravTop(Entity const& e) const { return upsideDown ? -e.getBottom() : e.getTop(); }
	inline float gravFloor() const { return upsideDown ? -ceiling : floor; }
	inline float gravCeiling() const { return upsideDown ? -floor : ceiling; }

	void setVelocity(double v, bool override=false);
};
