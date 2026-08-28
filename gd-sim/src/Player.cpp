#include <Player.hpp>
#include <Level.hpp>
#include <Slope.hpp>
#include <cmath>
#include <climits>

Entity Player::innerHitbox() const { return {pos, Vec2D{9, 9}, rotation}; }
Entity Player::unrotatedHitbox() const { return {pos, size, 0}; }

void Player::setVelocity(double v, bool override) {
	velocityOverride = override;
	velocity = v * (small ? 0.8 : 1);
	if (v != 0) grounded = false;
}

Player const& Player::prevPlayer() const { return level->getState(frame - 1, player2); }
Player const* Player::nextPlayer() const { return level->currentFrame() <= frame ? nullptr : &level->getState(frame + 1, player2); }

double roundVel(double velocity, bool upsideDown) {
	double nVel = velocity / 54.0 * (upsideDown * 2 - 1);
	double floored = (int)nVel;
	if (nVel != floored)
		nVel = (double)std::round((nVel - floored) * 1000.0) / 1000.0 + floored;
	return nVel * 54.0 * (upsideDown * 2 - 1);
}

void Player::preCollision(bool pressed) {
	// TimeWarp affects the simulated game timestep, not the search tick itself.
	dt *= std::clamp(timeWarp, 0.05f, 4.0f);

	dBlock = false;
	jBlock = false;
	hBlock = false;
	fBlock = false;

	if (dashing && pressed) {
		float radians = deg2rad(dashAngle);
		pos.x += std::cos(radians) * dashSpeed * dt;
		pos.y += std::sin(radians) * dashSpeed * dt;
		velocity = 0;
		acceleration = 0;
		grounded = false;
	} else {
		if (dashing && !pressed) dashing = false;
		pos.x += direction * player_speeds[(int)speed] * dt;
		pos.y += grav(velocity) * dt;
	}

	frame++;
	timeElapsed += dt;
	grounded = false;
	velocityOverride = false;
	gravityPortal = false;
	roundVelocity = true;

	if (button != pressed) {
		button = pressed;
		input = button;
		buffer = button;
	}

	for (auto& i : actions) i(*this);
	actions.clear();
	potentialSlopes.clear();
	if (slopeData.slope && slopeData.slope->gravOrient(*this) == 1) grounded = true;
}

void Player::postCollision() {
	if (small != prevPlayer().small)
		size = small ? (size * 0.6) : (size / 0.6);

	if (dashing) {
		grounded = false;
		velocity = 0;
		acceleration = 0;
		buffer = false;
		return;
	}

	if (gravBottom(*this) <= gravFloor() && !velocityOverride && velocity <= 0) {
		pos.y = grav(gravFloor()) + grav(size.y / 2);
		grounded = true;
		snapData.playerFrame = 0;
	}

	if (pos.y > 1476.3 || pos.y < -1476.3 || (upsideDown && getBottom() < floor)) {
		dead = true;
		return;
	}

	if (prevPlayer().gravBottom(*this) > prevPlayer().gravFloor() && upsideDown == prevPlayer().upsideDown && !grounded && velocity <= 0) {
		if (prevPlayer().grounded && !prevPlayer().input) coyoteFrames = 0;
		coyoteFrames++;
	} else {
		coyoteFrames = INT_MAX;
	}

	vehicle.update(*this);

	// 2.2 mode corrections from PlayerObject::updateTimeMod/updateJump.
	// Robot uses half the normal yStart for its initial jump, then a 0.9 gravity factor.
	if (vehicle.type == VehicleType::Robot) {
		static constexpr double robotJumpVelocity[5] = {
			286.740864,
			301.8608586,
			308.340864,
			303.210864,
			303.210864
		};
		static constexpr double robotGravity[5] = {
			-2467.4582556,
			-2514.6975186,
			-2512.0730556,
			-2522.5706556,
			-2522.5706556
		};

		bool freshRobotJump = prevPlayer().grounded && input && !jBlock &&
			(!prevPlayer().input || prevPlayer().buffer);
		if (freshRobotJump)
			setVelocity(robotJumpVelocity[speed]);

		// While the jump is actively held, GD adds and removes the same 0.9-gravity
		// increment during the boost phase. In the simulator this is equivalent to
		// cancelling gravity until the short variable-height window ends.
		if (input && velocity > 0 && robotBoostTime < 0.15)
			acceleration = 0;
		else
			acceleration = robotGravity[speed];
	}

	// Swing uses the flying-mode 0.9582 gravity constant at 0.4x normal size and
	// 0.6x mini size. Fresh-input gravity flips remain handled by Vehicle::update.
	if (vehicle.type == VehicleType::Swing)
		acceleration = small ? -1676.46672 : -1117.64448;

	acceleration *= gravityScale;

	if (!velocityOverride) {
		double newVel = velocity + acceleration * dt;
		if (!grounded && prevPlayer().grounded && ((!input && (prevPlayer().button || !button)) || buffer) && prevPlayer().gravBottom(*this) > prevPlayer().gravFloor() && size == prevPlayer().size) {
			pos.y += roundVel(prevPlayer().grav(prevPlayer().acceleration) * dt, prevPlayer().upsideDown) * dt;
			if (gravityPortal && vehicle.type != VehicleType::Ship) newVel = -newVel;
			if (velocity == 0) newVel += roundVel(prevPlayer().acceleration * dt, upsideDown);
		}
		velocity = newVel;
	}

	if (roundVelocity) velocity = roundVel(velocity, upsideDown);
	if (slopeData.slope) slopeData.slope->calc(*this);
	vehicle.clamp(*this);
}

Player::Player() :
	Entity({{0, 15}, {30, 30}, 0}), frame(1), timeElapsed(0), dead(false), completed(false),
	vehicle(Vehicle::from(VehicleType::Cube)), ceiling(999999), floor(0), grounded(true),
	coyoteFrames(0), acceleration(0), velocity(0), robotBoostTime(0),
	dashAngle(0), dashSpeed(0), gravityScale(1.f), timeWarp(1.f), direction(1),
	velocityOverride(false), button(false), input(false), buffer(false), vehicleBuffer(false),
	upsideDown(false), small(false), gravityPortal(false), roundVelocity(true), dashing(false),
	dualActive(false), player2(false), dBlock(false), jBlock(false), hBlock(false), fBlock(false),
	speed(1), slopeData({{}, 0, false}) {}
