#include <Slope.hpp>
#include <cmath>
#include <algorithm>
#include <Player.hpp>

Slope::Slope(Vec2D size, std::unordered_map<int, std::string>&& fields) : Block(size, std::move(fields)) {
	auto rot = stod_def(fields[6].c_str());

	bool flipX = atoi(fields[4].c_str()) == 1;
	bool flipY = atoi(fields[5].c_str()) == 1;

	orientation = static_cast<int>(rot / 90);
	if (flipX && flipY)
		orientation += 2;
	else if (flipX)
		orientation += 1;
	else if (flipY)
		orientation += 3;

	orientation %= 4;
	if (orientation < 0) orientation += 4;
	rotation = 0;
}

bool Slope::isFacingUp() const {
	return orientation < 2;
}

int Slope::gravOrient(Player const& p) const {
	int orient = orientation;
	if (p.upsideDown) {
		switch (orient) {
			case 3: orient = 0; break;
			case 2: orient = 1; break;
			case 0: orient = 3; break;
			case 1: orient = 2; break;
		}
	}
	return orient;
}

double Slope::angle() const {
	auto ang = std::atan(size.y / size.x);
	if (orientation == 1 || orientation == 3)
		ang = -ang;
	return ang;
}

double Slope::expectedY(Player const& p) const {
	double ydist = (isFacingUp() ? 1 : -1) * (p.size.y / 2.f) / std::cos(angle());
	float posRelative = (size.y / size.x) * (p.pos.x - getLeft());

	if (angle() > 0)
		return getBottom() + std::min(posRelative + ydist, size.y + p.size.y / 2.0);
	return getTop() - std::min(posRelative - ydist, size.y + p.size.y / 2.0);
}

static void clearSlopeNext(Player& p) {
	p.actions.push_back(+[](Player& next) {
		next.slopeData.slope = {};
		next.slopeData.elapsed = 0.0;
		next.slopeData.snapDown = false;
	});
}

void Slope::calc(Player& p) const {
	int rel = gravOrient(p.prevPlayer());

	if (rel == 0) {
		// Uphill supporting slope, identical for normal and upside-down gravity.
		if (!touching(p))
			clearSlopeNext(p);

		double target = expectedY(p);
		if (p.grav(p.pos.y) < p.grav(target))
			p.pos.y = target;

		if (p.grounded && p.gravBottom(p) >= p.gravTop(*this)) {
			double gravAngle = p.grav(angle());
			double safeAngle = std::max(std::abs(gravAngle), 0.001);
			double vel = 0.9 * std::min(1.12 / safeAngle, 1.54) * (size.y * player_speeds[p.speed] / size.x);
			double time = std::clamp(10 * (p.timeElapsed - p.slopeData.elapsed), 0.4, 1.0);

			if (p.vehicle.type == VehicleType::Ball || p.vehicle.type == VehicleType::Ship || p.vehicle.type == VehicleType::Swing)
				vel *= 0.75;
			if (p.vehicle.type == VehicleType::Ufo)
				vel *= 0.7499;
			vel *= time;

			p.actions.push_back([vel](Player& next) {
				next.velocity = roundVel(vel, next.upsideDown);
				next.slopeData.slope = {};
				next.slopeData.elapsed = 0;
				next.slopeData.snapDown = false;
			});
		}
		return;
	}

	if (rel == 1) {
		// Downhill supporting slope.
		if (p.velocity > 0)
			clearSlopeNext(p);

		double target = expectedY(p);
		if (p.grav(p.pos.y) < p.grav(target))
			p.pos.y = target;

		// Once the player leaves the downhill face, inherit a downward component.
		if (p.pos.x >= getRight() - p.size.x * 0.25f) {
			static double falls[5] = {226.044054, 280.422108, 348.678108, 421.200108, 518.4};
			double vel = -falls[std::clamp(p.speed, 0, 4)] * (size.y / size.x);
			p.velocity = 0;
			p.actions.push_back([vel](Player& next) {
				next.velocity = vel;
				next.slopeData.slope = {};
				next.slopeData.elapsed = 0;
				next.slopeData.snapDown = false;
			});
		}
		return;
	}

	// Relative orientations 2 and 3 are the two ceiling-facing slope directions.
	// The old simulator implemented only orientation 2, which is why upside-down
	// traversal worked in one direction and failed in the other.
	if (rel == 2 || rel == 3) {
		if (p.velocity < 0) {
			clearSlopeNext(p);
			return;
		}

		p.velocity = 0;
		double target = expectedY(p);
		if (p.grav(p.pos.y) > p.grav(target))
			p.pos.y = target;

		bool reachedExit = rel == 2
			? p.pos.x >= getRight() - p.size.x * 0.25f
			: p.pos.x <= getLeft() + p.size.x * 0.25f;
		if (reachedExit) {
			p.velocity = roundVel(p.prevPlayer().acceleration * p.dt, p.prevPlayer().upsideDown);
			clearSlopeNext(p);
		}
	}
}

void Slope::collide(Player& p) const {
	p.potentialSlopes.push_back(this);
	int rel = gravOrient(p);
	double target = expectedY(p);

	// Compare in gravity-relative coordinates so both ceiling slope orientations work
	// with upside-down gravity exactly like their floor counterparts.
	if (rel < 2) {
		if (p.grav(target) <= p.grav(p.pos.y))
			return;
	} else {
		if (p.grav(target) >= p.grav(p.pos.y))
			return;
	}

	if ((p.vehicle.type == VehicleType::Cube || p.vehicle.type == VehicleType::Robot || p.vehicle.type == VehicleType::Spider) &&
		p.gravTop(p) - p.gravBottom(*this) < 16)
		return;

	if (p.vehicle.type == VehicleType::Wave && !p.dBlock) {
		p.dead = true;
		return;
	}

	// Entering a supporting downhill slope before the center reaches it is block-like.
	if (!p.prevPlayer().slopeData.slope && rel == 1 && p.velocity <= 0 && p.pos.x - getLeft() < 0) {
		p.pos.y = target;
		p.grounded = true;
		return;
	}

	if (!p.prevPlayer().slopeData.slope && rel >= 2 && p.velocity >= 0 && p.pos.x - getLeft() < 0) {
		p.pos.y = target;
		p.velocity = 0;
		return;
	}

	auto pSlope = p.slopeData.slope;
	if (!pSlope || !pSlope->touching(p) ||
		(pSlope->gravOrient(p) == rel && p.grav(expectedY(p)) > p.grav(pSlope->expectedY(p))) ||
		pSlope->id == id) {
		bool hasSlope = p.prevPlayer().slopeData.slope.has_value();

		double pAngle = std::atan((p.prevPlayer().velocity * p.dt) / (player_speeds[p.speed] * p.dt));
		if (rel > 1)
			pAngle = -pAngle;

		bool projectedHit = (rel == 1 || rel == 3) ? (pAngle * 5.0 <= p.grav(angle())) : (pAngle <= p.grav(angle()));
		bool snapDown = rel == 1 && p.velocity > 0 && p.pos.x - getLeft() > 0;

		if (hasSlope ? p.velocity <= 0 : projectedHit || snapDown) {
			p.grounded = true;
			p.slopeData.slope = *this;

			if (snapDown && !hasSlope) {
				p.velocity = 0;
				p.pos.y = target;
				p.slopeData.snapDown = true;
			}

			if (!p.slopeData.elapsed)
				p.slopeData.elapsed = p.prevPlayer().timeElapsed;
		}
	}
}

void SlopeHazard::collide(Player& p) const {
	int rel = gravOrient(p);
	if (rel < 2) {
		if (p.grav(expectedY(p)) <= p.grav(p.pos.y)) return;
	} else {
		if (p.grav(expectedY(p)) >= p.grav(p.pos.y)) return;
	}
	p.dead = true;
}

double SlopeHazard::expectedY(Player const& p) const {
	return Slope::expectedY(p) + (orientation > 1 ? -4 : 4);
}

bool SlopeHazard::touching(Player const& p) const {
	Entity hitbox = p.unrotatedHitbox();
	hitbox.size.y += 8;
	if (!intersects(hitbox))
		return false;

	int rel = gravOrient(p);
	return rel < 2
		? p.grav(expectedY(p)) > p.grav(p.pos.y)
		: p.grav(expectedY(p)) < p.grav(p.pos.y);
}
