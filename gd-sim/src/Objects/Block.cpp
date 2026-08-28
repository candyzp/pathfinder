#include <Block.hpp>
#include <Slope.hpp>
#include <Level.hpp>
#include <Player.hpp>
#include <cmath>
#include <array>
#include <algorithm>

Block::Block(Vec2D s, std::unordered_map<int, std::string>&& fields) : Object(s, std::move(fields)) {
	// Blocks have a prio of 1, so they are processed later than most other objects.
	prio = 1;

	// Cardinal rotations can still use the old fast axis-aligned collision path.
	float normalized = std::fmod(rotation, 360.f);
	if (normalized < 0) normalized += 360.f;
	if (std::abs(normalized - 90.f) < 0.01f || std::abs(normalized - 270.f) < 0.01f) {
		size = {size.y, size.x};
		rotation = 0;
	} else if (normalized < 0.01f || std::abs(normalized - 180.f) < 0.01f || std::abs(normalized - 360.f) < 0.01f) {
		rotation = 0;
	}

	// Edge case for this specific block.
	if (fields[1] == "468" && size.y == 5)
		size.y -= 3.5;
}

float snapThreshold(Vec2D const& diff, Player const& p) {
	std::array<Vec2D, 3> stairs;
	float threshold;

	switch (p.speed) {
	case 0:
		stairs = { Vec2D(120, -30), Vec2D(90, 30), Vec2D(60, 60) };
		threshold = 1;
		break;
	case 1:
		stairs = { Vec2D(150, -30), Vec2D(p.small ? 90 : 120, 30), Vec2D(90, 60) };
		threshold = 1;
		break;
	case 2:
		stairs = { Vec2D(180, -30), Vec2D(p.small ? 90 : 150, 30), Vec2D(120, 60) };
		threshold = 2;
		break;
	case 3:
		stairs = { Vec2D(225, -30), Vec2D(p.small ? 90 : 180, 30), Vec2D(135, 60) };
		threshold = 2;
		break;
	default:
		stairs = { Vec2D(150, -30), Vec2D(120, 30), Vec2D(90, 60) };
		threshold = 1;
		break;
	}

	for (auto& stair : stairs) {
		if (std::abs(diff.x - stair.x) <= threshold && std::abs(diff.y - stair.y) <= threshold)
			return threshold;
	}

	return 0;
}

void trySnap(Block const& b, Player& p) {
	auto snapData = p.snapData;
	auto diff = b.pos - snapData.object.pos;
	diff.y = p.grav(diff.y);

	if (float threshold = snapThreshold(diff, p); threshold > 0) {
		auto next = p.level->getState(snapData.playerFrame, p.player2).nextPlayer();
		if (!next) return;
		p.pos.x = std::clamp(next->pos.x + diff.x, p.pos.x - threshold, p.pos.x + threshold);
	}
}

static bool isCardinal(Block const& b) {
	return std::abs(std::fmod(b.rotation, 90.f)) < 0.01f;
}

// Resolve an arbitrarily rotated solid rectangle in the block's local coordinate system.
// This is deliberately geometric instead of special-casing Cube/UFO/Ball rotations.
static bool collideRotated(Block const& b, Player& p) {
	if (!b.intersects(static_cast<Entity const&>(p)))
		return false;

	float rad = deg2rad(b.rotation);
	float c = std::cos(rad);
	float s = std::sin(rad);
	Vec2D delta = p.pos - b.pos;
	Vec2D local(delta.x * c + delta.y * s, -delta.x * s + delta.y * c);

	float pc = std::abs(c);
	float ps = std::abs(s);
	float playerHalfX = pc * p.size.x * 0.5f + ps * p.size.y * 0.5f;
	float playerHalfY = ps * p.size.x * 0.5f + pc * p.size.y * 0.5f;
	float overlapX = b.size.x * 0.5f + playerHalfX - std::abs(local.x);
	float overlapY = b.size.y * 0.5f + playerHalfY - std::abs(local.y);
	if (overlapX <= 0 || overlapY <= 0)
		return false;

	// Side impact remains lethal for normal ground vehicles. Flying vehicles get a
	// conservative velocity stop instead, matching their existing axis-aligned path.
	if (overlapX < overlapY) {
		if (p.vehicle.type == VehicleType::Ship || p.vehicle.type == VehicleType::Ufo ||
			p.vehicle.type == VehicleType::Ball || p.vehicle.type == VehicleType::Swing) {
			p.velocity = 0;
			return true;
		}
		if (p.vehicle.type == VehicleType::Wave && p.dBlock) {
			p.velocity = 0;
			return true;
		}
		p.dead = true;
		return true;
	}

	float sign = local.y >= 0 ? 1.f : -1.f;
	local.y = sign * (b.size.y * 0.5f + playerHalfY);
	Vec2D resolved(
		b.pos.x + local.x * c - local.y * s,
		b.pos.y + local.x * s + local.y * c
	);

	// Surface normal in world coordinates. A face opposing gravity is a floor,
	// while the other face is a ceiling/head collision.
	Vec2D normal(-sign * s, sign * c);
	float gravityDown = p.upsideDown ? 1.f : -1.f;
	bool floorFace = normal.y * gravityDown < 0;

	if (floorFace) {
		p.pos = resolved;
		p.velocity = 0;
		p.grounded = true;
		return true;
	}

	if (p.hBlock) {
		p.pos = resolved;
		p.velocity = 0;
		return true;
	}
	if (p.fBlock && (p.vehicle.type == VehicleType::Cube || p.vehicle.type == VehicleType::Robot || p.vehicle.type == VehicleType::Spider)) {
		p.pos = resolved;
		p.upsideDown = !p.upsideDown;
		p.velocity = 0;
		p.gravityPortal = true;
		return true;
	}

	if (p.vehicle.type == VehicleType::Ship || p.vehicle.type == VehicleType::Ufo ||
		p.vehicle.type == VehicleType::Ball || p.vehicle.type == VehicleType::Swing) {
		p.pos = resolved;
		p.velocity = 0;
		return true;
	}

	p.dead = true;
	return true;
}

void Block::collide(Player& p) const {
	if (!isCardinal(*this)) {
		collideRotated(*this, p);
		return;
	}

	int clip = (p.vehicle.type == VehicleType::Ufo || p.vehicle.type == VehicleType::Ship || p.vehicle.type == VehicleType::Swing) ? 7 : 10;

	if (p.upsideDown != p.prevPlayer().upsideDown && !p.gravityPortal)
		return;

	double bottom = p.gravBottom(p);
	if (p.slopeData.slope) {
		if (p.slopeData.slope->angle() > 0) {
			bottom = bottom + sin(p.slopeData.slope->angle()) * p.size.y / 2;
			clip = 7;
			if (p.gravTop(*this) - bottom < 2)
				return;
		}
	}

	for (auto& entity : p.potentialSlopes) {
		auto block_comp = entity->orientation < 2 ? getTop() : getBottom();
		auto slope_comp = entity->orientation < 2 ? entity->getBottom() : entity->getTop();
		if (block_comp - slope_comp < 2)
			return;
	}

	bool padHitBefore = (!p.prevPlayer().grounded && p.prevPlayer().velocity <= 0 && p.velocity > 0);

	if (p.innerHitbox().intersects(*this)) {
		if (p.vehicle.type == VehicleType::Wave && p.dBlock) {
			p.velocity = 0;
			return;
		}

		if (p.hBlock) {
			p.velocity = 0;
			return;
		}

		if (p.fBlock && (p.vehicle.type == VehicleType::Cube || p.vehicle.type == VehicleType::Robot || p.vehicle.type == VehicleType::Spider)) {
			p.upsideDown = !p.upsideDown;
			p.velocity = 0;
			p.gravityPortal = true;
			return;
		}

		p.dead = true;
	} else if ((p.vehicle.type != VehicleType::Wave || p.dBlock) && p.gravTop(*this) - bottom <= clip && (padHitBefore || p.velocity <= 0 || p.gravityPortal)) {
		p.pos.y = p.grav(p.gravTop(*this)) + p.grav(p.size.y / 2);

		if (!padHitBefore)
			p.grounded = true;

		if (p.slopeData.slope && p.slopeData.slope->angle() < 0)
			p.slopeData.slope = {};

		if (p.vehicle.type == VehicleType::Cube || p.vehicle.type == VehicleType::Robot) {
			if (!p.prevPlayer().grounded && p.snapData.playerFrame > 0 && p.snapData.playerFrame + 1 < p.frame)
				trySnap(*this, p);

			p.snapData.playerFrame = p.level->currentFrame();
			p.snapData.object = *this;
		}
	} else {
		if (p.vehicle.type == VehicleType::Ship || p.vehicle.type == VehicleType::Ufo ||
			p.vehicle.type == VehicleType::Ball || p.vehicle.type == VehicleType::Swing) {
			if (p.gravTop(p) - p.gravBottom(*this) <= clip - 1 && p.velocity > 0) {
				p.pos.y = p.grav(p.gravBottom(*this)) - p.grav(p.size.y / 2);
				p.velocity = 0;
			}
		}
	}
}
