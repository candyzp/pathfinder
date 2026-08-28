#include <Orb.hpp>
#include <Player.hpp>
#include <Level.hpp>
#include <cmath>

Orb::Orb(Vec2D size, std::unordered_map<int, std::string>&& fields)
	: EffectObject(size, std::unordered_map<int, std::string>(fields)), teleport(fields) {
	switch (std::stoi(fields[1])) {
		case 36: type = OrbType::Yellow; break;
		case 84: type = OrbType::Blue; break;
		case 141: type = OrbType::Pink; break;
		case 1333: type = OrbType::Red; break;
		case 1330: type = OrbType::Black; break;
		case 1022: type = OrbType::Green; break;
		case 1704: type = OrbType::Dash; break;
		case 1751: type = OrbType::GravityDash; break;
		case 3004: type = OrbType::Spider; break;
		case 3027: type = OrbType::Teleport; break;
		default: type = OrbType::Yellow; break;
	};
}

bool Orb::touching(Player const& p) const {
	// Coyote frame for orbs
	return EffectObject::touching(p) || EffectObject::touching(p.prevPlayer());
}

const velocity_map<OrbType, VehicleType, bool> orb_velocities = {
	{{OrbType::Yellow, VehicleType::Cube, false}, {573.48, 603.72, 616.68, 606.42}},
	{{OrbType::Yellow, VehicleType::Cube, true}, {458.784, 482.976, 481.734, 485.136}},
	{{OrbType::Yellow, VehicleType::Ship, false}, {573.48, 603.72, 616.68, 606.42}},
	{{OrbType::Yellow, VehicleType::Ship, true}, {458.784, 482.976, 481.734, 485.136}},
	{{OrbType::Yellow, VehicleType::Ball, false}, {401.435993, 422.60399, 431.67599, 424.493993}},
	{{OrbType::Yellow, VehicleType::Ball, true}, {321.148795, 338.08319, 345.34079, 339.59519}},
	{{OrbType::Yellow, VehicleType::Ufo, false}, {573.48, 603.72, 616.68, 606.42}},
	{{OrbType::Yellow, VehicleType::Ufo, true}, {458.784, 482.976, 481.734, 485.136}},

	{{OrbType::Blue, VehicleType::Cube, false}, {-229.392, -241.488, -246.672, -242.568}},
	{{OrbType::Blue, VehicleType::Cube, true}, {-183.519, -193.185, -197.343, -194.049}},
	{{OrbType::Blue, VehicleType::Ship, false}, {-229.392, -241.488, -246.672, -242.568}},
	{{OrbType::Blue, VehicleType::Ship, true}, {-183.519, -193.185, -197.343, -194.049}},
	{{OrbType::Blue, VehicleType::Ball, false}, {-160.574397, -169.04160, -172.6704, -169.7976}},
	{{OrbType::Blue, VehicleType::Ball, true}, {-128.463298, -135.2295, -138.1401, -135.8343}},
	{{OrbType::Blue, VehicleType::Ufo, false}, {-229.392, -241.48, -246.672, -242.568}},
	{{OrbType::Blue, VehicleType::Ufo, true}, {-183.519, -193.185, -197.343, -194.049}},

	{{OrbType::Pink, VehicleType::Cube, false}, {412.884, 434.7, 443.988, 436.644}},
	{{OrbType::Pink, VehicleType::Cube, true}, {330.318, 347.76, 355.212, 349.272}},
	{{OrbType::Pink, VehicleType::Ship, false}, {212.166, 223.398, 228.15, 224.37}},
	{{OrbType::Pink, VehicleType::Ship, true}, {169.776, 178.686, 182.52, 179.496}},
	{{OrbType::Pink, VehicleType::Ball, false}, {309.090595, 325.42019, 332.37539, 326.85659}},
	{{OrbType::Pink, VehicleType::Ball, true}, {247.287596, 260.3286, 265.923, 261.5004}},
	{{OrbType::Pink, VehicleType::Ufo, false}, {240.84, 253.584, 258.984, 254.718}},
	{{OrbType::Pink, VehicleType::Ufo, true}, {192.672, 202.824, 207.198, 203.742}},

	{{OrbType::Red, VehicleType::Cube, false}, {779.976, 821.448, 839.43, 825.174}},
	{{OrbType::Red, VehicleType::Cube, true}, {621.702, 654.858, 669.222, 657.828}},
	{{OrbType::Red, VehicleType::Ship, false}, {569.754, 599.994, 612.954, 602.694}},
	{{OrbType::Red, VehicleType::Ship, true}, {637.902, 671.814, 686.286, 674.838}},
	{{OrbType::Red, VehicleType::Ball, false}, {530.928, 559.278, 571.482, 561.816}},
	{{OrbType::Red, VehicleType::Ball, true}, {423.36, 446.04, 455.76, 448.092}},
	{{OrbType::Red, VehicleType::Ufo, false}, {577.962, 608.85, 622.026, 611.604}},
	{{OrbType::Red, VehicleType::Ufo, true}, {615.762, 648.648, 662.742, 651.564}},

	{{OrbType::Green, VehicleType::Cube, false}, {562.032, 592.056, 605.07, 594.756}},
	{{OrbType::Green, VehicleType::Cube, true}, {447.336, 471.312, 481.734, 485.136}},
	{{OrbType::Green, VehicleType::Ship, false}, {406.08, 427.248, 432, 429.138}},
	{{OrbType::Green, VehicleType::Ship, true}, {326.592, 343.548, 350.784, 345.06}},
	{{OrbType::Green, VehicleType::Ball, false}, {394.47, 415.638, 424.71, 417.528}},
	{{OrbType::Green, VehicleType::Ball, true}, {314.172, 331.074, 331.074, 332.586}},
	{{OrbType::Green, VehicleType::Ufo, false}, {432, 432, 432, 432}},
	{{OrbType::Green, VehicleType::Ufo, true}, {450.576, 474.768, 485.136, 476.928}},
};

static VehicleType orbPhysicsVehicle(VehicleType type) {
	switch (type) {
		case VehicleType::Robot:
		case VehicleType::Spider:
			return VehicleType::Cube;
		case VehicleType::Swing:
			return VehicleType::Ufo;
		default:
			return type;
	}
}

static bool shouldActivateOrb(Player const& p) {
	return p.buffer || (p.prevPlayer().buffer && !p.button) ||
		(p.vehicle.type == VehicleType::Ball && p.vehicleBuffer);
}

void Orb::collide(Player& p) const {
	if (!shouldActivateOrb(p) || p.jBlock)
		return;

	p.buffer = false;
	p.vehicleBuffer = false;

	if (type == OrbType::Teleport) {
		if (teleport.apply(p, pos))
			EffectObject::collide(p);
		p.input = false;
		return;
	}

	EffectObject::collide(p);

	if (type == OrbType::Dash || type == OrbType::GravityDash) {
		if (type == OrbType::GravityDash)
			p.upsideDown = !p.upsideDown;

		p.dashing = true;
		p.dashAngle = rotation;
		p.dashSpeed = player_speeds[p.speed] * 1.5f;
		p.velocity = 0;
		p.acceleration = 0;
		p.grounded = false;
		return;
	}

	if (type == OrbType::Spider) {
		bool targetCeiling = !p.upsideDown;
		float surface = p.level->findOppositeSurface(p, targetCeiling);
		if (std::isfinite(surface)) {
			p.upsideDown = !p.upsideDown;
			p.pos.y = targetCeiling ? surface - p.size.y / 2.f : surface + p.size.y / 2.f;
			p.velocity = 0;
			p.velocityOverride = true;
			p.grounded = true;
		}
		p.input = false;
		return;
	}

	// Wave can't use normal jump orbs, but gravity-changing orbs still apply.
	if (p.vehicle.type != VehicleType::Wave) {
		if (type == OrbType::Black) {
			p.velocity = -810;
		} else {
			VehicleType physicsType = orbPhysicsVehicle(p.vehicle.type);
			p.velocity = orb_velocities.get(type, physicsType, p.small, std::min(3, p.speed));
			p.grounded = false;
			if (type == OrbType::Green)
				p.velocityOverride = true;
		}
	}

	if (type == OrbType::Blue || type == OrbType::Green)
		p.upsideDown = !p.upsideDown;

	if (p.vehicle.type == VehicleType::Ball || p.vehicle.type == VehicleType::Spider)
		p.input = false;
}
