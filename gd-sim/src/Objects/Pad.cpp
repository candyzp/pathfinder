#include <Pad.hpp>
#include <Player.hpp>
#include <Level.hpp>
#include <cmath>

Pad::Pad(Vec2D size, std::unordered_map<int, std::string>&& fields) : EffectObject(size, std::move(fields)) {
	switch (atoi(fields[1].c_str())) {
		case 35: type = PadType::Yellow; break;
		case 67: type = PadType::Blue; break;
		case 140: type = PadType::Pink; break;
		case 1332: type = PadType::Red; break;
		case 3005: type = PadType::Spider; break;
		default: type = PadType::Yellow; break;
	}

	if (atoi(fields[5].c_str()) == 1)
		rotation += 180.f;
}

const velocity_map<PadType, VehicleType, bool> pad_velocities = {
	{{PadType::Yellow, VehicleType::Cube, false}, {864, 864, 864, 864}},
	{{PadType::Yellow, VehicleType::Cube, true}, {691.2, 691.2, 691.2, 691.2}},
	{{PadType::Yellow, VehicleType::Ship, false}, {864, 864, 864, 864}},
	{{PadType::Yellow, VehicleType::Ship, true}, {691.2, 691.2, 691.2, 691.2}},
	{{PadType::Yellow, VehicleType::Ball, false}, {518.4000206, 518.4000206, 518.4000206, 518.4000206}},
	{{PadType::Yellow, VehicleType::Ball, true}, {414.7200165, 414.7200165, 414.7200165, 414.7200165}},
	{{PadType::Yellow, VehicleType::Ufo, false}, {573.48, 432, 432, 432}},
	{{PadType::Yellow, VehicleType::Ufo, true}, {458.784, 691.2, 691.2, 691.2}},

	{{PadType::Blue, VehicleType::Cube, false}, {-345.6, -345.6, -345.6, -345.6}},
	{{PadType::Blue, VehicleType::Cube, true}, {-276.48, -276.48, -276.48, -276.48}},
	{{PadType::Blue, VehicleType::Ship, false}, {-229.392, -345.6, -345.6, -345.6}},
	{{PadType::Blue, VehicleType::Ship, true}, {-183.519, -276.48, -276.48, -165.888}},
	{{PadType::Blue, VehicleType::Ball, false}, {-160.574397, -207.360008, -207.360008, -207.360008}},
	{{PadType::Blue, VehicleType::Ball, true}, {-128.463298, -165.888007, -165.888007, -165.888007}},
	{{PadType::Blue, VehicleType::Ufo, false}, {-229.392, -345.6, -345.6, -345.6}},
	{{PadType::Blue, VehicleType::Ufo, true}, {-183.519, -276.48, -276.48, -276.48}},

	{{PadType::Pink, VehicleType::Cube, false}, {561.6, 561.6, 561.6, 561.6}},
	{{PadType::Pink, VehicleType::Cube, true}, {449.28, 449.28, 449.28, 449.28}},
	{{PadType::Pink, VehicleType::Ship, false}, {302.4, 302.4, 302.4, 302.4}},
	{{PadType::Pink, VehicleType::Ship, true}, {241.92, 241.92, 241.92, 241.92}},
	{{PadType::Pink, VehicleType::Ball, false}, {362.880014, 362.880014, 362.880014, 362.880014}},
	{{PadType::Pink, VehicleType::Ball, true}, {290.304012, 290.304012, 290.304012, 290.304012}},
	{{PadType::Pink, VehicleType::Ufo, false}, {345.6, 345.6, 345.6, 345.6}},
	{{PadType::Pink, VehicleType::Ufo, true}, {276.48, 276.48, 276.48, 276.48}},

	{{PadType::Red, VehicleType::Cube, false}, {1080, 1080, 1080, 1080}},
	{{PadType::Red, VehicleType::Cube, true}, {864, 864, 864, 864}},
	{{PadType::Red, VehicleType::Ship, false}, {544.32, 544.32, 544.32, 544.32}},
	{{PadType::Red, VehicleType::Ship, true}, {656.64, 656.64, 656.64, 656.64}},
	{{PadType::Red, VehicleType::Ball, false}, {648.00002575, 648.00002575, 648.00002575, 648.00002575}},
	{{PadType::Red, VehicleType::Ball, true}, {518.4000206, 518.4000206, 518.4000206, 518.4000206}},
	{{PadType::Red, VehicleType::Ufo, false}, {518.4, 518.4, 518.4, 518.4}},
	{{PadType::Red, VehicleType::Ufo, true}, {677.376, 677.376, 677.376, 677.376}},
};

static VehicleType padPhysicsVehicle(VehicleType type) {
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

void Pad::collide(Player& p) const {
	if (type == PadType::Spider) {
		bool targetCeiling = !p.upsideDown;
		float surface = p.level->findOppositeSurface(p, targetCeiling);
		if (std::isfinite(surface)) {
			p.upsideDown = !p.upsideDown;
			p.pos.y = targetCeiling ? surface - p.size.y / 2.f : surface + p.size.y / 2.f;
			p.velocity = 0;
			p.velocityOverride = true;
			p.grounded = true;
		}
		EffectObject::collide(p);
		return;
	}

	if (type == PadType::Blue) {
		auto rot = std::abs(rotation);
		if ((rot > 90 && !p.upsideDown) || (rot < 90 && p.upsideDown))
			return;

		if (p.upsideDown != p.prevPlayer().upsideDown)
			return;

		p.upsideDown = !p.upsideDown;
		if (p.vehicle.type == VehicleType::Wave)
			p.velocity = -p.velocity;
	}

	if (p.vehicle.type != VehicleType::Wave) {
		VehicleType physicsType = padPhysicsVehicle(p.vehicle.type);
		p.velocity = pad_velocities.get(type, physicsType, p.small, std::min(3, p.speed));
	}

	p.grounded = false;
	p.gravityPortal = false;
	EffectObject::collide(p);
}
