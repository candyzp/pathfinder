#include <Portals.hpp>
#include <Player.hpp>
#include <cmath>

VehiclePortal::VehiclePortal(Vec2D size, std::unordered_map<int, std::string>&& fields) : EffectObject(size, std::move(fields)) {
	switch (atoi(fields[1].c_str())) {
		case 12: type = VehicleType::Cube; break;
		case 13: type = VehicleType::Ship; break;
		case 47: type = VehicleType::Ball; break;
		case 111: type = VehicleType::Ufo; break;
		case 660: type = VehicleType::Wave; break;
		case 745: type = VehicleType::Robot; break;
		case 1331: type = VehicleType::Spider; break;
		case 1933: type = VehicleType::Swing; break;
		default: type = VehicleType::Cube; break;
	}
}

void VehiclePortal::collide(Player& player) const {
	EffectObject::collide(player);

	if (player.vehicle.type != type) {
		player.size = Vec2D(30, 30) * (player.small ? 0.6 : 1);

		// Going from wave to any other vehicle changes the carried velocity.
		if (player.vehicle.type == VehicleType::Wave)
			player.velocity *= 0.9;

		player.vehicle = Vehicle::from(type);
		player.vehicle.enter(player);
	}

	if (std::isfinite(player.vehicle.bounds) && player.vehicle.bounds < 100000.0f) {
		player.floor = std::max(0., 30 * std::ceil((pos.y - (player.vehicle.bounds / 2. + 30)) / 30.));
		player.ceiling = player.floor + player.vehicle.bounds;
	} else {
		player.floor = 0;
		player.ceiling = 999999;
	}
}
