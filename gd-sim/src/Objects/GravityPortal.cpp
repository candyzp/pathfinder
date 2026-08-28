#include <Portals.hpp>
#include <Player.hpp>

GravityPortal::GravityPortal(Vec2D size, std::unordered_map<int, std::string>&& fields) : EffectObject(size, std::move(fields)) {
	int objectId = std::stoi(fields[1]);
	mode = objectId == 11 ? 1 : (objectId == 2926 ? 2 : 0);
}

void GravityPortal::collide(Player& p) const {
	EffectObject::collide(p);

	bool target = mode == 2 ? !p.upsideDown : mode == 1;
	if (target != p.upsideDown) {
		// Gravity portals halve and reflect carried vertical velocity.
		p.velocity = -p.velocity / 2;
		p.upsideDown = target;
		p.gravityPortal = true;
	}
}
