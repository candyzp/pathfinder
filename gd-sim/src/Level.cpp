#include <sstream>
#include <iomanip>
#include <cfloat>
#include <cmath>
#include <unordered_set>
#include <Level.hpp>

namespace {
void registerGroupTargets(std::unordered_map<int, std::string> const& obj,
                          std::unordered_map<int, std::vector<Entity>>& targets) {
	if (!obj.contains(2) || !obj.contains(3))
		return;

	Entity marker{
		{stod_def(obj.at(2)), stod_def(obj.at(3))},
		{0, 0},
		- stod_def(obj.contains(6) ? obj.at(6) : std::string{})
	};

	std::unordered_set<int> groups;
	auto addGroup = [&groups](std::string const& value) {
		if (value.empty()) return;
		int id = atoi(value.c_str());
		if (id > 0) groups.insert(id);
	};

	if (auto it = obj.find(33); it != obj.end())
		addGroup(it->second);

	if (auto it = obj.find(57); it != obj.end()) {
		std::stringstream groupStream(it->second);
		std::string group;
		while (std::getline(groupStream, group, '.'))
			addGroup(group);
	}

	for (int group : groups)
		targets[group].push_back(marker);
}
}

void Level::initLevelSettings(std::string const& lvlSettings, Player& player) {
	std::unordered_map<std::string, std::string> obj;

	std::stringstream ss2(lvlSettings);
	std::string k, v;
	while (std::getline(ss2, k, ',')) {
		std::getline(ss2, v, ',');
		obj[k] = v;
	}

	// Helper to make a default value for nonexistant keys
	auto get_or = [&obj](std::string const& key, std::string const& def) {
		if (auto it = obj.find(key); it != obj.end())
			return it->second.c_str();
		return def.c_str();
	};

	player.speed = atoi(get_or("kA4", "0"));

	// RobTop stores 1x speed as 0 and slow speed as 1.
	if (player.speed == 0)
		player.speed = 1;
	else if (player.speed == 1)
		player.speed = 0;

	if ((player.small = atoi(get_or("kA3", "0"))))
		player.size = player.size * 0.6;

	player.upsideDown = atoi(get_or("kA11", "0"));
	int vehicle = atoi(get_or("kA2", "0"));
	if (vehicle < 0 || vehicle > static_cast<int>(VehicleType::Swing))
		vehicle = 0;
	player.vehicle = Vehicle::from(static_cast<VehicleType>(vehicle));

	player.floor = 0;
	player.ceiling = player.vehicle.bounds;
}

Level::Level(std::string const& lvlString) {
	std::stringstream ss(lvlString);
	std::string objstr;
	bool first = true;

	auto player = Player();

	while (std::getline(ss, objstr, ';')) {
		if (first) {
			initLevelSettings(objstr, player);
			first = false;
			continue;
		}

		std::unordered_map<int, std::string> obj;

		std::stringstream ss2(objstr);
		std::string k, v;
		while (std::getline(ss2, k, ',')) {
			std::getline(ss2, v, ',');
			if (atoi(k.c_str()) > 0)
				obj[atoi(k.c_str())] = v;
		}

		if (!obj.contains(1))
			continue;

		// Group targets must be indexed before Object::create moves the parsed map,
		// and must include decorative/invisible helper objects that the physics parser ignores.
		registerGroupTargets(obj, groupTargets);

		if (obj[1] == "31") {
			initLevelSettings(objstr, player);
			player.pos.x = stod_def(obj[2], 0);
			player.pos.y = stod_def(obj[3], 0);
		}

		if (auto ob_o = Object::create(std::move(obj))) {
			auto ob = ob_o.value();

			ob->id = objectCount++;

			size_t sectionPos = static_cast<size_t>(std::max(.0f, ob->pos.x / sectionSize));
			if (sectionPos >= sections.size())
				sections.resize(sectionPos + 1);
			sections[sectionPos].push_back(ob);

			if (ob->pos.x > length)
				length = ob->pos.x + 100;
		}
	}

	player.level = this;
	player.player2 = false;
	gameStates.push_back(player);

	Player second = player;
	second.player2 = true;
	second.dualActive = false;
	gameStates2.push_back(second);
}

void Level::simulatePlayer(Player& p, bool pressed, float dt) {
	if (p.dead)
		return;

	p.dt = dt;
	p.preCollision(pressed);

	if (sections.empty()) {
		p.postCollision();
		return;
	}

	size_t sectionIdx = std::min(std::max(0, (int)(p.pos.x / sectionSize)), (int)sections.size() - 1);
	auto prevSection = &sections[sectionIdx == 0 ? 0 : sectionIdx - 1];
	auto currSection = &sections[sectionIdx];
	auto nextSection = &sections[sectionIdx + 1 >= sections.size() ? sections.size() - 1 : sectionIdx + 1];

	std::vector<ObjectContainer>* nearby[3] = { prevSection, nullptr, nullptr };
	if (currSection != prevSection)
		nearby[1] = currSection;
	if (nextSection != currSection && nextSection != prevSection)
		nearby[2] = nextSection;

	std::vector<ObjectContainer> blocks;
	std::vector<ObjectContainer> hazards;
	blocks.reserve(100);
	hazards.reserve(100);

	size_t numCollisions = 0;

	for (auto section : nearby) {
		if (section == nullptr) continue;
		for (auto& o : *section) {
			if (p.dead) break;
			if (o->prio == 1)
				blocks.push_back(o);
			else if (o->prio == 2)
				hazards.push_back(o);
			else if (o->touching(p)) {
				++numCollisions;
				o->collide(p);
			}
		}
	}

	for (int i = static_cast<int>(blocks.size()) - 1; i >= 0; --i) {
		if (p.dead) break;
		auto& b = blocks[i];
		if (b->touching(p)) {
			++numCollisions;
			b->collide(p);
		}
	}

	for (auto& h : hazards) {
		if (p.dead) break;
		if (h->touching(p)) {
			++numCollisions;
			h->collide(p);
		}
	}

	if (!p.dead)
		p.postCollision();

	if (debug) {
		std::cout << "P" << (p.player2 ? 2 : 1) << " Frame " << currentFrame() << std::fixed << std::setprecision(8)
				  << " X " << p.pos.x << " Y " << p.pos.y - 15 << " Vel " << p.velocity
				  << " Accel " << p.acceleration << " Rot " << p.rotation << " Coll " << numCollisions
				  << std::endl;
	}
}

Player& Level::runFrame(bool pressed, float dt) {
	return runFrame(pressed, pressed, dt);
}

Player& Level::runFrame(bool player1Pressed, bool player2Pressed, float dt) {
	Player p1 = gameStates.back();
	Player p2 = gameStates2.back();
	bool wasDual = p1.dualActive;

	simulatePlayer(p1, player1Pressed, dt);

	if (p1.dualActive) {
		if (!wasDual) {
			p2 = p1;
			p2.player2 = true;
			p2.dualActive = true;
			if (std::isfinite(p1.ceiling) && p1.ceiling < FLT_MAX / 2) {
				p2.pos.y = p1.floor + p1.ceiling - p1.pos.y;
				p2.upsideDown = !p1.upsideDown;
				p2.velocity = -p1.velocity;
			}
		} else {
			p2.dualActive = true;
			simulatePlayer(p2, player2Pressed, dt);

			if (!p2.dualActive)
				p1.dualActive = false;
		}

		if (p1.dead || p2.dead) {
			p1.dead = true;
			p2.dead = true;
		}
	} else {
		p2 = p1;
		p2.player2 = true;
		p2.dualActive = false;
	}

	gameStates.push_back(p1);
	gameStates2.push_back(p2);
	return gameStates.back();
}

float Level::findOppositeSurface(Player const& player, bool towardsCeiling) const {
	float best = towardsCeiling ? FLT_MAX : -FLT_MAX;
	float halfWidth = player.size.x / 2.f;

	for (auto const& section : sections) {
		for (auto const& o : section) {
			if (o->prio != 1)
				continue;

			if (player.pos.x + halfWidth < o->getLeft() || player.pos.x - halfWidth > o->getRight())
				continue;

			if (towardsCeiling) {
				float surface = o->getBottom();
				if (surface >= player.getTop() - 0.5f && surface < best)
					best = surface;
			} else {
				float surface = o->getTop();
				if (surface <= player.getBottom() + 0.5f && surface > best)
					best = surface;
			}
		}
	}

	if (towardsCeiling)
		return best == FLT_MAX ? player.ceiling : best;
	return best == -FLT_MAX ? player.floor : best;
}

std::optional<Entity> Level::getGroupTarget(int groupID, size_t index) const {
	if (groupID <= 0)
		return {};

	auto it = groupTargets.find(groupID);
	if (it == groupTargets.end() || it->second.empty())
		return {};

	return it->second[index % it->second.size()];
}

void Level::rollback(int frame) {
	int target = frame > 0 ? frame : 1;
	gameStates.resize(target);
	gameStates2.resize(target);
}

int Level::currentFrame() const {
	return gameStates.size();
}

Player const& Level::getState(int frame, bool player2) const {
	auto const& states = player2 ? gameStates2 : gameStates;
	if (frame <= 0)
		return states[0];
	if (states.size() < static_cast<size_t>(frame))
		return states.back();
	return states[frame - 1];
}

Player& Level::latestState() {
	return gameStates.back();
}

Player& Level::latestState2() {
	return gameStates2.back();
}
