#include <sstream>
#include <iomanip>
#include <cfloat>
#include <cmath>
#include <algorithm>
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

// Estimate the X span of the object cloud connected to the real start. Remote helper
// clusters are common in modern levels and must not become the completion point merely
// because they contain an object with a very large X coordinate.
float connectedExtent(std::vector<float> xs, float startX, float maxConnectedGap) {
	if (xs.empty())
		return 0.f;

	std::sort(xs.begin(), xs.end());

	std::vector<float> uniqueXs;
	uniqueXs.reserve(xs.size());
	for (float x : xs) {
		if (uniqueXs.empty() || std::abs(x - uniqueXs.back()) > 0.5f)
			uniqueXs.push_back(x);
	}

	constexpr float backAllowance = 600.f;
	float minimumX = std::max(0.f, startX - backAllowance);

	auto it = std::lower_bound(uniqueXs.begin(), uniqueXs.end(), minimumX);
	if (it == uniqueXs.end())
		return 0.f;

	// If there is no authored content anywhere near the actual start, don't let a
	// remote helper cluster become the whole level. The caller will use a small fallback.
	if (*it > startX + maxConnectedGap)
		return 0.f;

	float extent = *it;
	for (++it; it != uniqueXs.end(); ++it) {
		if (*it - extent > maxConnectedGap)
			break;
		extent = *it;
	}

	return extent;
}

struct TeleportReachLink {
	float sourceX = 0.f;
	int targetGroup = 0;
};

float reachableGameplayExtent(
	std::vector<float> xs,
	float startX,
	float maxConnectedGap,
	std::vector<TeleportReachLink> const& teleportLinks,
	std::unordered_map<int, std::vector<Entity>> const& groupTargets
) {
	struct ResolvedLink { float sourceX; float targetX; };
	std::vector<ResolvedLink> resolvedLinks;
	for (auto const& link : teleportLinks) {
		auto target = groupTargets.find(link.targetGroup);
		if (target == groupTargets.end() || target->second.size() != 1)
			continue;
		float targetX = target->second.front().pos.x;
		if (!std::isfinite(targetX) || targetX < 0.f)
			continue;
		xs.push_back(targetX);
		resolvedLinks.push_back({link.sourceX, targetX});
	}

	if (xs.empty())
		return 0.f;
	std::sort(xs.begin(), xs.end());
	xs.erase(std::unique(xs.begin(), xs.end(), [](float a, float b) {
		return std::abs(a - b) <= 0.5f;
	}), xs.end());

	struct Cluster { float low; float high; bool reachable = false; };
	std::vector<Cluster> clusters;
	for (float x : xs) {
		if (clusters.empty() || x - clusters.back().high > maxConnectedGap)
			clusters.push_back({x, x, false});
		else
			clusters.back().high = x;
	}

	auto clusterFor = [&](float x) -> int {
		for (size_t i = 0; i < clusters.size(); ++i) {
			if (x >= clusters[i].low - 1.f && x <= clusters[i].high + 1.f)
				return static_cast<int>(i);
		}
		return -1;
	};

	for (auto& cluster : clusters) {
		if (cluster.low <= startX + maxConnectedGap && cluster.high >= startX - 600.f) {
			cluster.reachable = true;
			break;
		}
	}

	bool changed = true;
	while (changed) {
		changed = false;
		for (auto const& link : resolvedLinks) {
			int source = clusterFor(link.sourceX);
			int target = clusterFor(link.targetX);
			if (source >= 0 && target >= 0 && clusters[source].reachable && !clusters[target].reachable) {
				clusters[target].reachable = true;
				changed = true;
			}
		}
	}

	float extent = 0.f;
	for (auto const& cluster : clusters) {
		if (cluster.reachable)
			extent = std::max(extent, cluster.high);
	}
	return extent;
}

bool boolField(std::unordered_map<int, std::string> const& fields, int key) {
	auto it = fields.find(key);
	return it != fields.end() && !it->second.empty() && atoi(it->second.c_str()) != 0;
}

// Unsupported objects are normally decoration and remain harmless metadata. These IDs
// are exceptions: they can alter collision geometry or trigger execution, so their X
// positions still contribute to offline endpoint inference and diagnostics.
bool unsupportedGameplayObject(int id) {
	switch (id) {
		case 901:  // Move
		case 1049: // Toggle
		case 1268: // Spawn
		case 1346: // Rotate
		case 1347: // Follow
		case 1595: // Touch
		case 1616: // Stop
		case 1814: // Follow Player Y
		case 1815: // Collision
		case 2067: // Scale
		case 2068: // Advanced Random
		case 3032: // Keyframe
		case 3607: // Sequence
			return true;
		default:
			return false;
	}
}

UnsupportedObjectInfo makeUnsupportedInfo(
	int objectID,
	std::unordered_map<int, std::string> const& obj
) {
	float scale = stod_def(obj.contains(32) ? obj.at(32) : std::string{}, 1.f);
	float scaleX = stod_def(obj.contains(128) ? obj.at(128) : std::string{}, 1.f);
	float scaleY = stod_def(obj.contains(129) ? obj.at(129) : std::string{}, 1.f);

	UnsupportedObjectInfo info;
	info.objectID = objectID;
	info.entity = Entity {
		{
			stod_def(obj.contains(2) ? obj.at(2) : std::string{}, 0.f),
			stod_def(obj.contains(3) ? obj.at(3) : std::string{}, 0.f)
		},
		{30.f * scale * scaleX, 30.f * scale * scaleY},
		- stod_def(obj.contains(6) ? obj.at(6) : std::string{}, 0.f)
	};
	return info;
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

	auto get_or = [&obj](std::string const& key, std::string const& def) {
		if (auto it = obj.find(key); it != obj.end())
			return it->second.c_str();
		return def.c_str();
	};

	player.speed = atoi(get_or("kA4", "0"));

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
	// Entering a Wave portal normally installs its 10x10 (6x6 mini) collision
	// box on the following frame. A level that starts as Wave has no portal, so
	// initialize that mode-specific hitbox directly instead of leaving a Cube box.
	if (player.vehicle.type == VehicleType::Wave)
		player.size = player.small ? Vec2D(6, 6) : Vec2D(10, 10);

	player.floor = 0;
	player.ceiling = player.vehicle.bounds;
}

Level::Level(std::string const& lvlString) {
	std::stringstream ss(lvlString);
	std::string objstr;
	bool first = true;

	auto player = Player();
	std::vector<float> authoredXs;
	std::vector<float> gameplayXs;
	std::vector<TeleportReachLink> teleportLinks;
	float explicitEndX = 0.f;

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

		if (!obj.contains(1) || obj[1].empty())
			continue;

		int objectID = atoi(obj[1].c_str());

		// Object 31 is an editor Start Position. Published/full-level Pathfinder
		// runs must begin from the level header, not from whichever test StartPos
		// the creator happened to leave in the object list.
		if (objectID == 31)
			continue;

		float authoredX = stod_def(obj.contains(2) ? obj[2] : std::string{}, 0.f);
		bool validAuthoredX = std::isfinite(authoredX) && authoredX >= 0.f;

		if (validAuthoredX) {
			authoredXs.push_back(authoredX);
			authoredExtent = std::max(authoredExtent, authoredX + 100.f);
		}

		registerGroupTargets(obj, groupTargets);
		if ((objectID == 2902 || objectID == 3022 || objectID == 3027) && validAuthoredX) {
			auto target = obj.find(51);
			if (target != obj.end() && !target->second.empty())
				teleportLinks.push_back({authoredX, atoi(target->second.c_str())});
		}

		// A normal, X-crossing End Trigger is an authored endpoint. Spawn/touch End
		// Triggers remain runtime mechanics and cannot safely define offline length.
		if ((objectID == 1931 || objectID == 3600) &&
			!boolField(obj, 62) && !boolField(obj, 11) && validAuthoredX) {
			explicitEndX = std::max(explicitEndX, authoredX);
		}

		auto unsupported = makeUnsupportedInfo(objectID, obj);

		if (auto ob_o = Object::create(std::move(obj))) {
			auto ob = ob_o.value();

			ob->id = objectCount++;
			if (validAuthoredX)
				gameplayXs.push_back(authoredX);

			int sectionPos = static_cast<int>(std::floor(ob->pos.x / sectionSize));
			sections[sectionPos].push_back(std::move(ob));
		} else {
			if (validAuthoredX && unsupportedGameplayObject(objectID))
				gameplayXs.push_back(authoredX);
			unsupportedObjects.push_back(std::move(unsupported));
		}
	}

	constexpr float maxGameplayGap = 3000.f;
	float gameplayExtent = reachableGameplayExtent(
		gameplayXs,
		player.pos.x,
		maxGameplayGap,
		teleportLinks,
		groupTargets
	);
	float authoredConnectedExtent = connectedExtent(authoredXs, player.pos.x, maxGameplayGap);

	if (explicitEndX > player.pos.x + 30.f && explicitEndX <= gameplayExtent + 1.f) {
		inferredLength = explicitEndX;
		lengthSource = "end-trigger";
	} else if (gameplayExtent > 0.f) {
		// A conservative tail prevents the last obstacle from masquerading as the
		// actual GD end position. It is safer for playback to contain a little empty
		// travel than for offline progress to jump to 100% early.
		inferredLength = gameplayExtent + 240.f;
		lengthSource = "gameplay-extent";
	} else if (authoredConnectedExtent > 0.f) {
		inferredLength = authoredConnectedExtent + 240.f;
		lengthSource = "authored-fallback";
	} else {
		inferredLength = player.pos.x + 300.f;
		lengthSource = "minimum-fallback";
	}

	length = std::max(inferredLength, player.pos.x + 300.f);

	player.level = this;
	player.player2 = false;
	gameStates.push_back(player);

	Player second = player;
	second.player2 = true;
	second.dualActive = false;
	gameStates2.push_back(second);
}

void Level::simulatePlayer(Player& p, bool pressed, float dt) {
	if (p.dead || p.completed)
		return;

	p.dt = dt;
	p.preCollision(pressed);

	if (sections.empty()) {
		p.postCollision();
		return;
	}

	int sectionIdx = static_cast<int>(std::floor(p.pos.x / sectionSize));
	std::vector<ObjectContainer> const* nearby[3] = {nullptr, nullptr, nullptr};
	for (int i = 0; i < 3; ++i) {
		auto it = sections.find(sectionIdx + i - 1);
		if (it != sections.end())
			nearby[i] = &it->second;
	}

	// Reuse pointer scratch space across frames. The old hot loop allocated two
	// vectors and copied every ObjectContainer on every simulated tick.
	thread_local std::vector<ObjectContainer const*> blocks;
	thread_local std::vector<ObjectContainer const*> hazards;
	blocks.clear();
	hazards.clear();
	if (blocks.capacity() < 128)
		blocks.reserve(128);
	if (hazards.capacity() < 128)
		hazards.reserve(128);

	size_t numCollisions = 0;

	for (auto const* section : nearby) {
		if (section == nullptr) continue;
		for (auto const& o : *section) {
			if (p.dead || p.completed) break;
			if (o->prio == 1)
				blocks.push_back(&o);
			else if (o->prio == 2)
				hazards.push_back(&o);
			else if (o->touching(p)) {
				++numCollisions;
				o->collide(p);
			}
		}
	}

	for (int i = static_cast<int>(blocks.size()) - 1; i >= 0; --i) {
		if (p.dead || p.completed) break;
		auto const& b = *blocks[i];
		if (b->touching(p)) {
			++numCollisions;
			b->collide(p);
		}
	}

	for (auto const* hazard : hazards) {
		if (p.dead || p.completed) break;
		auto const& h = *hazard;
		if (h->touching(p)) {
			++numCollisions;
			h->collide(p);
		}
	}

	if (!p.dead && !p.completed)
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
	Player const& p1Before = gameStates.back();
	Player const& p2Before = gameStates2.back();
	Player p1 = p1Before;
	Player p2 = p2Before;
	bool wasDual = p1.dualActive;

	auto detectNaturalFinish = [this](Player const& before, Player& after) {
		constexpr float maxNaturalFinishStep = 120.f;
		float step = after.pos.x - before.pos.x;
		if (!after.dead && !after.completed &&
			before.pos.x < length && after.pos.x >= length &&
			step >= 0.f && step <= maxNaturalFinishStep) {
			after.completed = true;
		}
	};

	simulatePlayer(p1, player1Pressed, dt);
	detectNaturalFinish(p1Before, p1);

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
			detectNaturalFinish(p2Before, p2);

			if (!p2.dualActive)
				p1.dualActive = false;
		}

		if (p1.dead || p2.dead) {
			p1.dead = true;
			p2.dead = true;
		}
		if (p1.completed || p2.completed) {
			p1.completed = true;
			p2.completed = true;
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

	for (auto const& [_, section] : sections) {
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
	auto trim = [target](std::vector<Player>& states) {
		while (states.size() > 1 && states.back().frame > target)
			states.pop_back();
	};
	trim(gameStates);
	trim(gameStates2);
}

int Level::currentFrame() const {
	return gameStates.empty() ? 0 : gameStates.back().frame;
}

Player const& Level::getState(int frame, bool player2) const {
	auto const& states = player2 ? gameStates2 : gameStates;
	if (frame <= states.front().frame)
		return states.front();
	if (frame >= states.back().frame)
		return states.back();

	size_t direct = static_cast<size_t>(frame - states.front().frame);
	if (direct < states.size() && states[direct].frame == frame)
		return states[direct];

	auto it = std::lower_bound(states.begin(), states.end(), frame,
		[](Player const& state, int value) { return state.frame < value; });
	return it == states.end() ? states.back() : *it;
}

Player& Level::latestState() {
	return gameStates.back();
}

Player& Level::latestState2() {
	return gameStates2.back();
}
