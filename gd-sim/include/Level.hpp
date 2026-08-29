#pragma once
#include <Object.hpp>
#include <Player.hpp>
#include <vector>
#include <unordered_map>
#include <optional>

struct UnsupportedObjectInfo {
	int objectID = 0;
	Entity entity;
};

/**
 * In order to use the simulator, you must create a Level. The Level class is
 * the root class of everything else, containing both objects and player states,
 * as well as the main update function. See Level.cpp for implementation info.
 */
class Level {
	/// Called by constructor, applies level settings to the initial player state
	void initLevelSettings(std::string const& lvlSettings, Player& player);
	void simulatePlayer(Player& player, bool pressed, float dt);
 public:
	/**
	 * All player states are stored, including previous states. This way, Pathfinder
	 * is able to seamlessly rewind when searching for solutions.
	 */
	std::vector<Player> gameStates;
	std::vector<Player> gameStates2;

	size_t objectCount = 0;

	/// Sections are used just like real GD. See Object.hpp for more info on ObjectContainer.
	/// Sparse X sections. Modern levels often keep helper/decoration objects very far
	/// outside the playable area; a dense vector indexed by X could allocate millions
	/// of empty sections before simulation even began.
	std::unordered_map<int, std::vector<ObjectContainer>> sections;

	/// Static lookup of authored group target positions/rotations. This includes
	/// decorative objects too, because teleport targets are often invisible helpers.
	std::unordered_map<int, std::vector<Entity>> groupTargets;

	/// Objects that exist in the level string but do not yet have a simulator class.
	/// They are kept as metadata instead of silently disappearing or pretending to collide.
	std::vector<UnsupportedObjectInfo> unsupportedObjects;

	/// Furthest authored coordinate, including decorations/helpers. This is metadata only.
	/// It must never be used as the playable completion point by itself.
	float authoredExtent = 0.0f;

	/// Playable level end used by the physics solver and progress counter.
	float length = 0.0f;

	/// The constructor's best offline endpoint estimate. `length` may later be
	/// replaced with PlayLayer::getEndPosition().x for an active-level solve.
	float inferredLength = 0.0f;

	/// Human-readable source for endpoint diagnostics.
	std::string lengthSource = "fallback";

	static constexpr uint32_t sectionSize = 100;
	bool debug = false;

	Level(std::string const& lvlString);

	/// The main update function. Every frame is associated with a press/release state.
	Player& runFrame(bool pressed, float dt = 1/240.);
	Player& runFrame(bool player1Pressed, bool player2Pressed, float dt);

	/// Go back to a certain frame. Used in Pathfinder.
	void rollback(int frame);

	/// Find the nearest solid surface above or below a player. Used by spider mechanics.
	float findOppositeSurface(Player const& player, bool towardsCeiling) const;

	/// Resolve a group target for teleport or target-position gameplay objects.
	std::optional<Entity> getGroupTarget(int groupID, size_t index = 0) const;

	int currentFrame() const;
	Player const& getState(int frame, bool player2 = false) const;
	Player& latestState();
	Player& latestState2();
	Player const& latestState() const { return gameStates.back(); }
	Player const& latestState2() const { return gameStates2.back(); }
};
