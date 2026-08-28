#pragma once
#include <Object.hpp>
#include <Player.hpp>
#include <vector>
#include <unordered_map>
#include <optional>

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
	std::vector<std::vector<ObjectContainer>> sections;

	/// Static lookup of authored group target positions/rotations. This includes
	/// decorative objects too, because teleport targets are often invisible helpers.
	std::unordered_map<int, std::vector<Entity>> groupTargets;

	float length = 0.0;

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
};
