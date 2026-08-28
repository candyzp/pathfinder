#include <set>
#include <cstdint>
#include <algorithm>
#include <Level.hpp>
#include <random>
#include <gdr/gdr.hpp>
#include "pathfinder.hpp"

class Replay2 : public gdr::Replay<Replay2, gdr::Input<"">> {
 public:
	Replay2() : Replay("Pathfinder", 1){}
};

struct Level2 : public Level {
	bool press1 = false;
	bool press2 = false;
	float highestY = 0;
	using Level::Level;

	Level2(std::string const& lvlString) : Level(lvlString) {
		for (auto& section : sections) {
			for (auto& object : section)
				highestY = std::max(highestY, object->pos.y);
	}

	void syncPresses() {
		press1 = latestState().button;
		press2 = latestState2().button;
	}
};

using SearchInput = uint32_t;

static SearchInput inputKey(uint32_t frame, bool player2) {
	return (frame << 1) | static_cast<uint32_t>(player2);
}

bool isLevelEnd(Level2& lvl) {
	return lvl.latestState().pos.x >= lvl.length;
}

int tryInputs(Level2& lvl, std::set<SearchInput> inputs) {
	auto frame = lvl.currentFrame();
	auto press1Before = lvl.press1;
	auto press2Before = lvl.press2;

	while (!inputs.empty() && !lvl.latestState().dead) {
		uint32_t current = static_cast<uint32_t>(lvl.currentFrame());
		auto p1 = inputKey(current, false);
		auto p2 = inputKey(current, true);

		if (inputs.contains(p1)) {
			lvl.press1 = !lvl.press1;
			inputs.erase(p1);
		}
		if (inputs.contains(p2)) {
			lvl.press2 = !lvl.press2;
			inputs.erase(p2);
		}

		lvl.runFrame(lvl.press1, lvl.press2, 1.f / 240.f);
	}

	int final = lvl.currentFrame();
	float lastX = lvl.latestState().pos.x;
	float lastY = lvl.latestState().pos.y;

	lvl.rollback(frame);
	lvl.press1 = press1Before;
	lvl.press2 = press2Before;

	if (lastX < lvl.length && (lastY > std::max(1300.f, lvl.highestY + 600.f) || lastY < -600.f))
		return 0;

	return final;
}

std::vector<uint8_t> pathfind(std::string const& lvlString, std::atomic_bool& stop, std::function<void(double)> callback) {
	Level2 lvl(lvlString);

	std::random_device rd;
	std::mt19937 rng(rd());
	std::uniform_int_distribution<int> frameDist(0, 999);
	std::uniform_int_distribution<int> playerDist(0, 1);

	int trueBest = 0;
	int fail = 1;
	int numAway = 1000;

	Level2 lvlBest = lvl;

	while (lvl.latestState().pos.x < lvl.length) {
		auto frame = lvl.currentFrame();

		std::set<SearchInput> bestInputs;
		int bestFrame = frame;

		constexpr int iterations = 300;
		for (int i = 0; i < iterations; i++) {
			std::set<SearchInput> inputs;

			bool dual = lvl.latestState().dualActive;
			int candidateCount = dual ? 54 : 30;
			for (int j = 0; j < candidateCount; j++) {
				uint32_t candidateFrame = static_cast<uint32_t>(frame + frameDist(rng));
				bool player2 = dual ? (playerDist(rng) != 0) : false;
				inputs.insert(inputKey(candidateFrame, player2));
			}

			int nf = tryInputs(lvl, inputs);
			if (nf > bestFrame) {
				bestFrame = nf;
				bestInputs = inputs;
				if (bestFrame - frame > 500 && fail < 1000)
					break;
			}
		}

		if (bestFrame == frame) {
			lvl.rollback(std::max(std::max(frame - fail, trueBest - numAway), 1));
			lvl.syncPresses();

			fail += 5;
			if (fail > numAway + 1000) {
				numAway += 1000;
				fail = 1;

				if (numAway > 10000) {
					numAway = 1000;
					trueBest = 0;
					lvl.rollback(1);
					lvl.syncPresses();
				}
			} else if (fail > 100) {
				fail += 50;
			}
		} else {
			int applyUntil = bestFrame - static_cast<int>((bestFrame - frame) / 1.5);
			for (int i = frame; i < applyUntil; ++i) {
				auto p1 = inputKey(static_cast<uint32_t>(i), false);
				auto p2 = inputKey(static_cast<uint32_t>(i), true);
				if (bestInputs.contains(p1))
					lvl.press1 = !lvl.press1;
				if (bestInputs.contains(p2))
					lvl.press2 = !lvl.press2;

				lvl.runFrame(lvl.press1, lvl.press2, 1.f / 240.f);
			}
		}

		if (lvl.currentFrame() > trueBest) {
			trueBest = lvl.currentFrame();
			fail = 0;
			numAway = 1000;
		}
		if (lvl.currentFrame() > lvlBest.currentFrame())
			lvlBest = lvl;

		if (callback)
			callback(std::min((lvl.latestState().pos.x / lvl.length) * 100, 100.0f));

		if (stop)
			break;
	}

	Replay2 output;
	for (size_t i = 1; i < lvlBest.gameStates.size(); ++i) {
		auto const& p1 = lvlBest.gameStates[i];
		auto const& p1Prev = lvlBest.gameStates[i - 1];
		if (p1.frame > 1 && p1.button != p1Prev.button)
			output.inputs.push_back(gdr::Input(p1.frame, 1, false, p1.button));

		if (i < lvlBest.gameStates2.size()) {
			auto const& p2 = lvlBest.gameStates2[i];
			auto const& p2Prev = lvlBest.gameStates2[i - 1];
			if (p2.dualActive && p2.frame > 1 && p2.button != p2Prev.button)
				output.inputs.push_back(gdr::Input(p2.frame, 1, true, p2.button));
		}
	}

	return output.exportData().unwrapOr({});
}
