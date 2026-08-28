# Pathfinder

Auto-generate macros for levels using simulation! This mod uses a physics simulator under the hood to search for inputs that solve levels. It does not come with a playback bot, so exported macros still need a compatible replay bot.

# Experimental Gameplay Support

This fork expands the simulator beyond the original classic-mode support. Experimental support now includes:

- Dual mode with independent Player 1 / Player 2 input search and macro export, including pending P2 inputs for tight dual entries.
- Robot mode with 2.2 speed-dependent jump/gravity behavior.
- Spider mode and Spider pads/orbs.
- Swing mode with 2.2 gravity scaling.
- Dash and gravity-dash orbs.
- Classic linked teleport portals.
- 2.2 unlinked blue teleport portals, Teleport Orbs and Teleport Triggers with static target-group lookup, Save Offset, Ignore X/Y, exit gravity, force redirection, Redirect Dash and Snap Ground.
- Green/toggle gravity portals.
- D, J, S, H and F modifier blocks.
- Force blocks.
- Gravity, TimeWarp, Reverse, End, Player Control and horizontal Gameplay Rotation triggers.
- Arbitrarily rotated solid blocks for Cube, UFO, Ball and other supported modes.
- Both upside-down slope orientations.
- Existing Cube, Ship, Ball, UFO and Wave physics, pads, orbs, speed portals and slopes.

# Remaining 2.2 Limitations

Pathfinder still uses a lightweight offline simulator rather than Geometry Dash itself, so some highly dynamic 2.2 systems need a larger simulation layer before they can be exact:

- Group/spawn/channel trigger graphs and collision objects that are moved, scaled or rotated dynamically after level start.
- Teleport targets that themselves move dynamically during the run.
- Full 90-degree vertical Gameplay Rotation / Arrow Trigger movement.
- Platformer left/right controls and platformer-only Player Control behavior.
- Some advanced 2.2 object-specific collision/force edge cases.

Unsupported decorative, camera, shader and audio objects are intentionally ignored because they do not affect whether a path survives.

# How To Use

1. Go to a level you want to pathfind, either in your saved or an online level.
2. Click the blue Pathfinder button to start the pathfinder.
3. Export the macro into the correct folder of whichever bot you are using.
4. Import the macro and play it back.

# Report Bugs

Simulation differences are best reported with the level ID, game mode, approximate percentage and the object/mechanic where the simulated path diverges.
