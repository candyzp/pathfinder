# Pathfinder

Auto-generate macros for levels using simulation! This mod uses a physics simulator under the hood to search for inputs that solve levels. It does not come with a playback bot, so exported macros still need a compatible replay bot.

# Experimental Gameplay Support

This fork expands the simulator beyond the original classic-mode support. Experimental support now includes:

- Dual mode with independent Player 1 / Player 2 input search and macro export.
- Robot mode.
- Spider mode and Spider pads/orbs.
- Swing mode.
- Dash and gravity-dash orbs.
- Linked teleport portals.
- Green/toggle gravity portals.
- D, J, S, H and F modifier blocks.
- Force blocks.
- Gravity, TimeWarp, Reverse, End and horizontal Gameplay Rotation triggers.
- Arbitrarily rotated solid blocks for Cube, UFO, Ball and other supported modes.
- Both upside-down slope orientations.
- Existing Cube, Ship, Ball, UFO and Wave physics, pads, orbs, speed portals and slopes.

# Remaining 2.2 Limitations

Pathfinder still uses a lightweight offline simulator rather than Geometry Dash itself, so some highly dynamic 2.2 systems need a larger simulation layer before they can be exact:

- Group/spawn/channel trigger graphs and dynamically moved/scaled/rotated collision objects.
- Exact target-group behavior for unlinked teleport portals, Teleport Orbs and Teleport Triggers.
- Full 90-degree vertical Gameplay Rotation / Arrow Trigger movement.
- Platformer left/right controls.
- Some advanced 2.2 object-specific collision behavior.

Unsupported decorative, camera, shader and audio objects are intentionally ignored because they do not affect whether a path survives.

# How To Use

1. Go to a level you want to pathfind, either in your saved or an online level.
2. Click the blue Pathfinder button to start the pathfinder.
3. Export the macro into the correct folder of whichever bot you are using.
4. Import the macro and play it back.

# Report Bugs

Simulation differences are best reported with the level ID, game mode, approximate percentage and the object/mechanic where the simulated path diverges.
