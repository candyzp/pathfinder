# Agent D Report

## 1. LANE

D — Physics-state representation / deduplication.

Assigned scope: audit exact Pathfinder search-state keys and hidden simulator state; determine false merges, redundant equivalent-state exploration, and how deduplication interacts with archive/failure memory. Investigation only. No solver source changes.

## 2. STATUS

INVESTIGATING

## 3. EXECUTIVE FINDINGS

### CONFIRMED

1. `SnapshotV7` preserves the full `Player` objects for P1/P2 plus press states and Move-trigger activation frames, but `StateKeyV7` represents only a small subset of those snapshots. Snapshot fidelity is therefore much richer than dedupe fidelity.
2. The exact key is not transition-complete: multiple future-affecting `Player` fields are omitted. Confirmed examples include persistent gameplay modifiers (`gravityScale`, `timeWarp`, `direction`), effect history (`usedEffects`), input/buffer state (`input`, `buffer`, `vehicleBuffer`), robot boost phase (`robotBoostTime`), dash parameters (`dashAngle`, `dashSpeed`), floor/ceiling bounds, coyote state, slope/snap state, player rotation, and deferred `actions`.
3. The dual-player portion is substantially incomplete. When dual is active, the key adds only P2 Y, vertical velocity, upside-down/small/mode flags and press state. It omits P2 X, speed, grounded state, dash state/parameters, buffers, persistent gameplay modifiers, effect history, floor/ceiling, slope/snap state and other future-affecting fields, even though P2 is simulated independently.
4. `CoarseKeyV7` is even more lossy and is used destructively by v9 archive compaction. V9 failure memory also derives from this coarse state. Thus two states that are physically/future-distinct can share archive/failure identity.
5. The exact key also quantizes P1 X/Y/velocity and frame (8/5/3 px position quanta, 45/28/18 velocity quanta, 3/2/1 frame quanta depending on precision), so even represented dimensions can be merged approximately.

## 4. EVIDENCE / RELEVANT CODE

- `src/pathfinder_state_v7.cpp`
  - `SnapshotV7`: stores full P1/P2 `Player`, `press1`, `press2`, and Move-trigger activation frames.
  - `stateFlagsV7`: stores P1 mode/speed/upsideDown/small/grounded/press/dashing/dual plus P2 press; in dual it adds only P2 upsideDown/small/mode.
  - `stateKeyV7`: adds quantized P1 X/Y/velocity/frame, P2 Y/vertical velocity when dual, and `triggerHashV7`.
  - `CoarseKeyV7` / `coarseKeyV7`: only coarse P1 X/Y/velocity + flags; no frame or Move-trigger hash.
- `gd-sim/include/Player.hpp`
  - `Player` contains substantially more future-affecting state than the key: rotation/size through `Entity`, acceleration, `robotBoostTime`, dash parameters, `gravityScale`, `timeWarp`, `direction`, `usedEffects`, `actions`, `slopeData`, `snapData`, floor/ceiling, coyoteFrames, input/buffers and other flags.
- `gd-sim/src/Player.cpp`
  - `preCollision` uses `timeWarp`, dash angle/speed, direction, pending `actions`, and input/buffer transitions.
  - `postCollision` uses floor/ceiling, coyoteFrames, previous input/buffer/acceleration, robot boost phase, gravityScale, slope state and rotation-sensitive hitboxes.
- `gd-sim/src/EffectObject.cpp`
  - `usedEffects` determines whether effect objects can touch/collide again.
- `gd-sim/src/Objects/AdvancedObjects.cpp`
  - gameplay triggers persistently mutate `gravityScale`, `timeWarp`, and `direction`; trigger crossing also consults `usedEffects`.
- `gd-sim/src/Objects/VehiclePortal.cpp`
  - vehicle portals set persistent `floor`/`ceiling` bounds; Wave entry can schedule deferred state mutation through the vehicle enter callback.
- `gd-sim/src/Vehicle.cpp`
  - Cube/Ball/UFO/Robot/Swing behavior uses hidden input/buffer/coyote/robot/slope state; Wave entry enqueues a deferred hitbox resize action.
- `gd-sim/src/Objects/Slope.cpp`
  - slope state and `timeElapsed` affect future velocity; slope logic also enqueues deferred actions.
- `gd-sim/src/Objects/Block.cpp`
  - `snapData` affects later X snapping; player rotation participates in collision geometry.
- `gd-sim/src/util.cpp` / `gd-sim/src/Objects/Object.cpp`
  - general `Entity::intersects` uses rotation, and non-cardinal object collision can use the rotated player entity.
- `src/pathfinder_state_v9.cpp`
  - produced nodes are deduplicated by `StateKeyV7`; `visited` can reject a same-key node based on score.
  - `mergeArchiveV9` keeps only the best node per `CoarseKeyV7`.
  - `coarseFailureKeyV9` uses the coarse state plus a coarsened action signature, allowing hidden-state variants to share failure counts/bans.

## 5. CONFIRMED FAILURE MODES

1. **Unsound exact-state equivalence:** same `StateKeyV7` does not guarantee the same next-state transition under the same input because omitted fields directly participate in physics/trigger logic.
2. **Hidden effect-history merge:** states with different `usedEffects` can be treated as identical even though one can reactivate an orb/portal/trigger and the other cannot.
3. **Persistent-modifier merge:** states with different gravity scale, time warp or horizontal direction can share a key and immediately diverge next tick.
4. **Deferred-action merge:** snapshots can carry pending callbacks that mutate the next frame, but the key does not represent the pending-action queue.
5. **Dual-state underrepresentation:** independently simulated P2 states can collapse despite materially different X/speed/grounding/effect/modifier state.
6. **Destructive coarse archive merge:** v9 archive compaction can discard hidden-state alternatives merely because they occupy the same coarse P1 bucket.
7. **Coarse failure contamination:** v9 can accumulate failure counts across hidden-state variants, risking hard bans on an action that is only bad in one underlying physics/environment state.

## 6. COMPETING HYPOTHESES

- H1: hidden-state false merges are a major contributor to complex-level stalls because simpler levels rarely exercise persistent triggers, slopes, deferred actions, dual asymmetry and dynamic bounds.
- H2: the exact key omissions exist but are infrequent in current benchmark levels; ancestry convergence/scoring may dominate observed stalls instead.
- H3: position/velocity quantization causes more real-world false merges than omitted fields in tight Ship/Wave sections.
- H4: absolute/quantized frame in the key may also prevent merging genuinely equivalent states in time-independent areas, creating redundant work; frequency is not yet measured.

## 7. TESTS TO PROVE / DISPROVE

Investigation continuing. Planned tests include a shadow full-state fingerprint, same-key/different-full-state counters, paired continuation simulation under identical actions, coarse-archive loss telemetry, dual-state collision tests and a static-level frame-redundancy audit.

## 8. PROPOSED CHANGES — proposal only, no implementation

Pending final audit. Direction is a layered identity model: transition-safe exact key, separate coarse diversity/basin key, symmetric P1/P2 representation, persistent environment/effect fingerprints, and non-destructive use of coarse similarity.

## 9. CROSS-LANE DEPENDENCIES

- Lane C: same-key and coarse-key selection decides which hidden variant survives.
- Lane E/B: rollback/archive/failure memory reuse these keys, so false equivalence can look like repeated dead-end learning.
- Lane G: mode-specific sensitivity determines safe quantization, especially Wave/Ship/Robot/Ball/Swing.
- Lane H: dynamic geometry and trigger support define the environment state that must enter the key.
- Lane F: a richer key/fingerprint must be designed without reintroducing heavy full-state copying/hashing.

## 10. QUESTIONS / CHALLENGES FOR OTHER AGENTS

Pending message-bus update after the remaining audit.

## 11. CONFIDENCE LEVELS

- Exact key omits future-affecting P1 state: HIGH.
- Dual key is transition-incomplete: HIGH.
- Coarse archive/failure identity can conflate hidden-state variants: HIGH.
- These false merges materially cause the user-observed complex-level stalls: MEDIUM until instrumented/reproduced.
- Frame identity causes substantial redundant exploration: LOW-MEDIUM until measured.
