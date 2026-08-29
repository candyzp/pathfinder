# Pathfinder Multi-Agent Leader Board

This file is the shared coordination channel for the Pathfinder investigation.

## Global rules

- Investigation only for now. Do not modify Pathfinder source code yet.
- Do not trigger or rerun GitHub Actions.
- Do not modify `locked-beta.245-stereo-working` or its known-good commit.
- Do not overwrite another agent's lane without evidence that crosses lanes.
- Separate CONFIRMED facts from HYPOTHESES.
- Cite exact files/functions/structures.
- If you discover something that contradicts another agent, write the contradiction here rather than silently converging.

## Leader

Leader/integrator: ChatGPT coordinator.

Leader responsibilities:
- maintain the architecture map
- compare agent reports
- resolve contradictions
- watch for duplicate investigations
- decide what should eventually be changed
- no source implementation until investigation is merged

## Investigation lanes

### A — Worker diversity / multi-worker architecture
Trace logical helpers, physical threads, ancestry, parent selection, guided/explorer split, convergence and preservation of alternate route families.

### B — Search state / dead-end / rollback
Trace dead-end detection, rollback, safe progress, retry behavior, failure memory, repeated dead ends and false progress.

### C — Candidate generation / scoring / selection pressure
Trace action generation, mutation/randomization, scoring, sorting, elites, pruning and locally-good/globally-doomed selection.

### D — Physics-state representation / deduplication
Audit exact state keys and missing hidden state. Determine false merges and redundant equivalent-state exploration.

### E — Checkpoint / prefix committing / recovery
Trace safe-prefix committing, archive usage, rollback anchors, recovery roots and loss/preservation of sibling routes.

### F — Performance / scaling
Find dominant CPU/memory costs and duplicated simulation/copy/allocation/sorting work as population and horizon increase.

### G — Mode-specific search quality
Audit Cube/Ship/Ball/UFO/Wave/Robot/Spider/Swing assumptions, portal transitions, hitboxes and timing generation.

### H — Level geometry / collision model
Audit authored object conversion, hazards, no-touch objects, moving geometry, section indexing, endpoint inference and activation state.

### I — Progress / completion / UI reporting
Trace displayed percentage, SAFE vs SEEN, completion, failure messages, frozen percent and status spam.

### J — Architecture redesign
Synthesize a diversity-preserving solver architecture grounded in the current implementation.

## Current leader findings

1. Current main builds `src/pathfinder_state_v9.cpp`; the older `pathfinder_team_*` files are not the active solver.
2. V9 defines 100 logical helpers and 30 physical threads, with a nominal 50 guided / 50 explorer split.
3. `selectFrontierV7` takes the top guided candidates by rank before diversity filtering.
4. Explorer selection tries coarse physics-state diversity, but fallback selection can fill remaining slots without maintaining that uniqueness.
5. Therefore action generation can be diverse while route ancestry is still highly converged. The population may represent far fewer genuinely independent route families than its helper count suggests.
6. V9 now separates revocable SAFE progress from speculative/furthest-seen progress and uses progressive rollback, death clustering, poisoned-basin quarantine and failure memory.
7. `main.cpp` still keeps the large displayed percent monotonically increasing (`if (m_progress < telemetry.progress)`), so internal SAFE rollback can occur while the old visible percentage remains stuck at a previous high-water mark.
8. Agent C independently reported the same ancestry-diversity weakness and monotonic-percent mismatch in `AGENT_C_COORDINATION.md`.

## Agent report protocol

Each agent should add or update its own file named:

`AGENT_<LETTER>_REPORT.md`

Use this structure:

1. LANE
2. STATUS — CLAIMED / INVESTIGATING / COMPLETE
3. EXECUTIVE FINDINGS
4. EVIDENCE / RELEVANT CODE
5. CONFIRMED FAILURE MODES
6. COMPETING HYPOTHESES
7. TESTS TO PROVE / DISPROVE
8. PROPOSED CHANGES — proposal only, no implementation
9. CROSS-LANE DEPENDENCIES
10. QUESTIONS / CHALLENGES FOR OTHER AGENTS
11. CONFIDENCE LEVELS

## Communication protocol

Agents should read this board plus existing `AGENT_*_REPORT.md` / coordination files before investigating.

To send another agent a message without editing their report, create or update:

`AGENT_MESSAGES.md`

Use entries like:

`[Agent A -> Agent B] <message>`

Include evidence when challenging another finding.

## Important current question for all agents

When you inspect a mechanism that appears to provide diversity, recovery or progress, ask BOTH:

- Does it work at the physics-state level?
- Does it preserve genuinely different route ancestry/prefix families?

Those are not the same thing.
