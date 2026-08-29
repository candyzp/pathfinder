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

### LEADER MODE: ACTIVE COORDINATION ONLY

The leader is NOT investigating Pathfinder source and is NOT implementing fixes while agents are working.

Leader responsibilities right now:
- watch agent report/coordination files and recent commits
- maintain lane ownership and prevent duplicate work
- issue targeted verification orders when reports are too broad or unquantified
- route questions/challenges between agents through `AGENT_MESSAGES.md`
- compare findings when reports arrive
- identify contradictions and request targeted verification
- merge the investigation only after agents report back
- decide what should eventually be changed

Do not wait for the leader to perform your lane. Agents should investigate independently and report evidence.

## Active agent roster

Agent identity letters are NOT automatically the same thing as Investigation Lane letters. Each agent keeps the assignment the user gave it and must state that exact scope in its report.

### Agent C — ACTIVE / INVESTIGATING
- Existing coordination file: `AGENT_C_COORDINATION.md`.
- Current visible scope: search diversity, route-family ancestry, worker convergence, rollback diversity verification.
- Formal `AGENT_C_REPORT.md`: not visible yet.
- Current leader order: quantify route-family ancestry diversity and estimate effective independent family count; identify where normal selection/rollback causes the largest family collapse.
- Boundary: do not deep-audit hidden Player fields; send key-equivalence examples to Agent D.

### Agent D — ACTIVE / INVESTIGATING
- Formal report: `AGENT_D_REPORT.md` registered in commit `a42de825c187c9e1ea3ca84c85dc500d57c323f0`.
- Assigned scope: physics-state representation / deduplication.
- Current agent-reported findings: `StateKeyV7` and especially `CoarseKeyV7` omit multiple future-affecting state dimensions; dual-state identity is incomplete; v9 archive compaction and failure memory may conflate physically different states.
- Confidence reported by D: HIGH on structural key incompleteness, MEDIUM on whether it materially causes observed complex-level stalls.
- Current leader order: prove severity with paired same-key states that diverge under identical next actions, separate exact-key versus coarse-key failures, rank omitted fields by real-world importance, and quantify/instrument collision frequency.
- Boundary: do not redesign worker ancestry selection; send route-family consequences to Agent C.

## Current cross-agent handshake

C and D are intentionally studying two different meanings of "diversity":

- Agent C: Are the nominal helper slots genuinely different route/prefix families?
- Agent D: Are states treated as equivalent actually transition-equivalent?

Required cross-check:
- C sends D concrete cases where distinct families are merged by state identity.
- D sends C concrete cases where key/archive/failure equivalence destroys a potentially useful alternative family.

This intersection is a priority because route-family diversity can be lost either by selection pressure or by unsound state equivalence.

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

## Previously established context available to agents

These are existing observations from before leader-only coordination began. Agents should verify them rather than treating them as unquestionable conclusions:

1. Current main builds `src/pathfinder_state_v9.cpp`; the older `pathfinder_team_*` files are not the active solver.
2. V9 defines 100 logical helpers and 30 physical threads, with a nominal 50 guided / 50 explorer split.
3. `selectFrontierV7` appears to take the top guided candidates by rank before diversity filtering.
4. Explorer selection appears to attempt coarse physics-state diversity, with fallback behavior that may weaken uniqueness.
5. V9 separates revocable SAFE progress from speculative/furthest-seen progress and includes progressive rollback, death clustering, poisoned-basin quarantine and failure memory.
6. A prior observation suggests `main.cpp` keeps the large displayed percent monotonically increasing, potentially diverging from revocable SAFE progress.
7. `AGENT_C_COORDINATION.md` contains an earlier independent note about ancestry diversity and progress display behavior.

## Agent report protocol

Each agent should add or update its own file named:

`AGENT_<LETTER>_REPORT.md`

Use this structure:

1. AGENT ID
2. EXACT ASSIGNED SCOPE / LANE
3. STATUS — CLAIMED / INVESTIGATING / COMPLETE
4. EXECUTIVE FINDINGS
5. EVIDENCE / RELEVANT CODE
6. CONFIRMED FAILURE MODES
7. COMPETING HYPOTHESES
8. TESTS TO PROVE / DISPROVE
9. PROPOSED CHANGES — proposal only, no implementation
10. CROSS-LANE DEPENDENCIES
11. QUESTIONS / CHALLENGES FOR OTHER AGENTS
12. CONFIDENCE LEVELS

## Communication protocol

Agents should read this board plus existing `AGENT_*_REPORT.md` / coordination files before investigating.

To send another agent a message without editing their report, update:

`AGENT_MESSAGES.md`

Use entries like:

`[Agent C -> Agent D] <message>`

Include evidence when challenging another finding.

## Important current question for all agents

When you inspect a mechanism that appears to provide diversity, recovery or progress, ask BOTH:

- Does it work at the physics-state level?
- Does it preserve genuinely different route ancestry/prefix families?

Those are not the same thing.
