# Pathfinder Agent Message Bus

This file is the shared communication channel between the leader and all investigation agents.

## Leader broadcast — START

[Leader -> ALL AGENTS]
The investigation is now officially active.

Read `AGENT_LEADER_BOARD.md` before doing anything else.

Rules:
- Investigation only. Do NOT modify Pathfinder source code yet.
- Do NOT trigger or rerun GitHub Actions.
- Do NOT touch the locked known-good branch.
- Stay in your assigned lane unless evidence genuinely crosses boundaries.
- Do not silently duplicate another agent's investigation.
- Write findings into your own `AGENT_<LETTER>_REPORT.md` file.
- Mark your report STATUS as `CLAIMED`, then `INVESTIGATING`, then `COMPLETE`.
- Put cross-agent questions/challenges in this file using `[Agent X -> Agent Y]` entries.
- If you see another agent's conclusion that looks wrong, challenge it with code evidence instead of converging automatically.
- Do not implement proposed fixes yet. The leader will merge the reports first and decide what gets changed.

## Coordination priority

The leader wants every agent to distinguish between:
1. apparent diversity/recovery/progress, and
2. genuinely independent route families / verified progress.

A mechanism can look diverse while all routes share the same ancestry. A mechanism can look like progress while only speculative X moved. Test those distinctions explicitly where relevant.

## Agent acknowledgments

When you begin, add one short entry to your own report stating your lane and STATUS. Do not edit another agent's report.

## Messages

[Leader -> Agent C]
You are ACTIVE. Your current visible scope is search diversity / route-family ancestry / rollback verification from `AGENT_C_COORDINATION.md`. Create `AGENT_C_REPORT.md` and stay centered on that scope.

Your next required result is quantitative, not just descriptive:
1. Define a route-family / ancestry signature that can distinguish genuinely different prefixes.
2. Determine how many distinct route families can survive normal `selectFrontierV7` selection out of the nominal 100 logical helpers.
3. Determine whether the guided top-50, explorer coarse-key filtering, archive compaction, or rollback fallback causes the biggest family collapse.
4. Identify a concrete case where multiple helpers have different physics buckets/actions but still share essentially the same recent route ancestry.
5. Report an estimated `effective independent family count` or a method to measure it.

Do NOT investigate hidden Player fields in depth; that is Agent D's lane. If coarse-state identity appears to destroy route families, send the specific case to D through this file.

[Leader -> Agent D]
Your `AGENT_D_REPORT.md` is registered and STATUS is INVESTIGATING. The confirmed list of omitted future-affecting state is strong, but the leader needs severity evidence before accepting it as a major root cause.

Your next required result:
1. Separate `StateKeyV7` false equivalence from `CoarseKeyV7` false equivalence.
2. Find or construct paired states that share a key but diverge under the SAME next input/action. Prefer direct simulator/code-path proof where possible.
3. Rank omitted fields by likely real-world impact on Pathfinder benchmarks instead of treating every omitted field equally.
4. Determine whether archive compaction/failure memory can erase or poison a route family that Agent C would otherwise preserve.
5. Estimate how often unsafe exact/coarse collisions are likely to occur, or give a concrete instrumentation plan if static analysis cannot answer frequency.

Do NOT redesign worker ancestry selection; that is Agent C's scope. Send any route-family consequence you discover to C through this file.

[Leader -> Agents C and D]
Cross-check handshake required:
- C owns `Are our 100 helpers genuinely different route families?`
- D owns `Are states considered equivalent actually transition-equivalent?`
- The intersection is important: if C finds two distinct families being merged by a key, send that example to D. If D finds an unsafe key collision that destroys archive alternatives, send that example to C.

Neither agent should implement fixes yet. I want evidence and contradictions first.
