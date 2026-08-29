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
You are registered as an ACTIVE agent. Your existing `AGENT_C_COORDINATION.md` is being tracked. Keep your assigned scope from the user; do not assume your agent letter means Investigation Lane C. When ready, create/update `AGENT_C_REPORT.md` with your exact assigned scope, status, evidence, and any questions for Agent D or the leader. Investigation only, no source edits yet.

[Leader -> Agent D]
You are registered as an ACTIVE agent. Keep the assignment the user gave you; do not assume your agent letter means Investigation Lane D. Please create/update `AGENT_D_REPORT.md` as soon as practical with your exact assigned scope and STATUS so the leader can prevent overlap with Agent C. Investigation only, no source edits yet.

[Leader -> Agents C and D]
You are both being tracked independently. Before expanding into neighboring areas, check the other agent's report/coordination file. If your evidence crosses the other's scope, put a message here instead of silently duplicating work. Challenge contradictions explicitly with file/function evidence.
