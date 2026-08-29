# Agent C coordination: search diversity / rollback verification

Agent C lane only. Please avoid overlapping changes unless your evidence directly touches this area.

## Scope

Verify and harden the actual behavior behind the logical helper population:

- 100 logical states should not collapse into many near-copies of the same route family.
- The explorer half must preserve materially different approaches, not merely different X/Y/velocity snapshots that share the same recent prefix.
- After a dead-end rollback, helpers must not repopulate the poisoned basin just because they re-reach the old furthest X.
- SAFE progress and speculative/furthest progress must remain separate.
- Failure memory should prevent one-frame / few-pixel variants from being treated as brand-new ideas.

## Current findings on main at 6a401b89496ab6c398aade3e37657e8ab16d5ed8

1. v9 already separates SAFE progress from speculative X and can revoke SAFE progress during rollback.
2. v9 has coarse failure memory, death clustering, poisoned-basin quarantine, progressive rollback, and a +150 X escape requirement.
3. The population is 100 logical helper states executed by 30 physical threads: 50 guided + 50 explorers.
4. Normal `selectFrontierV7` still chooses the top 50 guided candidates purely by rank before diversity filtering. Explorer diversity uses coarse physics state only (X/Y/velocity/flags), not route ancestry/prefix diversity. Therefore many helpers can still be variations of the same underlying route family.
5. `rollbackFrontierV9` does a stronger spread around earlier anchors, but its final fallback can fill remaining explorer slots without coarse-key deduplication.
6. `main.cpp` still stores the large percent monotonically (`if (m_progress < telemetry.progress)`), so the old percent label can remain at an old high-water mark even when SAFE progress rolls backward. The v9 brain dashboard is more truthful because it shows SAFE and SEEN separately.

## Acceptance criteria for the actual fix

- Explorer selection includes route-family / ancestry diversity in addition to physics-state diversity.
- A single dominant prefix cannot consume most explorer slots when viable alternative prefixes exist.
- Guided slots may exploit the current best route, but explorers should remain protected from that convergence.
- Rollback/recovery selection should keep distinct route families from earlier checkpoints whenever available.
- Reaching an old speculative high-water mark after rollback must not reset stagnation or count as new SAFE progress.
- Telemetry should make it possible to see whether the population is actually diverse (for example, unique route-family count).

Agent C will avoid unrelated simulator, UI styling, playback, and general dead-end-controller edits unless needed to prove/fix this behavior.
