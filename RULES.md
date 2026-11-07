# RULES

One-line rules learned from fixes that passed. Apply to every new unit of work.
- Check swept volumes, not just the stowed pose: any part attached to a moving stage must clear every other stage across the full travel.
- Derive feedforward (kG, kV, kA) from the motor model and masses and pin it with a test; never hand-guess it.
- AdvantageKit logs only value changes: seed any derived sequence with the value in effect at the reference time, never only with later change records.
- Fix the check only when it measured the wrong property, and re-run the corrected check on the same evidence first; the real defect must still show.
- Before tuning gains, log the profile reference next to position: if output is current-limited, gains cannot fix it; size limits for worst-case (down-decel: m(g+a)).
- Reproduce under match-like conditions (alliance set, FMS-ish DS state): bugs gated on alliance or game data stay invisible in a bare sim.
- Disable hardware with an explicit flag that skips config, control and reads, never with an invalid device ID.
- When a background watchdog is killed, kill its children too, or they keep inherited pipes open.
- Verify every dashboard/layout key against a real log from the code; never copy keys from an old layout.
- Compare branches against the remote ref (origin/main), not a local branch that may be stale.
