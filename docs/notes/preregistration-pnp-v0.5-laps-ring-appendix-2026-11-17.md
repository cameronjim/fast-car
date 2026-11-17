# Appendix to the lap gate v3 pre-registration: owner decisions given, 2026-11-17

Dated appendix. `preregistration-pnp-v0.5-laps-ring.md` and `evaluation/prereg/pnp-v0.5-laps-ring.yaml`
are frozen and not edited: the yaml's SHA-256 is a constant of the tooling and a check on every run,
so the `asked_not_given` fields stay as written and this appendix records the answers.

Source: `claude-docs/owner-decisions-2026-11-16.md`, item 11. The owner answered all four questions of
section 8 of the pre-registration YES:

1. A D8-S0 plus D8-S1 pass, with D8-C0 in the session, lets D v0 drive (D3-S0 and D7-S0 stay FAIL).
2. That pass stands in for "D3-S0 and D3-S1 passed" in front of B5 and B6.
3. The aliased-class tolerance (4 percent of the lap, floor 0.3 s) and the count rule are accepted as
   the measure of a lap gate where no start line can be seen.
4. The D8-C0 reference is the camera lap counter, with the phone video as backup.

Nothing else changes. Section 6 still applies in full (the freeze commit is what runs on the car,
D8-C0 passed earlier in the same session, lap-1 cap 0.5 m/s, 10 laps per direction, no D v1), and
the development checks of brief 50 gate the freeze: A5 failed (`lap-gate-v3-dev-2026-11.md`).
