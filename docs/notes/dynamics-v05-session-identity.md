# Identity of "the 2026-11-15 session" of the v0.5 dynamics pre-registration

Written before any v0.5 fit and before any 2026-11-15 held-out part was read. It changes no frozen
file (`evaluation/prereg/pnp-v0.5-dynamics.yaml`, SHA-256
`2767f297d951c11683b40834b864b832e8cbd0b01343e2765681e11b22e86a82`, and the notes next to it).

## What the label means

`session_2026_11_15` in the yaml, and every "2026-11-15 session" in the pre-registration, name the
first car session after PR 196 merged, by the date it was planned for. It is a label for that
session, not a calendar date of the bags. Nothing else was driven between PR 196 and these bags.

## The bags

All five bags of that session, in `/Users/cameronjim/code/car-data/bags/`. Bag names are UTC (the
car clock); local time is UTC-8.

| Bag (UTC name) | Local time of the first message | Content (owner and launch logs) |
|---|---|---|
| 2026-11-16T02-18-54_car_teleop | 2026-11-15 18:18 | PnP cap 1.0 counter-clockwise, 20 laps |
| 2026-11-16T02-29-27_car_teleop | 2026-11-15 18:29 | short, battery was low |
| 2026-11-16T22-38-47_car_teleop | 2026-11-16 14:38 | PnP cap 1.0 counter-clockwise, 20 laps (GP3-R1) |
| 2026-11-16T22-45-39_car_teleop | 2026-11-16 14:45 | excitation blocks (per the owner's log), two aborted P launches early |
| 2026-11-16T23-04-09_car_teleop | 2026-11-16 15:04 | excitation blocks (per the owner's log) |

So the session spans local evening 2026-11-15 to local afternoon 2026-11-16, and the bag names carry
the date 2026-11-16 for all five.

## The rule that applies

The rule `session_2026_11_15` of the yaml applies to EVERY run of EVERY one of these five bags,
whatever drove it (PnP planner, gap follow, any launch) and on whatever layout: a run under 15 s or
under 1.0 m of wheel distance is unused; otherwise the first 55 percent of its duration is train,
the next 2.0 s unused, the rest held-out. No run is chosen, dropped or relabelled by hand. A
truncated bag is read as far as it goes and kept.

The split file is `evaluation/prereg/sessions/2026-11-15-dynamics-v05.yaml` (the planned date, as
the yaml names it), not `2026-11-16-...`. This note is a deviation from the literal date of the
bag names and is listed as such in the first PR comment for the owner.
