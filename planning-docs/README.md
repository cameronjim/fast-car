# Planning docs: from parts arriving to the finished project

This folder is the step-by-step plan for the whole physical project, written for a two-person
build team (Cameron: software, Jetson, ROS; teammate: electrical/mechanical, soldering, wiring,
mounting) with Claude as the third pair of hands for guidance, code, analysis, and doc upkeep.
It sits on top of the project's authoritative rules in `claude-docs/` (ask Cameron for that
folder, it is deliberately not in git) and the roadmap in `claude-docs/01-roadmap.md`. When
this plan and those docs disagree, the docs win and this plan gets corrected.

Read these in order. Each file is one stage with numbered steps, a time estimate, a done
criterion, and where results get committed.

| Stage | File | Roadmap tasks | Rough time |
|---|---|---|---|
| 1 | `01-arrival-and-inventory.md` | prep | 1 to 2 hours |
| 2 | `02-bench-prep-and-soldering.md` | 1.1, 1.2 prep | 3 to 5 hours (shop session) |
| 3 | `03-chassis-and-drivetrain.md` | 1.1 | 3 to 5 hours |
| 4 | `04-power-tree.md` | 1.2 | 3 to 4 hours |
| 5 | `05-safety-mux-and-kill-test.md` | 1.3, gate G1 | 6 to 10 hours |
| 6 | `06-vesc-config-and-jetson-bringup.md` | 1.4, layer 2 | 3 to 5 hours |
| 7 | `07-first-drives-teleop-and-logging.md` | 1.5, 1.6, G1 sign-off | 3 to 5 hours |
| 8 | `08-phase2-sensors-and-localization.md` | 2.1 to 2.8, gate G2 | 4 to 8 weeks part-time |
| 9 | `09-phase3-system-id.md` | 3.1 to 3.5 | 2 to 4 weeks |
| 10 | `10-phase4-classical-baseline.md` | 4.1 to 4.3, gate G3 | 2 to 4 weeks |
| 11 | `11-phase5-policy-training-and-deployment.md` | 5.1 to 5.4 | 3 to 6 weeks |
| 12 | `12-phase6-evaluation-and-writeup.md` | 6.0 to 7.1 | 3 to 5 weeks |

Stages 1 to 7 are "the build": 20 to 35 hours of hands-on work, realistically three to five
weekends for first-timers, ending with a car you can drive by RC and by keyboard through the
computer, with a proven hardware kill switch. Stages 8 to 12 are the research project proper
and follow the calendar in `PLAN.md` (roughly 9 to 11 months total at about 12 hours a week).

## Rules that apply to every stage

- Two-person rule: any step that can move the car has one person on the RC kill switch and a
  separate person operating. Never a powered test alone.
- Wheels off the ground for the first test of any new wiring, firmware, or control code.
- LiPo batteries: charge only with the SkyRC balance charger, only while present, on a hard
  surface, stored in the fireproof bag. Never below 3.3 V per cell, never left on the charger.
- Multimeter before power: every new connection gets a continuity check (no shorts between
  battery + and -) before a battery is plugged in.
- Every measurement gets written into `config/vehicle_params.yaml` or a note in `docs/notes/`
  the same day, then bindings are regenerated with `python3 tools/gen_params.py`. A number that
  lives only in someone's head or a terminal did not happen.
- Photograph everything as built and commit photos to `docs/notes/`. Send Claude photos of
  parts, joints, and wiring for a second opinion before power flows.
- The roadmap box for a task gets ticked only when its done criterion is met, not before.

## The three gates

- G1 (end of stage 7): teleop works with layer-1 safety demonstrated, and the kill switch is
  proven by actually freezing the Jetson. Nothing downstream matters until this passes.
- G2 (end of stage 8): localization holds at 70 percent or more of target speed for 20
  consecutive laps. Fail means lower the speed target or pivot to the sync/estimation study.
- G3 (end of stage 10): the classical baseline laps reliably under the full evaluation protocol.
  Fail means the evaluation is impossible for any controller; fix before training a policy.

## Who does what

- Teammate (elec/mech): soldering and connectors, chassis work, motor swap, power wiring,
  mux board wiring, mounting, bench measurements, scope and multimeter work, the physical side
  of every bench procedure.
- Cameron: VESC Tool, Jetson flashing and the car image, ROS bring-up, driving software, all
  code changes and PRs, the software side of bench procedures, data and analysis.
- Claude: reads photos and screenshots, walks through each step, writes and fixes code and docs,
  updates `vehicle_params.yaml` and regenerates bindings, drafts bench procedures, runs the
  analysis. Ask early and often; that is cheaper than a redo.
