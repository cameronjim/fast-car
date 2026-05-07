# Physical 1/10 Autonomous Racer — Project Plan

Draft v0.3. Revised from v0.2. v0.2 fixed the honesty and safety problems in v0.1; v0.3 fixes
a structural contradiction that v0.2 itself introduced, resolves two concrete hardware
conflicts the BOM created, and upgrades the evaluation and schedule from lists of good
intentions into designs. The **[ACCEPTED]/[DISAGREE]** markers from v0.2 (responses to the
original critique) are preserved where those positions stand; changes new in this revision
are marked **[v0.3]**.

**What changed, in one paragraph:** v0.2's headline goal (§1) required beating the exact
method class that v0.2's own §8 cites as the fastest known on this platform — a plan whose
most likely outcome was a self-defined failure. v0.3 resolves this by connecting two things
v0.2 left dangling: the residual policy's unnamed base controller is now *the classical
baseline itself*, so the headline becomes the marginal value of learning — a paired
comparison in which every outcome is a publishable result. Downstream of that, the BOM's
wheel-sensor spec contradicted its chassis choice, the sync design assumed an MCU that
scope option A had cut, the evaluation design would have confounded controller effects with
session drift, and the schedule was a serial chain with no decision gates. All fixed below.

---

## 1. Thesis (restructured — v0.2 contradicted itself)

### [v0.3] The headline claim and the baseline evidence could not both be true

v0.2 carried a contradiction between two of its own sections. §8 defends the classical
baseline by citing the F1TENTH benchmark survey (arXiv:2402.18558): offline trajectory
optimization plus tracking achieved the *fastest* lap times on this platform, ahead of MPCC,
with end-to-end RL well behind. §1 then defines headline success as a learned controller
*beating* that baseline. Read together: the plan's success criterion is to outrun the method
the plan's own evidence says is state of the art, with one person's tuning budget. That is
not a stretch goal, it is a plan whose expected outcome is failure by its own definition —
and it re-creates exactly the overclaiming v0.2 was written to remove.

**Resolution.** v0.2 also left a thread dangling: §7 built an envelope around a *residual*
policy but never said what the base controller was, and §8 defined baselines without
connecting them to §7. Connect them: **the residual's base controller is the tuned
raceline-optimization + tracking stack — the same artifact as the primary baseline.** The
headline question becomes:

> Does a learned residual, trained in a carefully identified simulator, improve on the
> strongest classical stack we can build — and by how much, measured on hardware under a
> pre-registered protocol with confidence intervals?

This framing has three properties the v0.2 version lacked:

- It is an intrinsically **paired** comparison — same base controller, same raceline, same
  tuning, with and without the residual — which is both statistically stronger and immune to
  the strawman-baseline failure mode by construction (§8).
- **Every outcome is a result.** Improvement: the headline number. No effect: the fidelity
  study (§6) explains what the sim work did and didn't buy. Regression: §11's differential
  diagnosis becomes the writeup. The pre-registration in §10 names all three in advance.
- It is consistent with the survey evidence instead of betting against it.

What custom firmware buys and does not buy: unchanged from v0.2. It remains a scoped
secondary contribution under option B, not the enabling one.

---

## 2. Scope

Unchanged in structure. Option A stands as recommended, retitled to match §1:
**A — Marginal value of learning.** Options B and C stand as written in v0.2.

**[v0.3]** One addition: descoping is now governed by explicit go/no-go gates in the schedule
(§12), with option C as the pre-named fallback at each gate. v0.2 said "cut without guilt"
but gave no mechanism; without gates, descoping happens as a slow slide six months in rather
than a decision made on a stated date with stated criteria.

---

## 3. Open questions

**[v0.3]** Split into blocking and non-blocking, because v0.2 listed seven questions without
saying which ones stop the purchase order.

**Blocking (answer before buying anything):**

1. Do you already own a Jetson Orin Nano?
2. Budget ceiling?
3. **Venue:** where will you drive, can you book it repeatedly, and what is the surface?
   This is now triple-blocking: it gates the evaluation plan (as in v0.2), it determines
   tire choice (§5), and it determines where system ID happens — fitted tire parameters
   transfer only to the surface they were fitted on (§6).
4. **[v0.3] 2WD or 4WD chassis?** v0.2's BOM made this decision implicitly and
   contradictorily — see §5. It must be made consciously.

**Non-blocking (answer during Phase 0):**

5. What is the second Windows desktop?
6. Timeline and weekly hours, honestly stated.

**Answered:** scope option is A (§2). Zero-shot is a reported measurement, not a
requirement — v0.2 §7's adaptation-first ordering stands.

---

## 4. Machine roles and environments

Stands as revised in v0.2: separate pinned images with a shared source tree, no dual boot,
Mac never talks to hardware directly, ROS distro pinned by JetPack version, tooling demoted
to `docs/conventions.md`.

**[v0.3]** One sharpening: the JetPack + ROS distro decision and the port of the Foxy-era
`f1tenth_gym_ros` bridge are explicit Phase 0 deliverables, not incidental discoveries. The
bridge port is a known cost; schedule it as one.

---

## 5. Bill of materials

### [v0.3] The wheel-sensor spec contradicts the chassis spec

v0.2 called wheel speed sensing "on undriven or all wheels" non-negotiable for slip
estimation — and specified a **Traxxas Slash 4x4**, which has no undriven wheels. On a 4WD
car every wheel is driven, so every wheel speed decorrelates from ground speed *exactly* when
you need the truth: under longitudinal slip. The v0.2 BOM as written buys sensors that lie in
the regime the project is about. Two coherent configurations:

| | 2WD chassis | 4WD chassis (Slash 4x4) |
|---|---|---|
| Ground-speed reference | Free — undriven front wheels are near-truth | None from wheels; must be estimated |
| Grip / pace | Lower | Higher |
| F1TENTH parts/community compatibility | Weaker | Standard |
| Phase 2 estimator burden | Lower | Higher — ground speed from IMU + LiDAR odometry fusion |

**Recommendation: keep the 4x4** — grip, pace, and parts compatibility matter more, and
estimating ground speed by fusion is representative of the real problem rather than a
workaround. But this raises the stakes on Phase 2 and must be a conscious choice, which is
why it is now blocking question 4 in §3. Wheel speed sensors stay in the BOM either way:
on a 4x4 they measure driven-wheel speed, which combined with an estimated ground speed *is*
the slip signal.

### [v0.3] Additions to the core BOM

- **Independent lap timing** (~$50): an IR break-beam gate or fixed overhead camera at the
  start/finish line. The evaluation (§10) must not time laps using the onboard state
  estimator, because the estimator is part of the system under test.
- **Track furniture** (~$50–100): foam or flexible barriers and cones. A car evaluated at
  the friction limit will leave the track; what it hits is a budget line, not a surprise.
- **Tires matched to the venue surface, plus one spare set of the same compound.** Tire
  compound is part of the vehicle's identity for system ID and evaluation; wear state is a
  controlled variable in §10.

### [v0.3] Synchronization: v0.2 assumed hardware that option A had cut

v0.2's sync section floated "a single MCU timestamping everything" while option A explicitly
cuts the custom sensor hub. Resolve minimally: **a small off-the-shelf MCU board (RP2040 or
Teensy, ~$15) reads the IMU, wheel-speed sensors, and steering pot, timestamps them against
one clock, and streams to the Jetson.** This is an ingest board with a week of glue firmware,
not the Phase 2+ sensor hub project — the distinction is that it does no fusion, no
filtering, and has no custom PCB. LiDAR and VESC keep their own timelines, aligned by
measured, documented offsets. The deliverable stands as in v0.2: expected residual jitter
and its effect on state estimation at target speed, written down.

### Positions that stand from v0.2

Sensored motor: strongly recommended, ~$30 insurance, not a law. Power: either a properly
sized supply with headroom or a separate pack, with **rail voltage logged on every run** as
the actual non-negotiable. Core and Phase 2+ line items otherwise unchanged; budget rises
slightly to roughly **$2000–2550 CAD** with the v0.3 additions.

---

## 6. Vehicle model and system ID

The regime-stratified structure from v0.2 stands: claim transfer in the linear regime,
randomize heavily in the transitional regime, do not claim the saturated regime. Model
upgrades and the ID protocol stand. The headline fidelity deliverable stands: sim-vs-real
trajectory error as a function of proximity to the friction limit.

**[v0.3] Three additions:**

- **Validate on held-out maneuvers, not fit residuals.** A model scored on the data it was
  fitted to will look better than it is. Reserve a maneuver class (e.g. figure-eights) for
  validation only, and report validation error, not training error.
- **Parameters drift.** Tire wear, temperature, and surface dust change the vehicle
  session-to-session. Define a **standard re-ID battery** (two step responses, one
  constant-radius sweep, one coastdown — about ten minutes) run at the start of every driving
  session. The parameter-drift record across sessions becomes a deliverable in its own right,
  and it replaces intuition as the justification for domain-randomization ranges — v0.2 said
  "ranges justified by fit residuals"; drift data is even better.
- **ID is surface-specific.** All fitting and all evaluation happen on the venue surface from
  §3. Parameters fitted in a parking lot say nothing about a gym floor.

---

## 7. Learning approach

The safety envelope (hard residual bounds, rate limits, OOD fallback, speed envelope, all
enforced in the deployment node), the adaptation-first ordering with zero-shot reported
alongside, the action-rate-penalty-as-last-resort stance, and the citation discipline all
stand as written in v0.2.

**[v0.3] Three additions:**

- **The base controller is named** (this was v0.2's dangling thread): the tuned raceline
  tracker from §8. The residual observes the same interface and is bounded per the envelope.
  "Fallback to pure base controller" in the OOD case now means something concrete: fall back
  to the baseline, which is independently validated in Phase 4 before any policy runs.
- **Commit to boring choices up front:** SAC or PPO on low-dimensional state plus downsampled
  LiDAR; reward = progress along the raceline, minus crash, minus envelope violation.
  Nothing else. Every shaping term added later is logged and treated as evidence of a
  modeling defect, extending v0.2's action-rate logic to the whole reward.
- **"Held out track" is now defined** (v0.2's §10 used the phrase without semantics): the
  raceline optimizer and tracker are given the new track's map — that is their job and it is
  not cheating. The residual's *weights are frozen*. The test is whether the learned
  component generalizes across tracks, not whether it memorized one.

---

## 8. Baseline

### [PARTIAL DISAGREE from v0.2 — upheld and strengthened]

v0.2's position stands: pure pursuit on an optimized raceline is not the weak baseline it
sounds like, per the survey; "well-tuned" does enormous work; MPCC is a stretch goal that is
reported as absent if absent; tuning budgets are documented.

**[v0.3]** The §1 restructure strengthens this section's incentives. The classical stack is
no longer only the yardstick — it is the base the residual must improve on. Underinvesting in
it now directly weakens the headline result rather than flattering it, so the strawman
failure mode is eliminated by construction, not by discipline. Tuning-budget parity now
applies to the pair actually compared: base vs. base + residual.

---

## 9. Software architecture

The layered safety table (hardware mux as the only *guarantee*; firmware limits, ROS safety
node, and policy envelope as risk reduction), the full deployment contract with
refuse-on-mismatch, the `vehicle_params.yaml` schema with generated bindings, and the
TensorRT deferral all stand as written in v0.2.

**[v0.3] Two changes:**

- **Build vs. adopt, stated explicitly.** v0.2's "C++ from the start" was right about the
  language and silent about authorship. Adopt existing, proven C++ nodes first — the F1TENTH
  stack's particle-filter localization, `robot_localization` or an equivalent EKF — and
  replace a component only when profiling or accuracy data indicts it. Hand-writing an EKF
  in Phase 2 is precisely how Phase 2 becomes the overrun v0.2 predicted it would be.
- **Localization is a named design, not a phase label.** Offline SLAM map built once per
  track; LiDAR particle-filter localization against it at runtime, fused with ingest-board
  odometry (§5). Failure behavior is explicit: a covariance gate in the safety node slows
  the car when the pose estimate degrades, rather than trusting a diverged pose at speed.
  Pre-named fallback if localization cannot hold at target speed: lower the speed cap and
  report the envelope actually achieved. A slower car with honest numbers is a result; a
  fast car with a diverged estimator is a crash log.

One consequence acknowledged: even scope option A includes two small firmware artifacts —
the layer-1 mux MCU and the ingest board. Both are glue-scale, neither is "the custom
firmware project," and saying so in the README prevents scope-creep-by-euphemism.

---

## 10. Evaluation protocol

Everything in v0.2's list stands: ≥20 laps per controller per track, mean and 95% CI, DNF
and crash rates reported, interventions counted, identical speed caps, tuning parity, two
track layouts, battery window controlled, surface and temperature logged, pre-registration.

**[v0.3] The list becomes a design:**

- **Paired and interleaved, never blocked.** Alternate controllers within each session
  (randomized ABBA-style ordering), because tire wear, motor and ESC temperature, battery
  health, and surface dust all drift monotonically across a session. Running all of A then
  all of B converts that drift into a fake controller effect. Interleaving plus the §1
  pairing is what makes 20 laps per condition worth anything.
- **Lap timing comes from the independent gate (§5), not the onboard estimator.** The
  estimator is part of the system under test; it does not get to grade itself.
- **DNF handling pre-stated:** lap-time statistics are computed over completed laps only and
  reported alongside completion rate; no imputation, no dropping crashes from the narrative.
- **Statistical honesty about n:** at 20 laps, one crash moves the crash rate five points.
  Report rates with intervals (Wilson) and do not claim crash-rate *differences* the sample
  cannot support. Lap-time differences are the primary endpoint; safety metrics are reported,
  not tested, at this sample size.
- **Pre-registered outcomes, per §1:** improvement, null, and regression each have a named
  interpretation and a planned writeup before the first evaluation lap is driven.
- **Per-session re-ID battery (§6) runs before evaluation sessions too** — it is the drift
  detector that certifies the car is the same vehicle it was last session.

---

## 11. Failure diagnosis

The differential list from v0.2 stands, including "check the rail voltage log first" and
"replay from rosbags, never debug by re-driving."

**[v0.3]** One addition: the re-ID battery doubles as the first diagnostic. Any "the policy
got worse" report begins by checking whether the *car* changed — parameter drift is cheaper
to detect than policy regression and masquerades as it.

---

## 12. Schedule

### [v0.3] The serial chain becomes two tracks with gates

v0.2's phases were strictly sequential, which idles the sim work during every parts delay
and hardware fight. Corrected: two parallel tracks at ~12 h/week, merging at Phase 3.

**Hardware track:** Phase 0 (environments, bridge port) → Phase 1 (chassis, power, layer-1
safety, teleop) → Phase 2 (sensors, ingest board, sync, state estimation, localization).

**Sim track (needs no car):** model upgrades from §6, training pipeline, envelope
implementation, baseline raceline optimizer and tracker running in sim. This proceeds during
part waits and hardware debugging.

**Merge at Phase 3:** system ID requires the instrumented car; everything after it is
sequential as before (ID → baseline tuned on hardware → policy training and deployment →
evaluation → writeup assembly).

**Go/no-go gates, each with option C as the pre-named fallback:**

| Gate | Criterion | On failure |
|---|---|---|
| G1, end Phase 1 | Teleop with layer-1 safety demonstrated, kill tested | Hardware problem; nothing downstream matters until fixed |
| G2, end Phase 2 | Localization holds at ≥70% of target speed for 20 consecutive laps | Lower the speed target, or pivot to option C with the sync/estimation study as the result |
| G3, end Phase 4 | Baseline laps reliably under the full §10 protocol | The evaluation is impossible for *any* controller; fix before training a policy |

**Duration: still roughly 9–11 months.** Parallelism buys slack against the overruns v0.2
predicted (Phase 2 above all), not a shorter calendar, at fixed weekly hours. Two further
corrections: **writeup is continuous** — per-phase notes and the drift record are written as
they happen, and Phase 7 is assembly, not authorship, because nobody reconstructs month 3's
debugging in month 10; and **~10% of schedule is repair time**, because crashes are a
scheduled cost of evaluating at the limit, not an exception.

Option B remains a separate 3–4 month project on a separate timeline, sharing the vehicle.

---

## 13. Deliverables

Reframed to match §1; contents otherwise as in v0.2.

- Working vehicle with the documented, layered safety architecture
- Sim-to-real fidelity study: trajectory error vs. proximity to the friction limit
- A tuned classical stack (raceline + tracker) with a stated tuning budget — both the
  baseline and the residual's base
- A learned residual with an explicit, enforced safety envelope
- A paired, interleaved, pre-registered evaluation: lap-time CIs, completion rates,
  intervention counts, independent timing
- The parameter-drift record across sessions
- Honest negative results where they occur

The headline sentence the project is built to produce, whatever the numbers turn out to be:

> "On [surface], across two track layouts, a residual policy trained in an identified
> simulator changed lap time by X% [CI] relative to the tuned classical stack it rides on,
> with completion rates Y vs. Z and N safety interventions — and here is the fidelity curve
> that explains why."

That sentence is defensible if X is positive, zero, or negative. v0.2's version was only
defensible if it won. That is the difference between this revision and the last one.
