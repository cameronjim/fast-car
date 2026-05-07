# 05 — Safety Architecture

Read this before writing ANY code that can move the car. This layering goes in the README
verbatim, because getting it wrong is how people get hurt.

## The layers

| Layer | Mechanism | Protects against | Nature |
|---|---|---|---|
| 1 | Hardware RC mux + power cutoff on an **independent MCU** (`firmware/safety_mux/`) | Jetson freeze, Linux hang, software crash, ROS deadlock | **GUARANTEE** |
| 2 | Firmware-level current/speed limits configured in the VESC | Bad commands from a live-but-wrong stack | risk reduction |
| 3 | ROS safety_node: TTC braking, `/drive_raw` → `/drive` gating, covariance gate, watchdog | Bad commands from a live-and-correct stack | risk reduction |
| 4 | Policy envelope: residual bounds, rate limits, OOD fallback, speed envelope | Bad learned behaviour | risk reduction |

**Only layer 1 is a guarantee.** Layers 2–4 reduce risk. Never describe 2–4 as guarantees in
code comments, docs, or the README.

## Layer 1 rules

- The mux MCU shares no code, power rail, or failure mode with the Jetson.
- Kill is tested by ACTUALLY freezing the Jetson (roadmap 1.3), not by reasoning about it.
- No software task may reconfigure, reflash, or route around the mux as a side effect.

## Layer 3: safety_node (racer_safety, C++)

- Sole publisher of `/drive`. Enforces: TTC braking from `/scan`; speed cap (global +
  covariance-gated); command sanity (bounds, rate limits vs. `vehicle_params`); watchdog on
  `/drive_raw` staleness.
- Emits `/safety/events` on every intervention. Interventions are evaluation data (09) —
  an unlogged intervention is a bug.
- Fails CLOSED: any internal error → brake command, not passthrough.

## Layer 4: policy envelope (envelope/ library)

Enforced in the DEPLOYMENT NODE, not learned, not trusted from the policy:

- Hard bounds on residual magnitude as a fraction of base-controller command range
- Rate limits on residual change
- OOD detection (critic disagreement / distance from training state distribution) →
  fall back to pure base controller
- Speed envelope derived from the fidelity study: the policy may not command into the
  regime where the model is known unfaithful (saturated/sliding)

The same library runs inside the training environment so the policy never learns behaviors
the deployment will clip — train/deploy envelope divergence is a correctness bug.

## Operational rules

- Rail voltage logged on every run; a brownout must never be mistakable for a control failure.
- Wheels-off-ground bench test before any code change first drives the car.
- Two-person rule for first runs of new control code (driver on RC kill + operator).
- Every session starts with the kill-switch check and the 10-min re-ID battery (07).
