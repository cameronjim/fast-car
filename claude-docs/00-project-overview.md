# 00 — Project Overview

## Thesis

The sim-to-real gap on a 1/10 racer is dominated by tire/contact behaviour, localization
quality, and steering/actuator dynamics. This project attacks those three directly and
measures the result honestly.

**Headline question:** does a learned residual, trained in a carefully identified simulator,
improve on the strongest classical stack we can build — and by how much, measured on hardware
under a pre-registered protocol with confidence intervals?

Key structural decision: **the residual policy's base controller IS the classical baseline**
(raceline optimization + tracking). The comparison is base vs. base + residual — intrinsically
paired, immune to strawman baselines, and every outcome (improvement / null / regression) is a
reportable result.

## Scope: option A — marginal value of learning

In scope:

- Working vehicle with layered safety architecture
- Sim fidelity work: upgraded vehicle model + system ID, stratified by regime
- Tuned classical stack (raceline optimizer + tracker) with stated tuning budget
- Learned residual with enforced safety envelope
- Paired, interleaved, pre-registered hardware evaluation on ≥2 track layouts
- Sim-to-real fidelity study: trajectory error vs. proximity to the friction limit
- Parameter-drift record across sessions

Explicitly OUT of scope (cut without guilt; do not implement without being asked):

- Custom FOC / ESC firmware (that is a separate future project, option B)
- Custom sensor-hub PCB (a tiny off-the-shelf ingest MCU is in scope; see 11-hardware.md)
- TensorRT (profile PyTorch then ONNX Runtime first; revisit only if profiling demands it)
- On-board fine-tuning (stretch)
- MPCC baseline (stretch; if not implemented, results must say so explicitly)
- Zero-shot transfer as a requirement (it is a *reported measurement* before adaptation)

## Fidelity claims by regime (governs what the policy may do)

| Regime | Model | Claim |
|---|---|---|
| Linear (< ~60% friction limit) | single-track, linear tire | transfer claimed here |
| Transitional | single-track + Pacejka + load transfer | randomize heavily, moderate claim |
| Saturated / sliding | not modeled | no claim; envelope bounds the policy out |

## Honesty rules

- No citation of external results ("11.5% better", "beats MPC") as targets. Full citation +
  conditions only; the only number that matters is measured under this project's protocol.
- Negative results are deliverables, not embarrassments.
- Tuning budget parity is documented for every controller compared.

## The sentence the project exists to produce

> "On [surface], across two track layouts, a residual policy trained in an identified
> simulator changed lap time by X% [CI] relative to the tuned classical stack it rides on,
> with completion rates Y vs. Z and N safety interventions — and here is the fidelity curve
> that explains why."

Defensible whether X is positive, zero, or negative.
