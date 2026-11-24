# Preregistration pnp-v0.6, fused perception (fused-v2)

Registered 2026-11-23 under owner decisions 27 and 28. Constants: `evaluation/prereg/pnp-v0.6-perception-fused.yaml`,
SHA-256 `22d65de750ef56bcc90e33cc0a7a5a85eb69b5291d606ebd30fd452ed1e3e0b5`. Neither file is edited after the first commit.
The text below is the gate section of brief 55, unchanged.

## The gate

**GP1d and GP1d-K.** Candidate `fused-v2` (frozen `trackperc-v2` weights, no training) against
the geometric tracer, the deployed backend. Development: sim, the v0.2 train roles, H-run,
A-run, the 2026-11-12 layout, folds F1 and F2, and the 2026-11-16 bags. **Held-out, two sets,
each judged by itself: (14) the perception `heldout` segments of 2026-11-14 in `pnp-v0.4.yaml`,
never read by a perception pass; (22) runs R0 to R4 of the 2026-11-22 bag, whole.** H = labelled
scans, A = all moving scans (wheel speed at least 0.2 m/s). No seeds (nothing is trained);
bootstrap as v0.2 section 2. One pass after the freeze.

Precondition (development, before the freeze): mean coverage loss and excess at most 0.005 and
p99 of each at most 0.025 on EVERY development set (addendum 1, unchanged), and E_c(fused) at
most 0.95 times E_c(geometric) on every labelled development set. Not met: "GP1d not
attempted".

GP1d, thresholds of GP1c unchanged, on each held-out set:

1. At least 300 labelled scans. Fewer: that set is undecided.
2. E_c(fused) at most 0.90 times E_c(geometric), bootstrap upper end below 1.00.
3. V(fused) at least V(geometric).
4. Every cell (direction by speed bin 0.2 to 0.6, 0.6 to 0.85, 0.85 and up) with at least 40
   labelled scans: E_c(fused) at most 1.10 times E_c(geometric).
5. Label-free on A: mean intrusion at most max(0.02, geometric + 0.005); boundary residual
   median at most 0.10 m; clearance deficit p90 at most 0.05 m; C95 at least geometric minus
   0.02.
6. The v0.3 section 3.3 rule on the development sets (reported as development).
7. Latency p99 under 10 ms, 2000 real scans, one thread, tracer plus net plus fusion.
8. L5 PnP canary with the fused backend: every case completes, 0 wall contacts, median lap
   time at most 1.03 times the geometric backend's on the same case and seed.

GP1d-K, thresholds of GP1c-K unchanged, on A14 and on A22: mean coverage loss at most 0.01, p99
at most 0.05, mean excess at most 0.01, p99 at most 0.05.

Pass needs GP1d and GP1d-K on both sets: "candidate backend in the layout family it trained on,
on two fresh sessions". One set only: "not shown across sessions". Fails 2 or 3: "no better
than geometric in the layout". Passes 2 and 3, fails another item: "better where it trained,
not shown elsewhere". GP1d passes and K fails: "better centre, corridor not kept". Geometric
stays deployed in every case. Every report says: M-A2 measured under 3 percent of lap time for
a perfect track in sim, so a pass claims a more accurate centre, not a faster lap. GP1b stays
FAIL and GP1c stays "not attempted".
