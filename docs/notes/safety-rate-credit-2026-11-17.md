# safety_node's rate limiter carries unused time as a credit, 2026-11-17 (brief 49; nothing here has run on the car)

Owner decision 7 of `owner-decisions-2026-11-16.md` approves this change to `racer_safety`, and
nothing else in that package. The run links of the L5 Plug-and-Play canary are in the first comment
of the pull request (a note that held its own run links would change the head SHA the runs belong
to).

## The fault

`safety_node` hands the gate `dt_s`, the steady-clock time since its previous cycle, and the gate
allowed `rate * dt_s`. When a cycle runs late, the next one runs a few ms after it (rcl realigns the
wall timer to its phase). The late cycle was offered a large allowance it did not need, and that
allowance was thrown away. The catch-up cycle has a dt of 1 to 12 ms, so an ordinary planner step
(0.45 of the limit per 20 ms tick) was clipped. The L5 Plug-and-Play canary has been red on main
since PR 205 (`rate_limit` at t = 2.51 s with `dt=0.012115s`, and one `rate_limit` in each 1.0 m/s
stall case). The car's 2026-11-16 glitches (about 3 per minute in P blocks, within 5 s of the first
command) fit the same cause. That is inference, not a measurement.

## The rule

Per channel (steering, speed growth) the gate carries a credit in seconds, threaded by the caller.
With `T = SafetyLimits::control_period_s`, `dt = safe_dt(dt_s)` and `c` the credit that came in:

    allowance_s = min(c + dt, max(dt, T))
    used_s      = (change the limiter granted) / rate, in [0, allowance_s]
    c_out       = min(T, c + dt - used_s)

A garbage `dt_s` allows nothing and returns credit 0. A `c` that is not finite or negative counts as
0, above `T` as `T`. A `T` that is not above 0 gives the old rule. Both new fields default to 0, so
a caller that does not thread them gets the old rule bit for bit. `safety_node` sets both to 0 on its
fault path. The event detail, the messages and `config/vehicle_params.yaml` do not change.

## The bound

Over any run of consecutive cycles, with the credits threaded, `sum(used_s) <= sum(dt) + T`, so
`|output_k - output_j| <= rate * (sum(dt) + T)` for steering and for the summed speed growth, for any
sequence of cycle times. Per cycle `dt <= allowance_s <= max(dt, T)`. The weakening, plainly: the
bound used to be `rate * sum(dt)` with no slack and now adds one nominal period once per window, at
most 0.064 rad of steering and 0.19 m/s of speed at today's vehicle_params values. The long-run rate
is unchanged. Layer 3 is risk reduction, not a guarantee of safety.

What it fixes: a command stream that moves at share f of the limit by its own clock is never
clipped, whatever the cycle times, while the command's age at each cycle stays at or below
`T * (1 - f) / f` (24 ms at 0.45). An older command than that can still clip. That is intended.

## Tests

- `test_planner_start_safety` (racer_planner): main's planner in STARTING against the real gate with
  a stall of 1 to 3 cycles at every start tick and a catch-up dt of 1, 4, 8 and 12 ms. Red on main's
  gate (117 clips at the first commit), green with the credit and the rig threading.
- `test_rate_credit` (in `test_racer_safety_core`, so the coverage job counts it): table-driven tests
  of every clause, steering and speed each, and two property tests with fixed seeds, 20000 sequences
  of 200 cycles each: the bound over every window, the per-cycle cap, the credit invariants, and the
  fix (a stream at share f of the limit is never clipped). The fix test is red under the old rule.
- `test/ab`: the reference node threads the credit too (A = B = C = D), and three scripted node
  sequences (cycle gaps 20, 20, 52, 8, 20 ms against a planner ramp at 0.45 of the limit; an injected
  fault, then a catch-up cycle; the same without the fault).
- `racer_safety` gate-logic branch coverage stays at 100 per cent (415 of 415 branches).

## One thing this does not change

The gate decides that the limiter clipped by comparing its recomputed output (`previous + granted
change`) with the request, bit for bit. That sum can land one ulp off a request that is well inside
the limit (`0.0696 + (0.0120 - 0.0696)` is not `0.0120`), and the gate then publishes a `rate_limit`
event although nothing was clipped. It happens for pairs whose magnitudes differ by more than a factor
of 2, such as a steering command crossing zero over a long cycle. The events are unchanged by this
brief (event logging is as before), so the new tests count a rate_limit activation as a clip only when
the output differs from the request by more than 8 ulps, and print how many they let through.
