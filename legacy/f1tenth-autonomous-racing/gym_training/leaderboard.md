# Lap time leaderboard

Generated 2026-08-14.

Pure pursuit runs the raw 100 Hz env for 5 laps per map, one deterministic attempt, so its crash rate is 0% or 100%.

A centerline row means the map's shipped raceline runs closer to a wall than half the car's width, so no lap on it is possible at any speed.

A generated raceline row uses `f1rl.track.generate_raceline` output at the default 0.15 m margin, driven with `--raceline-csv`. Its speed profile is planned at the full friction limit rather than the shipped line's conservative one, so the fastest clean `speed_scale` differs per map: 1.15 on Spielberg, 1.35 on Monza, 1.12 on YasMarina. Everything else is the planner's tuned default.

| map | controller | best lap | mean lap | crash rate | top speed |
| --- | --- | --- | --- | --- | --- |
| Monza | pure pursuit | n/a | n/a | 100% | 9.45 m/s |
| Monza | pure pursuit (centerline) | 111.93 s | 112.00 s | 0% | 4.00 m/s |
| Monza | pure pursuit (generated raceline) | 42.04 s | 42.17 s | 0% | 10.80 m/s |
| Monza | rl residual (m5) | 39.21 s | 39.32 s | 0% | 11.88 m/s |
| Spielberg | pure pursuit | 37.99 s | 38.10 s | 0% | 9.60 m/s |
| Spielberg | pure pursuit (generated raceline) | 38.64 s | 38.74 s | 0% | 9.20 m/s |
| Spielberg | rl sac (m4, 9.5 cap) | 37.97 s | 38.03 s | 0% | 9.50 m/s |
| Spielberg | rl sac (m6 deploy, 8.0 cap) | 43.43 s | 43.45 s | 0% | 7.92 m/s |
| Spielberg | rl sac sb3 (m2, 3.0 cap) | 116.08 s | 116.14 s | 0% | 3.00 m/s |
| Spielberg | rl sac hand-rolled (3.0 cap) | 115.13 s | 115.20 s | 0% | 3.00 m/s |
| Spielberg | rl residual (m5) | 33.40 s | 33.42 s | 0% | 11.07 m/s |
| YasMarina | pure pursuit | n/a | n/a | 100% | 9.60 m/s |
| YasMarina | pure pursuit (centerline) | 100.75 s | 100.82 s | 0% | 4.00 m/s |
| YasMarina | pure pursuit (generated raceline) | 50.98 s | 51.08 s | 0% | 8.96 m/s |
| YasMarina | rl residual (m5) | 43.70 s | 43.71 s | 0% | 10.43 m/s |

The generated line turns Monza and YasMarina from undrivable into clean 5-lap runs, 2.7x and 2.0x faster than the centerline fallback they needed before. On Spielberg, where the shipped line already clears the car, it stays 0.65 s slower: the generator gives up roughly 0.3 m of corridor on each side to keep 0.31 m of wall clearance against the shipped line's 0.26 m.

The rl row is the m4 curriculum policy gated over 20 randomized-start episodes (80 laps, flying-lap mean), against pure pursuit's single deterministic attempt. It ties pure pursuit while driving the centerline-referenced reward at 25 Hz control; the artifacts live in `artifacts/m4/`.

## The deploy row

`rl sac (m6 deploy)` is the policy the ROS demo runs, and it is slower than the m4 row on
purpose. It trains without `frenet_pose`, so it drives on LiDAR, speed, yaw rate and its own
last steering command alone, and its curriculum stops at an 8.0 m/s cap rather than being
pushed to the 9.5 m/s ceiling. That buys reliability: 20/20 collision-free over 60 laps with
a flying-lap spread of 0.03 s, from a policy whose observation a ROS node can actually
rebuild. Trained 1.5M steps from scratch, no warm start, since m4's 116-dim policy cannot
seed a 113-dim one. Artifacts live in `artifacts/m6_deploy/`.

Driven live in `f1tenth_gym_ros` on the same map, the same policy laps in 63.8 s. The gap is
not the policy: the demo's safety node brakes to its PB1 and PB2 caps whenever forward time
to collision drops under its thresholds, which at 8 m/s happens on the approach to most
corners. `/drive_raw` stays in the 6.4 to 7.9 m/s band the policy asked for while `/drive`
dips to 2.0 m/s. The legacy physics in that bridge differ from the training sim too.

## Hand-rolled SAC

The two 3.0 m/s rows are the same task twice. `rl sac sb3 (m2)` is the M2 policy trained by
stable-baselines3; `rl sac hand-rolled` is the `SACTrainer` in `learned_control/sac`, the
implementation written for the ROS online trainer, run against the identical env through
`f1rl/train_handrolled.py`. Same `configs/sac_m2.yaml`: Spielberg, 3.0 m/s cap,
`RL_RANDOM_RANDOM` starts, the wrapper's progress reward, 116-dim observation, from scratch
with no BC warm start and no reference actor. Both rows are 20 deterministic episodes on the
same seeds. Their top-speed column is the cap, not a measurement: at 3 m/s both policies sit
on the cap for most of the lap.

The hand-rolled trainer clears it: 20/20 collision-free, 115.13 s best and 115.20 s mean
against SB3's 116.08 and 116.14. It laps 5/5 in evaluation by 100k steps, matches SB3's lap
time by 250k, and its evaluation curve is flat from there to the 600k budget, so the best
checkpoint at 350k is a plateau pick rather than a lucky round.

Reading that 0.95 s as "the student code beats the library" would be wrong twice over. The
task is easy enough that both policies are speed-capped almost everywhere, so the gap is a
racing line worth under a second, not a capability difference. And the two runs differ in
more than the algorithm:

| | sb3 | hand-rolled |
| --- | --- | --- |
| envs | 8 subproc workers | 1, the trainer has no vec support |
| net arch | [256, 256] actor and critic | [256, 128], fixed in `learned_control/sac/model.py` |
| batch | 512 | 256 |
| update cadence | 64 gradient steps per 512 transitions | 1 per 10 transitions |
| replay | 1M | 600k, so nothing is evicted either |
| wall clock to 600k | 42 min | 52 min |

The gradient-to-experience ratio comes out close, 1 update per 8 transitions against 1 per 10,
which is most of why the single-env run is competitive at all. What it costs is wall clock:
one env at 108 beams runs about 300 steps/s end to end, so 52 minutes buys what eight workers
buy in 42.

Two quirks of the hand-rolled implementation are worth naming because the adapter had to work
around one and could not fix the other. `SACActorNet.sample` squashes to `[0, 1]` rather than
`[-1, 1]`, so `train_handrolled.py` maps affinely at the env boundary and stores the `[0, 1]`
form in the replay buffer, keeping the critics' action space the one the actor emits. And the
log-prob it returns carries tanh's Jacobian correction only, one `log 2` per dimension short of
the true density of the halved action, so automatic entropy tuning settles about 1.39 nats more
deterministic than the nominal `-action_dim` target. Alpha collapsing to 0.006 by 200k steps and
staying there is that, not entropy collapse from a diverging critic: the policy keeps improving
after it happens.

Artifacts live in `artifacts/handrolled/`: the best checkpoint, the rolling checkpoint, the
20-episode gate log, the per-eval curve csv, and the full training log.

## Head to head

M7 is a race, not a time trial, so it gets outcomes rather than a lap-time row. The ego is a
residual SAC policy warm started from m5; the rival is `GapFollowerOpponent`, a port of
`reactive_control`'s gap follower running on `agent_1` at reactive_control's own tuned
constants. Both cars spawn on the Spielberg raceline facing forward, the ego 5 to 15 m behind
with 0.15 m of lateral jitter, and the opponent draws a fresh speed cap in 3.0 to 4.5 m/s each
episode. An overtake counts when the ego's unwrapped centerline lead passes 1.58 m, the car's
length plus a metre. Episodes run 18 s.

| driver | overtake success | ego collisions | mean time to pass |
| --- | --- | --- | --- |
| pure pursuit (zero residual) | 67.5% | 32.5% | 2.63 s |
| rl residual (m7) | 90.0% | 10.0% | 2.29 s |

Both rows are the same 40 episodes: two blocks of 20 seeds, reported together because one
block alone is not stable. The policy scores 95% / 5% on seeds 0-19 and 85% / 15% on seeds
100-119, and the planner 85% / 15% and 50% / 50%. Quoting only the better block would have
claimed 95%.

The zero-residual row is the same control m5 uses: a zero action reproduces the embedded
planner exactly, so this is the tuned pure pursuit driver meeting the same opponent on the same
spawns. It is a real driver rather than a straw man, and it already passes most of the time,
because a 3.5 m/s rival on a 12 m/s line gets swallowed. What it cannot do is avoid the rival
it fails to pass. The policy cuts collisions by a factor of three and passes half a second
sooner.

Two caveats. The opponent is much slower than the ego, so this measures "arrive at a slow car
at 11 m/s and get around it", not wheel-to-wheel racing between equals; the gap follower's own
clean ceiling on Spielberg is about 5.5 m/s against the residual's 11. And the residual bounds
here are wider than m5's, 0.25 rad and 4.0 m/s against 0.15 and 1.5, because at m5's bounds the
ego cannot drop below roughly 8 m/s and so can never slow behind a 4 m/s car. That change is
what made the run work, and it is why the m7 policy is not a drop-in m5 replacement.

Artifacts live in `artifacts/m7/`, including the gate logs for all four blocks and an mp4 of a
clean pass.

## The residual rows

`rl residual (m5)` embeds the tuned pure pursuit planner in the training env. The planner replans against the reference line every 100 Hz physics step and the policy adds a bounded delta at 50 Hz: at most 0.15 rad of steering and 1.5 m/s of speed, clipped to the vehicle's steering limit and a 12 m/s ceiling. A zero action is therefore the planner itself, which makes the baseline exactly reproducible: driven that way through the same wrapper over the same 20 randomized-start episodes, the embedded planner laps Spielberg in 37.99 s, matching its own row above. Mean lap is the flying-lap mean over 40 laps for Spielberg and Monza and 20 for YasMarina, all 20/20 collision-free.

Three caveats keep the comparison honest.

Control rate is not equal and cannot be made equal. Pure pursuit runs at 100 Hz because it is a closed-form geometric law. The residual policy runs at 50 Hz, but its 100 Hz planner still steers between policy calls, so the residual rows are not a 50 Hz controller beating a 100 Hz one: they are the same 100 Hz controller with a 50 Hz correction layered on. The m4 row, at 25 Hz with no planner underneath, is the only row that drives the car alone at a lower rate.

Attempt counts are not equal. Each pure pursuit row is one deterministic attempt, so its crash rate can only read 0% or 100%. Each residual row is a single deterministic policy replayed over 20 randomized-start episodes, and it is the best-scoring checkpoint of the run's evaluation rounds, 27 on Spielberg and 16 on each of the others, selected on collision-free rate first and lap time second. The planner rows carry no such selection, though their speed scales were themselves tuned over a handful of attempts in m3.

The Monza baseline differs from the row above it. The 42.04 s planner row uses `speed_scale` 1.35 from a fixed grid start on the line. Under the randomized off-line spawns the residual env trains on, 1.35 crashes 16 times out of 16, so the embedded planner runs at 1.30 and laps 43.62 s. The residual beats the 42.04 s number anyway, but it is worth naming that it started 1.6 s behind it. Spielberg (1.2) and YasMarina (1.12) keep their tuned scales unchanged.

Residual artifacts live in `artifacts/m5/`, `artifacts/m5_monza/`, and `artifacts/m5_yasmarina/`. The exported `obs_config.json` marks every reference-line feature `deployable: false`, because no ROS node can rebuild them from `/scan` and `/odom`: these policies are sim only until a deploy-side planner exists.
