# Shutdown SIGSEGV in the L5 and e2e canaries (2026-11-14)

Nodes of a canary exited -11 (SIGSEGV) at the end of the test, after every assertion had passed.
This note is the root cause, the evidence, the fix, and what it does not claim.

## Root cause

A race inside Fast-DDS 2.6.12 (`libfastrtps`, the newest Humble ships; `apt-cache policy` shows no
newer candidate in the ros-dev image). Two teardowns of the same endpoint run at once:

* the process's main thread deletes its own subscription
  (`rmw_destroy_node` -> `rmw_fastrtps_shared_cpp::destroy_subscription` ->
  `RTPSDomain::removeRTPSReader` -> `RTPSParticipantImpl::deleteUserEndpoint`), and
* the same process's UDP receive thread handles the departure of a PEER participant
  (`PDPListener::onNewCacheChangeAdded` -> `PDP::remove_remote_participant` ->
  `PDP::actions_on_remote_participant_removed` -> `EDP::unpairWriterProxy`) and dereferences
  state the first thread is freeing.

launch_testing ends a test by sending SIGINT to every process it launched at the same moment, so
in a canary every participant is leaving while every other one is. The crash is in the middleware,
not in a racer node: our `main()` functions all tear down in the normal order.

## Evidence

* Two core dumps (2026-11-14, ros-dev image, 2 CPUs, 20 and 60 rclpy nodes with 24 and 8
  publishers plus subscribers each on shared topics, SIGINT to all at once, `destroy_node` on
  the way out as bridge_node does), `gdb /usr/bin/python3 core`. Identical in both:
  thread crashed: `EDP::unpairWriterProxy` <- `PDP::actions_on_remote_participant_removed` <-
  `PDP::remove_remote_participant` <- `PDPListener::onNewCacheChangeAdded` <- ... <-
  `UDPChannelResource::perform_listen_operation`; main thread at the same moment: blocked in
  `pthread_cond_wait` inside `RTPSParticipantImpl::deleteUserEndpoint` <- `removeRTPSReader` <-
  `DataReaderImpl::~DataReaderImpl` <- `destroy_subscription` <- `rmw_destroy_node`.
* CI run 43629406596 (L5 park canary): bridge_node's `PYTHONFAULTHANDLER` trace ends in
  `rclpy/subscription.py destroy` <- `Node.destroy_subscription` <- `Node.destroy_node` <-
  `bridge_node.py main`: the same main-thread half.
* PR 171 (gap_follow, `matched_writer_remove` on the receive thread) is the same race seen from
  the other side; PR 116's safety_node and issue 172's ekf_node fit it.
* CI history: `exit code -11` appears in 11 of the 80 failed runs among the last 289 runs of this
  repo (since 2026-10-28): wheel_twist_node 3, gap_follow_node 3, bridge_node, ekf_node,
  mppi_planner_node, park_node, safety_node one each.

## Fix

`tests/launch_support/orderly_shutdown.py`, called from the last `tearDownClass` of every L5 and
e2e launch_testing canary (14 files) right after the test's own `rclpy.shutdown()`. It SIGINTs the
launched processes newest first, one at a time, waits until each has exited, then waits 0.5 s so
the survivors have processed that departure. The oldest process (the sim bridge) is left for
launch_testing's own shutdown, so it too exits alone, and launch_testing does not mistake "all
processes gone" for a dead launch. At no point do two participants leave together.

It never kills, never reads or rewrites an exit code. The post-shutdown `assertExitCodes` of every
canary is unchanged, so a node that crashes for its own reasons still fails the test.

The ekf_node SIGSEGV tolerance that PR 173 added to
`tests/l5_localization/l5_localization_common.py` is removed: it hid exactly this crash class
and the canaries no longer produce it. No exit code is tolerated generically anywhere.

Not changed: `racer_safety`, `racer_drivers`, any node code, any launch file, any on-car behaviour.
The per-file distinct `ROS_DOMAIN_ID` of PR 171 stays (those tests need no peers). A per-test
domain would not help here: the racing participants are the test's own stack and must share a
domain.

Scope: the 51 launch_testing files are not all wired. The L5 and e2e canaries (multi-node stacks
with the sim bridge) are; the single-node L3 files each have one node plus the test's own client
and were not seen to crash this way except the already fixed ones.

## Numbers

All local runs: ros-dev image, pinned to 2 CPUs, serial, 2026-11-14.

| | Before (all processes SIGINT at once) | After (one at a time) |
|---|---|---|
| CI, last 289 runs since 2026-10-28 | `exit code -11` in 11 runs | 0 in this PR's last CI run |
| rclpy stress nodes, shared topics | 4 SIGSEGV in 8,790 exits | 0 in 500 exits |
| L5 park canary, real | 0 in 60 | 0 in 21 |
| L5 tracker lap canary, real | 0 in 20 | 0 in 21 |
| L5 localisation oval canary, real | 0 in 12 | 0 in 13 |

Every modified canary (13 files) also passed once with the change. Read the table for what it is:
the crash is about one participant exit in 2,000 here, so zero in a few hundred orderly exits does
not by itself prove an improvement. The proof is the mechanism (two core dumps, one CI
faulthandler trace, all the same two code paths) plus the fix removing the overlap by
construction. A widened window (a pause between subscription deletes, spread-out SIGINTs, six
busy loops) did not raise the rate either: 0 in 1,400 exits.

The stress nodes are `/stress_node.py` in the scratch container: rclpy nodes with 8 to 32
publishers and subscribers on shared topics, `spin_until_shutdown` then `destroy_node`, the shape of
bridge_node's exit. Not committed: they are a measuring tool, not a test.

One unrelated local flake seen on the way: the oval canary's heading p95 was 0.1225 against the
0.12 limit in 1 of 12 baseline runs on the 2-CPU pinned container. Not touched here.
