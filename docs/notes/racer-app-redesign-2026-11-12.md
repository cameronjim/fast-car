# Racer app redesign (2026-11-12)

Web UI only (`tools/racer_app/web`). The agent API, the safety gating (required checks,
non-overridable recorder/stack/disk, override with a reason) and the STOP path are unchanged.

## Layout

- Persistent top bar: car name and connection pill, Ready, Recording (size and MB/s), battery.
- Persistent bottom STOP ALL bar with one short line: software stop only, keep a hand on the RC kill knob.
- Tabs (the last one is remembered): Drive, Live, Checks, Logs, System.
- Long explanations sit behind the info button on each card or inside an expandable row.

## Feedback model

- `feedback.ts`: every button is an `ActionRunner` (idle, pending, success, error). A second click while
  pending is ignored; the error reason shows under the button; outcomes also go to a toast stack.
- `ops.ts`: optimistic operations. Starting or stopping the stack, cameras, perception, a drive or STOP ALL
  shows the target state at once with a timeline (stack: requested, container up, nodes up, recorder up,
  ready). Only the agent's status poll and the ROS link advance a step. A refused request fails at once with
  the agent's reason; a target not reached in time shows as timed out and names the unmet step. Stack stop
  allows 11 minutes (the agent waits up to 600 s for the bag to close).
- `ConnectionTracker`: two failed polls in a row show a non-blocking "Reconnecting" banner with the last-seen
  time; values stay on screen, greyed, with their age.
- Skeletons show until the first status reply.

## Demo mode

`npm run demo` (or `?demo=1`) runs the UI against `demoAgent.ts` (fake agent, same API and refusals) and
`demoRos.ts` (fake subscribe-only ROS link). `&state=` picks the start state (idle, running, driving,
starting, offline, failed); `&panel=1` opens the side panel that drops the link, arms a failing start or
sets a low battery (which exercises the override flow). `npm run shots -- <dir>` takes light and dark
screenshots of every tab with a headless system Chrome (`playwright-core`, no browser download).
