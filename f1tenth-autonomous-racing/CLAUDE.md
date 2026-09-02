# F1TENTH Autonomous Racing

Autonomous racing software for the F1TENTH 1/10-scale platform. Two ROS 2 packages hold
the classical controllers and the learned controllers, and a plain-Python `gym_training/`
package (in progress) trains RL policies directly against the F1TENTH Gym simulator,
hundreds of times faster than real time. The physical car is retired: everything targets
the simulator now, and the ROS 2 side exists to demo trained policies and to keep the
classical baselines runnable.

Read this file first, then the guide you need:

| Document | Covers |
|---|---|
| `claude-docs/architecture.md` | Packages, nodes, topics, how sim and training fit together |
| `claude-docs/code-style.md` | Comments, naming, module boundaries, error handling |
| `claude-docs/master-plan.md` | The RL overhaul: phases, milestones, what is done and what is next |
| `claude-docs/testing.md` | What to test and the bar for calling a change done |
| `docs/` | Deep dives on the algorithms (reactive control, learned control) |
| `README.md` | The front door: what this is, quick start, topics table |

## Non-negotiable rules

**1. One responsibility per module.** A node file holds one node. Algorithm logic that
does not need ROS lives outside the node file so it can run and be tested without ROS.
Training code never imports rclpy; deploy nodes never import training code. Files past
roughly 500 lines split along responsibility lines, never by line count.

**2. Comments are lowercase, one line, and rare.** `#` only, never stacked into
paragraph blocks. A comment earns its place only for an invariant, a quirk, or a reason
the code cannot show. Docstrings are one lowercase line, no Args/Returns boilerplate.
Every module opens with one line naming what it does. Details and examples in
`claude-docs/code-style.md`.

**3. Naming carries meaning.** snake_case functions and locals, SCREAMING_SNAKE
constants with the unit in the name (`MAX_RANGE_M`, `TTC_FB_SEC`), domain words over
generic ones: it is `scan`, `steering`, `raceline`, `progress`, never `data`, `value`,
`info`.

**4. Prose reads human.** README, docs, commit messages: direct, specific, no em dashes,
no filler, no marketing voice. Commits are a single lowercase tldr line ("fix safety node
recovery"), authored under the user's git identity only, never with Claude attribution.

**5. The train/deploy contract is a file, not a convention.** Anything the trained policy
needs at inference (LiDAR downsample indices, clip range, normalization, action bounds)
lives in the exported `obs_config.json` next to the policy weights. The deploy node reads
it; nothing is duplicated by hand. Constants copied between files with a "must match"
comment are the bug this rule exists to prevent.

**6. Sim facts are read from the sim, not assumed.** No hardcoded 1080-ray indexing, no
baked-in 0.25 deg/ray factors: use `angle_increment` and scan length from the message.
The one sanctioned assumption is the ROS safety layer's forward-cone geometry, which is
parameterized in `config/*.yaml`.
