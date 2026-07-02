# Code style

These rules expand CLAUDE.md rules 1 through 4 with the concrete shapes to use. When the
existing code disagrees with this document, the code is the bug.

## Comments

`#` only, lowercase, one line, and only where the code cannot say it. Never stack `#`
lines into a paragraph. The three things worth a comment: an invariant, a quirk being
worked around, a reason a guard exists.

Good:

```python
# sim publishes 270 deg over 1080 gaps, so the -90 deg ray is index 179 here
```

Delete on sight:

```python
# set the speed to zero
speed = 0.0
```

Docstrings are one lowercase line stating what the function does, only where the name
does not already say it. No Args/Returns/Raises blocks, no restating parameter names.

Good:

```python
def lap_progress(self, x: float, y: float) -> float:
    """arc-length progress along the centerline, wraparound-safe."""
```

Bad, and the shape most of the legacy code had:

```python
def lap_progress(self, x, y):
    """
    Computes the lap progress.

    Args:
        x: The x coordinate.
        y: The y coordinate.

    Returns:
        The lap progress.
    """
```

Every module opens with one `#` or docstring line naming what it does, nothing more.

## Naming

- snake_case functions and locals, PascalCase classes, SCREAMING_SNAKE constants.
- Constants carry their unit: `MAX_RANGE_M`, `TTC_FB_SEC`, `HOLDOFF_SEC`, `DT`.
- Domain words over generic ones: `scan`, `ranges`, `steering`, `raceline`, `progress`,
  `lookahead`, never `data`, `item`, `value`, `info`, `result`.
- Predicates read as questions: `is_clear`, `crossed_finish`, not `check_clear_flag`.
- ROS callbacks are `<topic>_callback`; keep that existing convention.

## Module boundaries

- One node per file, and the node file is glue: subscriptions, publications, parameters.
  Math and algorithm logic live in importable functions or classes that never touch rclpy,
  so they run under plain pytest.
- `gym_training/` never imports rclpy. ROS nodes never import from `gym_training/` except
  the shared obs-building code, which is dependency-free numpy by design.
- The exported `obs_config.json` is the only channel for train-time constants reaching
  the deploy node. Never copy a constant and comment "must match".
- Files past roughly 500 lines split along responsibility lines. Never split by half.

## Error handling

- Nodes never crash on a weird message: guard divisions, clamp indices, treat non-finite
  ranges as max range (the pattern the scan preprocessing already uses).
- Training scripts fail fast and loud: a bad config or missing file raises immediately
  with the path in the message. No silent defaults for anything that changes results.
- Checked math at the edges: clamp steering and speed at every boundary the policy
  crosses (train wrapper, export, deploy node), because each boundary has seen a bug.

## Prose

README, docs, commits, log messages: direct, specific, human. No em dashes anywhere.
Log messages are lowercase and shaped like what they report: `recovered, forward
clearance 1.2m`, `checkpoint saved at step 40000`. Commits are one lowercase tldr line.
