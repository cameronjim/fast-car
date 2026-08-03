"""Headless smoke test for the sim-cpu image (roadmap task 0.2).

Proves the two things claude-docs/03-environments.md and 01-roadmap.md
require of this image:

  1. racer_gym (currently upstream f1tenth_gym, pinned commit — see
     pyproject.toml) constructs and steps headlessly: no display, no
     rendering, finite observations, for a few hundred steps with a
     trivial controller.
  2. CPU-only torch is importable and a basic CPU tensor op works (proof
     that no CUDA build leaked in and that torch is usable at all).

This script is run as the container's CMD in CI (see
.github/workflows/ci.yml, job sim-cpu-image) after `docker build`. It exits
nonzero on any failure — that failure is the whole point of building the
image in CI rather than trusting it blind.
"""

from __future__ import annotations

import sys

import numpy as np

NUM_STEPS = 300


def build_env():
    """Construct a headless f1tenth_gym env on a synthetic reference line.

    Uses Track.from_refline (a synthetic sinusoidal centerline) rather than
    a named map like "Spielberg": named maps are fetched over the network
    from api.f1tenth.org at first use, which would make this "headless
    smoke test" secretly depend on an external host being reachable at
    container-build/run time. A synthetic track needs no network access
    and is fully reproducible from the pinned f1tenth_gym commit alone.
    """
    import gymnasium as gym
    from f1tenth_gym.envs.track import Track

    xs = np.linspace(0, 50, 100)
    ys = np.sin(xs / 3.0) * 3.0
    velxs = np.full_like(xs, 3.0)
    track = Track.from_refline(x=xs, y=ys, velx=velxs)

    env = gym.make(
        "f1tenth_gym:f1tenth-v0",
        config={
            "map": track,
            "num_agents": 1,
            "observation_config": {"type": "kinematic_state"},
        },
        # No display, no rendering: render_mode is left at its default
        # (None), and env.render() is never called anywhere in this file.
        render_mode=None,
    )
    return env


def assert_finite_obs(obs: dict, step: int) -> None:
    agent_obs = obs["agent_0"]
    for key, value in agent_obs.items():
        arr = np.asarray(value, dtype=np.float64)
        if not np.all(np.isfinite(arr)):
            raise AssertionError(f"non-finite observation at step {step}: {key}={value!r}")


def run_gym_smoke_test() -> None:
    env = build_env()
    try:
        obs, _info = env.reset()
        assert_finite_obs(obs, step=-1)

        # Trivial controller: constant modest speed, mild steering
        # oscillation. The point is exercising step()/reset() and the
        # dynamics/observation pipeline end to end, not tracking well.
        steer = 0.05
        speed = 2.0

        completed_steps = 0
        for step in range(NUM_STEPS):
            steer = -steer  # oscillate so the steering actuator model is exercised
            action = np.array([[steer, speed]], dtype=np.float64)
            obs, _reward, terminated, truncated, _info = env.step(action)
            assert_finite_obs(obs, step=step)
            completed_steps += 1
            if terminated or truncated:
                # A synthetic straight-ish reference line with a trivial
                # controller should not terminate in 300 steps, but if it
                # ever does (e.g. collision), that is still a completed
                # episode, not a failure of "runs headless".
                break

        if completed_steps == 0:
            raise AssertionError("gym env completed zero steps")

        print(
            f"f1tenth_gym headless smoke test OK: {completed_steps} steps, "
            f"terminated={terminated} truncated={truncated}"
        )
    finally:
        env.close()


def run_torch_smoke_test() -> None:
    import torch

    if torch.cuda.is_available():
        # This image must NEVER contain CUDA (claude-docs/03-environments.md).
        # If CUDA is visible here, the CPU-only wheel index was bypassed.
        raise AssertionError(
            "torch.cuda.is_available() is True in the sim-cpu image; this image must be CPU-only"
        )

    a = torch.arange(6, dtype=torch.float32, device="cpu").reshape(2, 3)
    b = torch.ones((3, 2), dtype=torch.float32, device="cpu")
    result = a @ b
    expected = np.array([[3.0, 3.0], [12.0, 12.0]], dtype=np.float32)
    if not np.allclose(result.numpy(), expected):
        raise AssertionError(f"unexpected torch CPU matmul result: {result}")

    print(f"torch CPU smoke test OK: torch {torch.__version__}, matmul result {result.tolist()}")


def main() -> int:
    try:
        run_gym_smoke_test()
        run_torch_smoke_test()
    except Exception as exc:  # noqa: BLE001 - top-level smoke test, report and fail
        print(f"SMOKE TEST FAILED: {exc}", file=sys.stderr)
        return 1
    print("sim-cpu smoke test PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
