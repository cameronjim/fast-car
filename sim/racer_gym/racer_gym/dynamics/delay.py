"""Fixed transport lag (command-to-torque delay) as an N-step FIFO buffer.

Roadmap S.1 requirement 4 (claude-docs/07-sim-and-sysid.md): "Measured command-to-torque
delay as a fixed transport lag." The delay is expressed in whole simulation steps
(`delay_steps = round(delay_s / dt_s)`, computed by racer_gym/params.py from
`vehicle_params.yaml`'s `actuation.command_to_torque_delay_s`), and implemented here as a
fixed-size FIFO: the command applied at step `t` is exactly the command pushed at step
`t - delay_steps` (see tests/test_delay.py::test_delay_applies_exactly_n_steps_late). Before
enough commands have been pushed to fill the buffer, `initial_value` is returned (matching
`reset()` semantics: the vehicle is not assumed to have received phantom pre-episode
commands).
"""

from __future__ import annotations

from collections import deque


class TransportDelay:
    def __init__(self, delay_steps: int, initial_value: float = 0.0) -> None:
        if delay_steps < 0:
            raise ValueError("delay_steps must be >= 0")
        self.delay_steps = delay_steps
        self._initial_value = initial_value
        self._buffer: deque[float] = deque(
            [initial_value] * delay_steps, maxlen=delay_steps if delay_steps > 0 else None
        )

    def reset(self, initial_value: float | None = None) -> None:
        if initial_value is not None:
            self._initial_value = initial_value
        self._buffer.clear()
        self._buffer.extend([self._initial_value] * self.delay_steps)

    def step(self, command: float) -> float:
        """Push `command`, return the command issued `delay_steps` steps ago (FIFO)."""
        if self.delay_steps == 0:
            return command
        delayed = self._buffer.popleft()
        self._buffer.append(command)
        return delayed
