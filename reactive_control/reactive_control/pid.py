"""
PID controller module.

This module implements a PID controller with tunable gains for Kp, Ki, and Kd.
"""


class PID:
    """
    PID controller with time-based integration and differentiation.
    """

    def __init__(self, K_p: float, K_i: float, K_d: float) -> None:
        """
        Initialize the PID controller.

        Args:
            K_p: Proportional gain (how strongly to react to current error).
            K_i: Integral gain (how strongly to react to accumulated error).
            K_d: Derivative gain (how strongly to react to change in error).

        Returns:
            None
        """
        self.K_p = K_p
        self.K_i = K_i
        self.K_d = K_d

        self.prev_err = 0
        self.int_acc = 0
        self.prev_time = None

    def pid_err(self, curr_err: float, current_time: float) -> float:
        """
        Calculate the PID control output for the current error.

        Combines the proportional, integral, and derivative terms, using the
        actual time between callbacks (dt) for integration and differentiation.

        Args:
            curr_err: Current error measurement (distance from desired position).
            current_time: Current timestamp in seconds from the ROS clock.

        Returns:
            Control output (steering angle correction).
        """
        # Compute the time step between callbacks
        if self.prev_time is None:
            # First iteration uses a placeholder time step
            t_step = 0.01
        else:
            t_step = current_time - self.prev_time
            if t_step <= 0.0:
                t_step = 1e-6

        # Proportional error
        p_err = self.K_p * curr_err

        # Integral (accumulated) error, clamped to avoid integral windup
        self.int_acc += curr_err * t_step
        self.int_acc = max(min(self.int_acc, 100.0), -100.0)
        i_err = self.K_i * self.int_acc

        # Derivative error
        d_err = self.K_d * (curr_err - self.prev_err) / t_step

        # Store state for the next iteration
        self.prev_err = curr_err
        self.prev_time = current_time

        return p_err + i_err + d_err

    def reset(self) -> None:
        """
        Reset internal state.

        Call after an emergency stop to prevent a control spike on resume.

        Returns:
            None
        """
        self.prev_err = 0
        self.int_acc = 0
        self.prev_time = None

    def set_gains(self, K_p: float, K_i: float, K_d: float) -> None:
        """
        Update the controller gains at runtime.

        Args:
            K_p: Proportional gain.
            K_i: Integral gain.
            K_d: Derivative gain.

        Returns:
            None
        """
        self.K_p = K_p
        self.K_i = K_i
        self.K_d = K_d
