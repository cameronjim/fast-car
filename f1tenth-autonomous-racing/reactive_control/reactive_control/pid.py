"""pid controller shared by the reactive controllers."""

FIRST_STEP_SEC = 0.01
MIN_STEP_SEC = 1e-6
INTEGRAL_LIMIT = 100.0


class PID:
    """pid controller integrating and differentiating against the wall clock."""

    def __init__(self, K_p: float, K_i: float, K_d: float) -> None:
        self.K_p = K_p
        self.K_i = K_i
        self.K_d = K_d

        self.prev_err = 0
        self.int_acc = 0
        self.prev_time = None

    def pid_err(self, curr_err: float, current_time: float) -> float:
        """control output for the current error, using the real time since the last call."""
        if self.prev_time is None:
            t_step = FIRST_STEP_SEC
        else:
            t_step = current_time - self.prev_time
            if t_step <= 0.0:
                t_step = MIN_STEP_SEC

        p_err = self.K_p * curr_err

        # clamped to stop integral windup while the car sits latched at a stop
        self.int_acc += curr_err * t_step
        self.int_acc = max(min(self.int_acc, INTEGRAL_LIMIT), -INTEGRAL_LIMIT)
        i_err = self.K_i * self.int_acc

        d_err = self.K_d * (curr_err - self.prev_err) / t_step

        self.prev_err = curr_err
        self.prev_time = current_time

        return p_err + i_err + d_err

    def reset(self) -> None:
        """clear state after an emergency stop so resuming does not spike the output."""
        self.prev_err = 0
        self.int_acc = 0
        self.prev_time = None

    def set_gains(self, K_p: float, K_i: float, K_d: float) -> None:
        self.K_p = K_p
        self.K_i = K_i
        self.K_d = K_d
