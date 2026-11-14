"""Analysis code for the Plug-and-Play v0.2 pre-registration (evaluation/prereg/README.md)."""

from racer_eval.curves import LapCurve, RunLaps, RunMetrics, lap_curve, run_metrics
from racer_eval.gates import (
    GateItem,
    GateResult,
    SimBatteryResult,
    SimRun,
    gp3_decision,
    gp4_decision,
    planner_car_acceptance,
    sim_battery_decision,
)
from racer_eval.interleave import run_order
from racer_eval.interventions import RunInterventions, count_interventions
from racer_eval.laps import LapRow, lap_table
from racer_eval.runs import Run, find_runs
from racer_eval.split import Split, SplitError, load_split, role_of
from racer_eval.stats import (
    Interval,
    block_bootstrap_ratio,
    macro_rmse,
    paired_t_interval,
    rmse,
    t_interval,
    wilson_interval,
)

__all__ = [
    "GateItem",
    "GateResult",
    "Interval",
    "LapCurve",
    "LapRow",
    "Run",
    "RunInterventions",
    "RunLaps",
    "RunMetrics",
    "SimBatteryResult",
    "SimRun",
    "Split",
    "SplitError",
    "block_bootstrap_ratio",
    "count_interventions",
    "find_runs",
    "gp3_decision",
    "gp4_decision",
    "lap_curve",
    "lap_table",
    "load_split",
    "macro_rmse",
    "paired_t_interval",
    "planner_car_acceptance",
    "rmse",
    "role_of",
    "run_metrics",
    "run_order",
    "sim_battery_decision",
    "t_interval",
    "wilson_interval",
]
