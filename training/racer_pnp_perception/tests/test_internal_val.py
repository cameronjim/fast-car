"""Scoring on the internal validation labels (a simulated drive stands in for the bags)."""

from __future__ import annotations

import numpy as np
from racer_pnp_perception import internal_val, real
from racer_track_perception import core
from racer_track_perception.vehicle import LiveVehicle
from test_gp1 import random_net, sim_run  # noqa: F401  (a pytest fixture)


def test_validation_rows_score_every_label_with_both_backends(sim_run, live: LiveVehicle) -> None:  # noqa: F811
    _tmp, out, _split, _entry = sim_run
    everything = real.read_label_file(out / "simbag.npz")
    train = everything.take(everything.role == "train")
    assert 0 < len(train) < len(everything)
    perceivers = {
        core.BACKEND_GEOMETRIC: core.TrackPerceiver(core.BACKEND_GEOMETRIC, live),
        core.BACKEND_NET: core.TrackPerceiver(
            core.BACKEND_NET, live, net=random_net(), model_id="random"
        ),
    }
    rows = internal_val.validation_rows(train, out, perceivers, live)
    assert len(rows) == len(train)
    assert all(r.label is not None and r.path is not None for r in rows)
    for r in rows:
        assert set(r.e_c) == set(perceivers) and set(r.deficit) == set(perceivers)
    summary = internal_val.summarise_rows(rows, list(perceivers))
    geo = summary[core.BACKEND_GEOMETRIC]
    assert geo["scans"] == len(train) and geo["e_c_m"] < 0.1  # the tracer against its own labels
    assert np.isfinite(geo["free_deficit_p90_m"]) and geo["free_c95_scans"] == len(train)
    assert set(summary[core.BACKEND_NET]) == set(geo)


def test_common_station_ec_uses_only_the_stations_every_net_keeps() -> None:
    from racer_pnp_perception import gp1

    def row(errs: dict[str, list[float]]) -> gp1.ScanRow:
        r = gp1.ScanRow("b", 0, "s", "seg", 0.0, 0.0, 0.0, "cw", None, np.zeros((16, 5)), 16)
        r.station_err = {k: np.array(v) for k, v in errs.items()}
        return r

    rows = [row({"a": [0.1, 0.1, 0.9], "b": [0.2, 0.2]}), row({"a": [0.3], "b": [0.1, 0.5]})]
    got = gp1.common_station_ec(rows, ["a", "b"])
    # Scan 1: 2 common stations (a 0.1, b 0.2); scan 2: 1 (a 0.3, b 0.1).
    assert np.isclose(got["a"], 0.2) and np.isclose(got["b"], 0.15)
