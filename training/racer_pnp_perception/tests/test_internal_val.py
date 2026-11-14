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
