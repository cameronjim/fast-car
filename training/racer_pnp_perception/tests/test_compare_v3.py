"""L1: the trackperc-v3 ship rule (compare_v3.decide) as written in
evaluation/prereg/w2-trackperc-v3-2026-11-23.yaml."""

from __future__ import annotations

import math

from racer_pnp_perception import compare_v3 as cv
from racer_pnp_perception.gp1 import LabelFree, Labelled


def _lab(ec: float, v: float, c95: float = 0.9) -> Labelled:
    return Labelled(scans=500, e_c_m=ec, e_far_m=ec, v=v, c95=c95)


def _free(intrusion: float) -> LabelFree:
    return LabelFree(500, intrusion, 0.04, 0.01, 0.9, 500)


def _case(**over):  # type: ignore[no-untyped-def]
    lay = {cv.V2: _lab(0.10, 15.0), cv.V3: _lab(0.09, 15.0)}
    ses = {cv.V2: _lab(0.06, 15.5), cv.V3: _lab(0.06, 15.4)}
    free = {cv.V2: _free(0.003), cv.V3: _free(0.004)}
    lat = {cv.V2: 1.0, cv.V3: 1.1}
    lay.update(over.get("lay", {}))
    ses.update(over.get("ses", {}))
    free.update(over.get("free", {}))
    lat.update(over.get("lat", {}))
    labelled = {cv.SET_LAYOUT: lay, cv.SET_SESSION: ses}
    label_free = {cv.SET_LAYOUT: free, cv.SET_SESSION: free}
    return cv.decide(labelled, label_free, lat)


def test_a_candidate_better_on_the_layout_ships() -> None:
    ship, items = _case()
    assert ship and all(i["passed"] for i in items)


def test_worse_layout_centre_does_not_ship() -> None:
    ship, items = _case(lay={cv.V3: _lab(0.11, 15.0)})
    assert not ship
    assert [i["item"] for i in items if not i["passed"]] == ["H-layout E_c(v3) <= E_c(v2)"]


def test_layout_margins() -> None:
    assert _case(lay={cv.V3: _lab(0.09, 14.76)})[0]
    assert not _case(lay={cv.V3: _lab(0.09, 14.74)})[0]
    assert not _case(lay={cv.V3: _lab(0.09, 15.0, c95=0.87)})[0]
    assert _case(free={cv.V3: _free(0.019)})[0]  # under the 0.02 floor
    assert not _case(free={cv.V3: _free(0.021)})[0]


def test_session_and_latency_margins() -> None:
    assert _case(ses={cv.V3: _lab(0.063, 15.4)})[0]
    assert not _case(ses={cv.V3: _lab(0.064, 15.4)})[0]
    assert not _case(ses={cv.V3: _lab(0.06, 15.2)})[0]
    assert not _case(lat={cv.V3: 1.21})[0]


def test_a_nan_number_fails_its_item() -> None:
    ship, _items = _case(lay={cv.V3: _lab(math.nan, 15.0)})
    assert not ship
