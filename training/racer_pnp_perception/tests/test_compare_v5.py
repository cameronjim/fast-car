"""L1: the trackperc-v5 ship rule (compare_v5.decide) as written in
evaluation/prereg/w2-trackperc-v5-2026-11-25.yaml: the v4 rule unchanged, with v5 in v4's place."""

from __future__ import annotations

import math

import pytest
from racer_pnp_perception import compare_v3 as cv
from racer_pnp_perception import compare_v5 as c5
from racer_pnp_perception.gp1 import LabelFree, Labelled


def _lab(ec: float, v: float, c95: float = 0.9, scans: int = 500) -> Labelled:
    return Labelled(scans=scans, e_c_m=ec, e_far_m=ec, v=v, c95=c95)


def _free(intrusion: float) -> LabelFree:
    return LabelFree(500, intrusion, 0.04, 0.01, 0.9, 500)


def _case(**over):  # type: ignore[no-untyped-def]
    lay = {c5.V2: _lab(0.10, 15.0), c5.V5: _lab(0.09, 15.0)}
    ses = {c5.V2: _lab(0.06, 15.5), c5.V5: _lab(0.06, 15.4)}
    free = {c5.V2: _free(0.003), c5.V5: _free(0.004)}
    lat = {c5.V2: 1.0, c5.V5: 1.1}
    lay.update(over.get("lay", {}))
    ses.update(over.get("ses", {}))
    free.update(over.get("free", {}))
    lat.update(over.get("lat", {}))
    return c5.decide(
        {cv.SET_LAYOUT: lay, cv.SET_SESSION: ses},
        {cv.SET_LAYOUT: free, cv.SET_SESSION: free},
        lat,
    )


def test_a_candidate_better_on_the_layout_ships() -> None:
    ship, items = _case()
    assert ship and all(i["passed"] for i in items)
    assert [i["item"] for i in items] == [
        "H-layout E_c(v5) <= E_c(v2)",
        "H-layout V(v5) >= V(v2) - 0.25",
        "H-layout C95(v5) >= C95(v2) - 0.02",
        "H-layout intrusion(v5) <= max(0.02, intrusion(v2) + 0.005)",
        "H-session labelled scans >= 300",
        "H-session E_c(v5) <= 1.05 E_c(v2)",
        "H-session V(v5) >= V(v2) - 0.25",
        "latency p99(v5) <= 1.2 p99(v2)",
    ]


def test_the_v4_margins_hold_for_v5() -> None:
    assert not _case(lay={c5.V5: _lab(0.11, 15.0)})[0]
    assert _case(lay={c5.V5: _lab(0.09, 14.76)})[0]
    assert not _case(lay={c5.V5: _lab(0.09, 14.74)})[0]
    assert not _case(lay={c5.V5: _lab(0.09, 15.0, c95=0.87)})[0]
    assert not _case(free={c5.V5: _free(0.021)})[0]
    assert _case(ses={c5.V5: _lab(0.063, 15.4)})[0]
    assert not _case(ses={c5.V5: _lab(0.064, 15.4)})[0]
    assert not _case(lat={c5.V5: 1.21})[0]


def test_too_few_session_labels_fails() -> None:
    assert _case(ses={c5.V5: _lab(0.06, 15.4, scans=300)})[0]
    ship, items = _case(ses={c5.V5: _lab(0.06, 15.4, scans=299)})
    assert not ship
    assert [i["item"] for i in items if not i["passed"]] == ["H-session labelled scans >= 300"]


def test_a_nan_number_fails_its_item() -> None:
    assert not _case(lay={c5.V5: _lab(math.nan, 15.0)})[0]


def test_refs() -> None:
    assert c5.parse_refs(["v4=/a"]) == {"v4": __import__("pathlib").Path("/a")}
    for bad in ("v3", "=/a", "v2=/a", "v5=/a"):
        with pytest.raises(SystemExit):
            c5.parse_refs([bad])
