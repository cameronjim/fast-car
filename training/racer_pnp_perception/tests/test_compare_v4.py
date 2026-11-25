"""L1: the trackperc-v4 ship rule (compare_v4.decide) as written in
evaluation/prereg/w2-trackperc-v4-2026-11-25.yaml: the v3 rule with v4 in v3's place, plus at least
300 H-session labelled scans."""

from __future__ import annotations

import math

import pytest
from racer_pnp_perception import compare_v3 as cv
from racer_pnp_perception import compare_v4 as c4
from racer_pnp_perception.gp1 import LabelFree, Labelled


def _lab(ec: float, v: float, c95: float = 0.9, scans: int = 500) -> Labelled:
    return Labelled(scans=scans, e_c_m=ec, e_far_m=ec, v=v, c95=c95)


def _free(intrusion: float) -> LabelFree:
    return LabelFree(500, intrusion, 0.04, 0.01, 0.9, 500)


def _case(**over):  # type: ignore[no-untyped-def]
    lay = {c4.V2: _lab(0.10, 15.0), c4.V4: _lab(0.09, 15.0)}
    ses = {c4.V2: _lab(0.06, 15.5), c4.V4: _lab(0.06, 15.4)}
    free = {c4.V2: _free(0.003), c4.V4: _free(0.004)}
    lat = {c4.V2: 1.0, c4.V4: 1.1}
    lay.update(over.get("lay", {}))
    ses.update(over.get("ses", {}))
    free.update(over.get("free", {}))
    lat.update(over.get("lat", {}))
    return c4.decide(
        {cv.SET_LAYOUT: lay, cv.SET_SESSION: ses},
        {cv.SET_LAYOUT: free, cv.SET_SESSION: free},
        lat,
    )


def test_a_candidate_better_on_the_layout_ships() -> None:
    ship, items = _case()
    assert ship and all(i["passed"] for i in items)
    assert [i["item"] for i in items] == [
        "H-layout E_c(v4) <= E_c(v2)",
        "H-layout V(v4) >= V(v2) - 0.25",
        "H-layout C95(v4) >= C95(v2) - 0.02",
        "H-layout intrusion(v4) <= max(0.02, intrusion(v2) + 0.005)",
        "H-session labelled scans >= 300",
        "H-session E_c(v4) <= 1.05 E_c(v2)",
        "H-session V(v4) >= V(v2) - 0.25",
        "latency p99(v4) <= 1.2 p99(v2)",
    ]


def test_the_v3_margins_hold_for_v4() -> None:
    assert not _case(lay={c4.V4: _lab(0.11, 15.0)})[0]
    assert _case(lay={c4.V4: _lab(0.09, 14.76)})[0]
    assert not _case(lay={c4.V4: _lab(0.09, 14.74)})[0]
    assert not _case(lay={c4.V4: _lab(0.09, 15.0, c95=0.87)})[0]
    assert not _case(free={c4.V4: _free(0.021)})[0]
    assert _case(ses={c4.V4: _lab(0.063, 15.4)})[0]
    assert not _case(ses={c4.V4: _lab(0.064, 15.4)})[0]
    assert not _case(lat={c4.V4: 1.21})[0]


def test_too_few_session_labels_fails() -> None:
    assert _case(ses={c4.V4: _lab(0.06, 15.4, scans=300)})[0]
    ship, items = _case(ses={c4.V4: _lab(0.06, 15.4, scans=299)})
    assert not ship
    assert [i["item"] for i in items if not i["passed"]] == ["H-session labelled scans >= 300"]


def test_a_nan_number_fails_its_item() -> None:
    assert not _case(lay={c4.V4: _lab(math.nan, 15.0)})[0]


def test_refs() -> None:
    assert c4.parse_refs(["v3=/a"]) == {"v3": __import__("pathlib").Path("/a")}
    for bad in ("v3", "=/a", "v2=/a", "v4=/a"):
        with pytest.raises(SystemExit):
            c4.parse_refs([bad])
