"""The variant layout form (pre-registration v0.4 section 6.2, runbook protocol).

A variant is the donut plus at least two solid objects, each against a wall, that change the lane
by at least 0.15 m at two or more places, with at least 0.80 m of lane left at each object
(the narrowest lane the car has driven), at positions not used in an earlier session, and not
driven before the protocol's first run. The form records the evidence; this module checks it.

A form is valid only through `check_form`, which returns a `CheckedForm`. `gp4v.decide_gp4v`
accepts nothing else, so runs without a documented variant cannot be GP4-V runs.

Constants not given by a number in the pre-registration are marked "protocol constant": they are
this module's reading of the sentence quoted next to them and are listed in the PR comment.
"""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from racer_eval.interleave import run_order

#: v0.4 6.2 / design 16.7: "at least two solid objects".
MIN_OBJECTS = 2
#: v0.4 6.2: "the lane changes by at least 0.15 m at two or more places".
MIN_LANE_CHANGE_M = 0.15
MIN_PLACES_CHANGING_LANE = 2
#: Brief 41 section 2: "the remaining lane at each object at least 0.80 m (the narrowest lane the
#: car has driven)".
MIN_REMAINING_LANE_M = 0.80
#: Brief 41 section 2: the object's height is compared "with the LiDAR height from the vehicle
#: binding plus a margin". Protocol constant: the stated tolerance of the mount_z_m measurement in
#: config/vehicle_params.yaml (+/- 0.02 m).
LIDAR_HEIGHT_MARGIN_M = 0.02
#: Protocol constant: two objects on the same wall closer than this along the outer wall are the
#: same position (the length of a shoebox-sized object, not a measured quantity).
POSITION_REUSE_TOLERANCE_M = 0.30
WALLS = ("outer", "inner")


class FormInvalid(ValueError):
    """The form is not valid; `problems` lists every reason."""

    def __init__(self, problems: Sequence[str]) -> None:
        self.problems = tuple(problems)
        super().__init__("variant form invalid: " + "; ".join(self.problems))


@dataclass(frozen=True)
class VariantObject:
    object_id: str
    height_m: float
    size_m: tuple[float, float]
    wall: str
    #: Distance along the OUTER wall from the start line, for an object on either wall.
    distance_along_outer_wall_m: float
    protrusion_m: float
    #: Measured lane width left at the object.
    lane_width_m: float
    against_wall: bool
    heavy: bool
    reflective_or_glass: bool


@dataclass(frozen=True)
class VariantForm:
    variant_id: str
    photo: str
    #: Seed of racer_eval.interleave for GP4-V's run order, written before the first run.
    seed: int
    objects: tuple[VariantObject, ...]
    driven_before_first_protocol_run: bool


@dataclass(frozen=True)
class CheckedForm:
    form: VariantForm
    lidar_mount_z_m: float
    margin_m: float


def _num(x: Any) -> float | None:
    if isinstance(x, bool) or not isinstance(x, int | float):
        return None
    return float(x) if math.isfinite(x) else None


_OBJECT_KEYS = (
    "id",
    "height_m",
    "size_m",
    "wall",
    "distance_along_outer_wall_m",
    "protrusion_m",
    "lane_width_m",
    "against_wall",
    "heavy",
    "reflective_or_glass",
)


def _parse_object(i: int, o: Mapping[str, Any], problems: list[str]) -> VariantObject | None:
    absent = [k for k in _OBJECT_KEYS if k not in o]
    if absent:
        problems.append(f"object {i}: missing {absent}")
        return None
    size = o["size_m"]
    height = _num(o["height_m"])
    dist = _num(o["distance_along_outer_wall_m"])
    prot = _num(o["protrusion_m"])
    lane = _num(o["lane_width_m"])
    flags = [o[k] for k in ("against_wall", "heavy", "reflective_or_glass")]
    bad_size = not (isinstance(size, list | tuple) and len(size) == 2) or any(
        _num(s) is None for s in size
    )
    if (
        bad_size
        or None in (height, dist, prot, lane)
        or not all(isinstance(f, bool) for f in flags)
    ):
        problems.append(f"object {i}: a field has the wrong type")
        return None
    assert height is not None and dist is not None and prot is not None and lane is not None
    return VariantObject(
        object_id=str(o["id"]),
        height_m=height,
        size_m=(float(size[0]), float(size[1])),
        wall=str(o["wall"]),
        distance_along_outer_wall_m=dist,
        protrusion_m=prot,
        lane_width_m=lane,
        against_wall=flags[0],
        heavy=flags[1],
        reflective_or_glass=flags[2],
    )


def parse_form(doc: Mapping[str, Any]) -> VariantForm:
    """Structure only (every field present and typed); the rules are `check_form`."""
    problems = [
        f"missing {key!r}"
        for key in ("variant_id", "photo", "seed", "objects", "driven_before_first_protocol_run")
        if key not in doc
    ]
    raw_objects = doc.get("objects", [])
    if not isinstance(raw_objects, list):
        problems.append("`objects` must be a list")
        raw_objects = []
    objects: list[VariantObject] = []
    for i, o in enumerate(raw_objects):
        if not isinstance(o, Mapping):
            problems.append(f"object {i}: not a mapping")
            continue
        parsed = _parse_object(i, o, problems)
        if parsed is not None:
            objects.append(parsed)
    seed = doc.get("seed")
    if "seed" in doc and (isinstance(seed, bool) or not isinstance(seed, int)):
        problems.append("`seed` must be an integer")
    driven = doc.get("driven_before_first_protocol_run")
    if "driven_before_first_protocol_run" in doc and not isinstance(driven, bool):
        problems.append("`driven_before_first_protocol_run` must be true or false")
    if problems:
        raise FormInvalid(problems)
    return VariantForm(
        variant_id=str(doc["variant_id"]),
        photo=str(doc["photo"]),
        seed=int(seed),  # type: ignore[arg-type]
        objects=tuple(objects),
        driven_before_first_protocol_run=bool(driven),
    )


def positions_of(form: VariantForm) -> list[tuple[str, float]]:
    return [(o.wall, o.distance_along_outer_wall_m) for o in form.objects]


def _same_position(a: tuple[str, float], b: tuple[str, float]) -> bool:
    return a[0] == b[0] and abs(a[1] - b[1]) < POSITION_REUSE_TOLERANCE_M


def _object_problems(
    o: VariantObject, min_height: float, lidar_z: float, margin: float
) -> list[str]:
    tag = f"object {o.object_id}"
    out: list[str] = []
    if o.wall not in WALLS:
        out.append(f"{tag}: wall must be one of {list(WALLS)}")
    if not o.against_wall:
        out.append(f"{tag}: not against a wall (no free-standing object in v0.4)")
    if not o.heavy:
        out.append(f"{tag}: not heavy enough to stay put when touched")
    if o.reflective_or_glass:
        out.append(f"{tag}: reflective or glass")
    if o.height_m < min_height:
        out.append(
            f"{tag}: height {o.height_m:.3f} m is under the LiDAR plane "
            f"{lidar_z:.3f} m plus the {margin:.3f} m margin"
        )
    if min(o.size_m) <= 0.0:
        out.append(f"{tag}: size must be positive")
    if o.protrusion_m <= 0.0:
        out.append(f"{tag}: protrusion must be positive")
    if o.lane_width_m < MIN_REMAINING_LANE_M:
        out.append(
            f"{tag}: remaining lane {o.lane_width_m:.3f} m is under {MIN_REMAINING_LANE_M} m"
        )
    return out


def problems_of(
    form: VariantForm,
    lidar_mount_z_m: float,
    earlier_positions: Sequence[tuple[str, float]] = (),
    margin_m: float = LIDAR_HEIGHT_MARGIN_M,
) -> list[str]:
    """Every rule of the protocol that the form breaks (empty when none)."""
    problems: list[str] = []
    if not form.photo.strip():
        problems.append("no photo file name")
    if form.driven_before_first_protocol_run:
        problems.append("the layout was driven before the protocol's first run")
    if len(form.objects) < MIN_OBJECTS:
        problems.append(f"{len(form.objects)} object(s), at least {MIN_OBJECTS} are needed")
    ids = [o.object_id for o in form.objects]
    if len(set(ids)) != len(ids):
        problems.append("object ids are not unique")
    min_height = lidar_mount_z_m + margin_m
    for o in form.objects:
        problems += _object_problems(o, min_height, lidar_mount_z_m, margin_m)
    changing = sum(o.protrusion_m >= MIN_LANE_CHANGE_M for o in form.objects)
    if changing < MIN_PLACES_CHANGING_LANE:
        problems.append(
            f"the lane changes by at least {MIN_LANE_CHANGE_M} m at {changing} place(s), "
            f"at least {MIN_PLACES_CHANGING_LANE} are needed"
        )
    own = positions_of(form)
    for i, p in enumerate(own):
        problems += [
            f"two objects at the same position {p}" for q in own[i + 1 :] if _same_position(p, q)
        ]
        problems += [
            f"position {p} was used before ({e})" for e in earlier_positions if _same_position(p, e)
        ]
    return problems


def check_form(
    form: VariantForm,
    lidar_mount_z_m: float,
    earlier_positions: Sequence[tuple[str, float]] = (),
    margin_m: float = LIDAR_HEIGHT_MARGIN_M,
) -> CheckedForm:
    """The form as a `CheckedForm`, or FormInvalid with every problem."""
    problems = problems_of(form, lidar_mount_z_m, earlier_positions, margin_m)
    if problems:
        raise FormInvalid(problems)
    return CheckedForm(form, float(lidar_mount_z_m), float(margin_m))


def load_form(path: Path) -> VariantForm:
    doc = yaml.safe_load(Path(path).read_text())
    if not isinstance(doc, Mapping):
        raise FormInvalid(["the form file is not a mapping"])
    return parse_form(doc)


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def receipt_path(form_path: Path) -> Path:
    p = Path(form_path)
    return p.with_name(p.stem + ".checked.json")


def write_receipt(
    form_path: Path,
    checked: CheckedForm,
    earlier_positions: Sequence[tuple[str, float]],
    lidar_source: str,
) -> Path:
    """Write the receipt next to the form: the form's SHA-256 and the numbers the check used."""
    out = receipt_path(form_path)
    out.write_text(
        json.dumps(
            {
                "schema": "pnp-variant-form-receipt/1",
                "form": Path(form_path).name,
                "form_sha256": _sha256(form_path),
                "variant_id": checked.form.variant_id,
                "lidar_mount_z_m": checked.lidar_mount_z_m,
                "lidar_source": lidar_source,
                "margin_m": checked.margin_m,
                "earlier_positions": [list(p) for p in earlier_positions],
            },
            indent=2,
            sort_keys=True,
        )
        + "\n"
    )
    return out


def load_checked(form_path: Path) -> CheckedForm:
    """The form re-checked with the numbers of its receipt. Refuses a form with no receipt, or one
    edited after the receipt was written."""
    rp = receipt_path(form_path)
    if not rp.is_file():
        raise FormInvalid([f"no receipt {rp.name}: run the variant-form check first"])
    receipt = json.loads(rp.read_text())
    if receipt.get("form_sha256") != _sha256(form_path):
        raise FormInvalid(["the form was edited after its receipt was written"])
    return check_form(
        load_form(form_path),
        float(receipt["lidar_mount_z_m"]),
        [(str(w), float(d)) for w, d in receipt["earlier_positions"]],
        float(receipt["margin_m"]),
    )


def interleaved_order(
    checked: CheckedForm, conditions: Sequence[str], runs_per_condition: int | Mapping[str, int]
) -> list[str]:
    """GP4-V's run order: racer_eval.interleave with the seed written into the form."""
    return run_order(conditions, runs_per_condition, checked.form.seed)
