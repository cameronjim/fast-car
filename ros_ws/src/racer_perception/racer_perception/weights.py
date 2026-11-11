"""Pinned YOLO weights: where they come from and how they are checked (roadmap 5.1). No ROS.

WEIGHTS ARE NEVER COMMITTED. Each known file is pinned by URL AND sha256 here (and in
docker/car/perception-weights.txt, which the car image's PERCEPTION_WEIGHTS=bake step reads;
test_weights.py fails if the two tables differ). `locate_weights` finds a file in the model cache
or the image's bake directory, or downloads it from the pinned URL, and in every case refuses a
file whose sha256 does not match (CLAUDE.md invariant 3: refuse on mismatch, never a warning).

A bare name that is NOT in the table is not downloaded at all (the old behaviour handed any name
to ultralytics, which fetches unpinned files); it must already exist as a file. A path with a
directory part is used as given and, if its file name is a known one, still checked against the
pin; a custom model has no pin and is taken on trust, because the operator named it.
"""

from __future__ import annotations

import hashlib
import os
import urllib.request
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

#: Environment variable naming the directory the car image bakes weights into.
WEIGHTS_DIR_ENV = "RACER_WEIGHTS_DIR"

_ASSETS = "https://github.com/ultralytics/assets/releases/download/v8.3.0"


@dataclass(frozen=True)
class PinnedWeights:
    name: str
    sha256: str
    url: str


KNOWN_WEIGHTS: dict[str, PinnedWeights] = {
    w.name: w
    for w in (
        PinnedWeights(
            "yolov8n.pt",
            "f59b3d833e2ff32e194b5bb8e08d211dc7c5bdf144b90d2c8412c47ccfc83b36",
            f"{_ASSETS}/yolov8n.pt",
        ),
        PinnedWeights(
            "yolov8n-pose.pt",
            "c6fa93dd1ee4a2c18c900a45c1d864a1c6f7aba75d84f91648a30b7fb641d212",
            f"{_ASSETS}/yolov8n-pose.pt",
        ),
    )
}

#: Default model per detector mode.
DEFAULT_MODEL = {"detect": "yolov8n.pt", "pose": "yolov8n-pose.pt"}


class WeightsError(RuntimeError):
    """The weights are missing, cannot be fetched, or do not match their pinned checksum."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify(path: Path, pin: PinnedWeights) -> None:
    actual = sha256_file(path)
    if actual != pin.sha256:
        raise WeightsError(
            f"{path} has sha256 {actual}, expected {pin.sha256} ({pin.name}). Refusing to load "
            "weights that do not match their pin: delete the file to fetch it again."
        )


def parse_manifest(text: str) -> dict[str, PinnedWeights]:
    """Parse docker/car/perception-weights.txt: `<name> <sha256> <url>` per line, # comments."""
    out: dict[str, PinnedWeights] = {}
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        name, sha, url = line.split()
        out[name] = PinnedWeights(name, sha, url)
    return out


def _download(url: str, dest: Path) -> None:
    tmp = dest.with_suffix(dest.suffix + ".part")
    request = urllib.request.Request(url, headers={"User-Agent": "racer-perception"})
    with urllib.request.urlopen(request, timeout=60) as response, tmp.open("wb") as out:
        while chunk := response.read(1 << 20):
            out.write(chunk)
    os.replace(tmp, dest)


def locate_weights(
    model: str,
    cache_dir: str,
    baked_dir: str | None = None,
    downloader: Callable[[str, Path], None] = _download,
) -> Path:
    """Return a verified path to the weights for `model`.

    Search order for a bare known name: `cache_dir`, then `baked_dir` (default from the
    RACER_WEIGHTS_DIR environment variable), then download into `cache_dir` from the pinned
    URL. Raises WeightsError for an unknown bare name that is not a file, a failed download,
    or any checksum mismatch.
    """
    if not model:
        raise ValueError("model must not be empty")
    given = Path(os.path.expanduser(model))
    pin = KNOWN_WEIGHTS.get(given.name)
    if given.is_absolute() or given.parent != Path("."):
        if not given.is_file():
            raise WeightsError(f"model file {given} does not exist")
        if pin is not None:
            verify(given, pin)
        return given

    cache = Path(os.path.expanduser(cache_dir))
    candidates = [cache / given.name]
    baked = baked_dir if baked_dir is not None else os.environ.get(WEIGHTS_DIR_ENV)
    if baked:
        candidates.append(Path(baked) / given.name)
    for path in candidates:
        if path.is_file():
            if pin is not None:
                verify(path, pin)
            return path
    if pin is None:
        raise WeightsError(
            f"model {model!r} is not a known pinned weights file ({sorted(KNOWN_WEIGHTS)}) and "
            f"was not found in {[str(c.parent) for c in candidates]}. Put the file there, or give "
            "model:= a full path."
        )
    dest = cache / pin.name
    try:
        cache.mkdir(parents=True, exist_ok=True)
        downloader(pin.url, dest)
    except Exception as exc:  # network, permissions
        raise WeightsError(
            f"{pin.name} is missing and could not be downloaded from {pin.url} ({exc}). Copy the "
            f"file into {cache} by hand (sha256 {pin.sha256})."
        ) from exc
    try:
        verify(dest, pin)
    except WeightsError:
        dest.unlink(missing_ok=True)
        raise
    return dest
