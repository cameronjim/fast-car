"""The YOLO detector behind detector_node (roadmap 5.1), isolated so ROS code never imports it.

`ultralytics` (and the torch it pulls in) exist only in a car image built with
INSTALL_PERCEPTION=required (docker/car/README.md "Perception"). Every import of them is lazy
and inside `load_yolo_backend`, so this module, and detector_node.py with a fake backend,
import and test fine without them (CI's ros-dev image has neither). When they are missing,
`load_yolo_backend` raises DetectorUnavailable with an actionable message and detector_node
logs ONE error and exits non-zero.

Backend contract (what detector_node needs, and what tests fake):

    infer(jpeg: bytes) -> list[RawDetection]     class name, score, pixel box (centre, size)
    annotate(jpeg: bytes, dets) -> bytes         the JPEG with boxes drawn, for the debug image

WEIGHTS ARE NEVER COMMITTED. `resolve_model_path` maps a bare model name (yolov8n.pt) into
`cache_dir`; if the file is not there ultralytics downloads it on first run (the car-camera
container needs internet once, or the file copied into cache_dir by hand).
"""

from __future__ import annotations

import os
import re
from collections.abc import Sequence
from pathlib import Path
from typing import Protocol

from racer_perception.fusion import RawDetection

#: detector_node's exit code when ultralytics/torch are missing or the device is unusable.
EXIT_DETECTOR_UNAVAILABLE = 3

DEFAULT_CACHE_DIR = "~/.cache/racer_perception"

_DEVICE_RE = re.compile(r"^(cpu|cuda(:\d+)?)$")

#: Told to anyone whose ultralytics import fails.
MISSING_MESSAGE = (
    "the YOLO detector needs the `ultralytics` package and PyTorch, which this environment "
    "does not have. On the car, build the image with --build-arg INSTALL_TORCH=required "
    "--build-arg TORCH_WHEEL_URL=<url> --build-arg INSTALL_PERCEPTION=required "
    "--build-arg TORCHVISION_WHEEL_URL=<url> (docker/car/README.md, Perception) and run "
    "perception.launch.py in the car-camera container. detector_node cannot run in the "
    "default INSTALL_TORCH=skip image."
)


class DetectorUnavailable(RuntimeError):
    """The detector cannot be built here: missing package, bad device, no weights."""


class DetectorBackend(Protocol):
    def infer(self, jpeg: bytes) -> list[RawDetection]: ...

    def annotate(self, jpeg: bytes, detections: Sequence[RawDetection]) -> bytes: ...


def validate_device(device: str) -> str:
    """Accept cpu, cuda or cuda:N. Anything else (mps, a typo) is refused, not guessed."""
    if not _DEVICE_RE.match(device):
        raise ValueError(f"device must be 'cpu', 'cuda' or 'cuda:N', got {device!r}")
    return device


def resolve_model_path(model: str, cache_dir: str) -> Path:
    """A bare name lives in cache_dir; anything with a directory part is used as given."""
    if not model:
        raise ValueError("model must not be empty")
    path = Path(os.path.expanduser(model))
    if path.is_absolute() or path.parent != Path("."):
        return path
    return Path(os.path.expanduser(cache_dir)) / path


class YoloBackend:
    def __init__(self, yolo, cv2, np, device: str, score_threshold: float) -> None:
        self._yolo = yolo
        self._cv2 = cv2
        self._np = np
        self._device = device
        self._conf = score_threshold

    def _decode(self, jpeg: bytes):
        image = self._cv2.imdecode(self._np.frombuffer(jpeg, dtype=self._np.uint8), 1)
        if image is None:
            raise ValueError("could not decode the JPEG")
        return image

    def infer(self, jpeg: bytes) -> list[RawDetection]:
        image = self._decode(jpeg)
        result = self._yolo.predict(image, device=self._device, conf=self._conf, verbose=False)[0]
        out: list[RawDetection] = []
        for box in result.boxes:
            cx, cy, w, h = (float(v) for v in box.xywh[0].tolist())
            out.append(
                RawDetection(
                    class_name=str(result.names[int(box.cls[0])]),
                    score=float(box.conf[0]),
                    cx_px=cx,
                    cy_px=cy,
                    width_px=w,
                    height_px=h,
                )
            )
        return out

    def annotate(self, jpeg: bytes, detections: Sequence[RawDetection]) -> bytes:
        cv2 = self._cv2
        image = self._decode(jpeg)
        for d in detections:
            x0, y0 = int(d.cx_px - d.width_px / 2), int(d.cy_px - d.height_px / 2)
            x1, y1 = int(d.cx_px + d.width_px / 2), int(d.cy_px + d.height_px / 2)
            cv2.rectangle(image, (x0, y0), (x1, y1), (0, 255, 0), 2)
            cv2.putText(
                image,
                f"{d.class_name} {d.score:.2f}",
                (x0, max(y0 - 6, 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
            )
        ok, buf = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            raise ValueError("could not encode the debug JPEG")
        return buf.tobytes()


def load_yolo_backend(
    model: str, device: str, cache_dir: str, score_threshold: float
) -> YoloBackend:
    """Import ultralytics lazily and build the backend; raise DetectorUnavailable on any gap."""
    try:
        validate_device(device)
    except ValueError as exc:
        raise DetectorUnavailable(str(exc)) from exc
    try:
        import cv2
        import numpy as np
        import torch
        from ultralytics import YOLO
    except ImportError as exc:
        raise DetectorUnavailable(f"{MISSING_MESSAGE} (import failed: {exc})") from exc

    if device.startswith("cuda") and not torch.cuda.is_available():
        raise DetectorUnavailable(
            f"device={device} was requested but torch.cuda.is_available() is False. This "
            "container needs --runtime nvidia and a CUDA-enabled torch wheel. Refusing to fall "
            "back to the CPU silently: pass device:=cpu if that is what you want."
        )

    path = resolve_model_path(model, cache_dir)
    if not path.is_file():
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            from ultralytics.utils.downloads import attempt_download_asset

            attempt_download_asset(str(path))
        except Exception as exc:  # network, permissions, unknown asset name
            raise DetectorUnavailable(
                f"model file {path} is missing and could not be downloaded ({exc}). Copy the "
                "weights there by hand, or give model:= a full path."
            ) from exc
        if not path.is_file():
            raise DetectorUnavailable(f"model file {path} is missing after the download attempt")
    return YoloBackend(YOLO(str(path)), cv2, np, device, score_threshold)
