"""The YOLO detector behind detector_node (roadmap 5.1), isolated so ROS code never imports it.

`ultralytics` (and the torch it pulls in) exist only in a car image built with
INSTALL_PERCEPTION=required (docker/car/README.md "Perception"). Every import of them is lazy
and inside `load_yolo_backend`, so this module, and detector_node.py with a fake backend,
import and test fine without them (CI's ros-dev image has neither). When they are missing,
`load_yolo_backend` raises DetectorUnavailable with an actionable message and detector_node
logs ONE error and exits non-zero.

Backend contract (what detector_node needs, and what tests fake):

    infer(jpeg: bytes) -> list[RawDetection]     class name, score, pixel box (centre, size),
                                                 plus body keypoints in pose mode
    annotate(jpeg: bytes, dets) -> bytes         the JPEG with boxes (and skeletons) drawn

Two tasks: "detect" (boxes, any COCO class) and "pose" (people with 17 COCO keypoints each, the
yolov8n-pose family). In pose mode a RawDetection's `keypoints` is filled; in detect mode it is
empty. `image_size` reports the (width, height) of the last frame inferred.

WEIGHTS ARE NEVER COMMITTED, and never fetched unpinned. racer_perception/weights.py maps a
bare name to a verified file (model cache, the car image's baked directory, or a download from
the pinned URL) and refuses a sha256 mismatch.
"""

from __future__ import annotations

import os
import re
from collections.abc import Sequence
from pathlib import Path
from typing import Protocol

from racer_perception.fusion import RawDetection
from racer_perception.keypoints import keypoints_from_flat
from racer_perception.weights import DEFAULT_MODEL, WeightsError, locate_weights

#: Detector tasks.
TASKS = ("detect", "pose")

#: COCO-17 skeleton, pairs of keypoint indices to join when drawing a pose.
SKELETON_EDGES = (
    (5, 6), (5, 7), (7, 9), (6, 8), (8, 10), (5, 11), (6, 12), (11, 12),
    (11, 13), (13, 15), (12, 14), (14, 16), (0, 1), (0, 2), (1, 3), (2, 4),
)  # fmt: skip

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
    def __init__(self, yolo, cv2, np, device: str, score_threshold: float, task: str) -> None:
        self._yolo = yolo
        self._task = task
        self.image_size: tuple[int, int] = (0, 0)
        #: Last frame's ultralytics timing in ms: preprocess, inference, postprocess.
        self.last_speed_ms: dict[str, float] = {}
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
        self.image_size = (int(image.shape[1]), int(image.shape[0]))
        result = self._yolo.predict(image, device=self._device, conf=self._conf, verbose=False)[0]
        self.last_speed_ms = {k: float(v) for k, v in result.speed.items()}
        kp_xy = kp_conf = None
        if self._task == "pose" and result.keypoints is not None:
            kp_xy = result.keypoints.xy.tolist()
            kp_conf = result.keypoints.conf.tolist() if result.keypoints.conf is not None else None
        out: list[RawDetection] = []
        for i, box in enumerate(result.boxes):
            cx, cy, w, h = (float(v) for v in box.xywh[0].tolist())
            keypoints = ()
            if kp_xy is not None:
                keypoints = keypoints_from_flat(kp_xy[i], kp_conf[i] if kp_conf else None)
            out.append(
                RawDetection(
                    class_name=str(result.names[int(box.cls[0])]),
                    score=float(box.conf[0]),
                    cx_px=cx,
                    cy_px=cy,
                    width_px=w,
                    height_px=h,
                    keypoints=keypoints,
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
            pts = d.keypoints
            for a, b in SKELETON_EDGES:
                if len(pts) > max(a, b) and pts[a].confidence > 0.3 and pts[b].confidence > 0.3:
                    cv2.line(
                        image,
                        (int(pts[a].x_px), int(pts[a].y_px)),
                        (int(pts[b].x_px), int(pts[b].y_px)),
                        (255, 128, 0),
                        2,
                    )
            for p in pts:
                if p.confidence > 0.3:
                    cv2.circle(image, (int(p.x_px), int(p.y_px)), 3, (0, 0, 255), -1)
        ok, buf = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            raise ValueError("could not encode the debug JPEG")
        return buf.tobytes()


def load_yolo_backend(
    model: str, device: str, cache_dir: str, score_threshold: float, task: str = "detect"
) -> YoloBackend:
    """Import ultralytics lazily and build the backend; raise DetectorUnavailable on any gap.

    `model` empty means the pinned default for `task` (yolov8n.pt, yolov8n-pose.pt).
    """
    if task not in TASKS:
        raise DetectorUnavailable(f"task must be one of {list(TASKS)}, got {task!r}")
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

    try:
        path = locate_weights(model or DEFAULT_MODEL[task], cache_dir)
    except WeightsError as exc:
        raise DetectorUnavailable(str(exc)) from exc
    return YoloBackend(YOLO(str(path), task=task), cv2, np, device, score_threshold, task)
