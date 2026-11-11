"""Run the detector backend over images and report latency (roadmap 5.1). Not part of the car.

    python3 -m racer_perception.bench --task pose --device cpu test/fixtures/images/*.jpg
    python3 -m racer_perception.bench --task pose --resize-width 1280 --runs 30 img.jpg
    python3 -m racer_perception.bench --task pose --dump-keypoints out.json img.jpg

It goes through exactly the code detector_node uses (`load_yolo_backend`, `backend.infer` on JPEG
bytes), so what it measures is the node's per-frame cost minus the ROS transport. Needs the
perception image (torch, ultralytics). Prints one line per image and a summary: wall time of
`infer` (JPEG decode, ultralytics preprocess, model, postprocess) and ultralytics' own
preprocess / inference / postprocess split, median and 95th percentile over `--runs` frames per
image after `--warmup` discarded frames.

`--resize-width W` rescales each image to W pixels wide first (aspect kept), re-encoded as JPEG,
to see the cost of the real camera's 1280x720 frames from 640 px test photographs.

`--dump-keypoints FILE` writes the detections (boxes and keypoints) as JSON, which is how the
rule fixtures in test/fixtures/keypoints were made.

CUDA numbers can only come from a Jetson; on the Mac this is the CPU only (docs/notes/
spiderbot-2026-11-11.md).
"""

from __future__ import annotations

import argparse
import json
import platform
import statistics
import sys
import time
from pathlib import Path

from racer_perception.yolo_backend import DEFAULT_CACHE_DIR, load_yolo_backend


def percentile(values: list[float], pct: float) -> float:
    ordered = sorted(values)
    if not ordered:
        raise ValueError("no values")
    rank = (len(ordered) - 1) * pct / 100.0
    lower = int(rank)
    upper = min(lower + 1, len(ordered) - 1)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (rank - lower)


def _resized_jpeg(path: Path, width: int | None) -> bytes:
    data = path.read_bytes()
    if width is None:
        return data
    import cv2
    import numpy as np

    image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), 1)
    scale = width / image.shape[1]
    image = cv2.resize(
        image, (width, round(image.shape[0] * scale)), interpolation=cv2.INTER_LINEAR
    )
    ok, buf = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    if not ok:
        raise ValueError("could not re-encode the resized image")
    return buf.tobytes()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("images", nargs="+", type=Path)
    parser.add_argument("--task", choices=("detect", "pose"), default="pose")
    parser.add_argument("--model", default="")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--cache-dir", default=DEFAULT_CACHE_DIR)
    parser.add_argument("--score-threshold", type=float, default=0.25)
    parser.add_argument("--warmup", type=int, default=3)
    parser.add_argument("--runs", type=int, default=20)
    parser.add_argument("--resize-width", type=int, default=None)
    parser.add_argument("--dump-keypoints", type=Path, default=None)
    parser.add_argument("--json", type=Path, default=None, help="write the summary as JSON")
    args = parser.parse_args(argv)

    backend = load_yolo_backend(
        args.model, args.device, args.cache_dir, args.score_threshold, args.task
    )
    import torch

    walls: list[float] = []
    splits: dict[str, list[float]] = {"preprocess": [], "inference": [], "postprocess": []}
    dump: dict[str, list[dict]] = {}
    for path in args.images:
        jpeg = _resized_jpeg(path, args.resize_width)
        for _ in range(args.warmup):
            backend.infer(jpeg)
        per_image: list[float] = []
        detections = []
        for _ in range(args.runs):
            started = time.perf_counter()
            detections = backend.infer(jpeg)
            per_image.append(1e3 * (time.perf_counter() - started))
            for key, series in splits.items():
                series.append(backend.last_speed_ms.get(key, float("nan")))
        walls.extend(per_image)
        print(
            f"{path.name}: {backend.image_size[0]}x{backend.image_size[1]}  "
            f"{len(detections)} detections  median {statistics.median(per_image):.1f} ms  "
            f"p95 {percentile(per_image, 95):.1f} ms"
        )
        dump[path.name] = [
            {
                "class_name": d.class_name,
                "score": d.score,
                "box_cxcywh_px": [d.cx_px, d.cy_px, d.width_px, d.height_px],
                "keypoints": [[k.x_px, k.y_px, k.confidence] for k in d.keypoints],
            }
            for d in detections
        ]

    summary = {
        "task": args.task,
        "device": args.device,
        "model": args.model or "default",
        "resize_width": args.resize_width,
        "frames": len(walls),
        "wall_ms_median": statistics.median(walls),
        "wall_ms_p95": percentile(walls, 95),
        "ultralytics_ms_median": {k: statistics.median(v) for k, v in splits.items()},
        "torch": torch.__version__,
        "torch_threads": torch.get_num_threads(),
        "cuda_available": torch.cuda.is_available(),
        "machine": f"{platform.machine()} {platform.processor() or ''}".strip(),
    }
    print(json.dumps(summary, indent=2))
    if args.json:
        args.json.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    if args.dump_keypoints:
        args.dump_keypoints.write_text(json.dumps(dump, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
