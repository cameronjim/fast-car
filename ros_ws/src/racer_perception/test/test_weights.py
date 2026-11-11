"""L1: pinned weights, checksums, and the pose parsing of the YOLO backend (no ultralytics)."""

from __future__ import annotations

import hashlib
from pathlib import Path
from types import SimpleNamespace

import pytest
from racer_perception.keypoints import COCO17_NAMES, keypoints_from_flat
from racer_perception.weights import (
    DEFAULT_MODEL,
    KNOWN_WEIGHTS,
    PinnedWeights,
    WeightsError,
    locate_weights,
    parse_manifest,
    sha256_file,
    verify,
)
from racer_perception.yolo_backend import DetectorUnavailable, YoloBackend, load_yolo_backend

MANIFEST = Path(__file__).resolve().parents[4] / "docker" / "car" / "perception-weights.txt"


def _pin(tmp_path, name="w.pt", payload=b"weights"):
    f = tmp_path / name
    f.write_bytes(payload)
    return f, PinnedWeights(name, hashlib.sha256(payload).hexdigest(), "https://example.invalid/x")


def test_the_dockerfile_manifest_and_the_python_table_agree():
    manifest = parse_manifest(MANIFEST.read_text(encoding="utf-8"))
    assert manifest == KNOWN_WEIGHTS


def test_default_models_are_pinned():
    assert set(DEFAULT_MODEL.values()) <= set(KNOWN_WEIGHTS)
    for pin in KNOWN_WEIGHTS.values():
        assert len(pin.sha256) == 64 and pin.url.startswith("https://")


def test_verify_accepts_a_match_and_refuses_a_mismatch(tmp_path):
    f, pin = _pin(tmp_path)
    verify(f, pin)
    f.write_bytes(b"tampered")
    with pytest.raises(WeightsError, match="Refusing"):
        verify(f, pin)
    assert sha256_file(f) == hashlib.sha256(b"tampered").hexdigest()


def test_parse_manifest_skips_comments_and_blank_lines():
    text = "# c\n\nname.pt " + "a" * 64 + " https://x/y\n"
    assert parse_manifest(text) == {"name.pt": PinnedWeights("name.pt", "a" * 64, "https://x/y")}


def test_cached_known_weights_are_verified_not_trusted(tmp_path, monkeypatch):
    f, pin = _pin(tmp_path, name="yolov8n.pt")
    monkeypatch.setitem(KNOWN_WEIGHTS, "yolov8n.pt", pin)
    assert locate_weights("yolov8n.pt", str(tmp_path), baked_dir="") == f
    f.write_bytes(b"corrupted")
    with pytest.raises(WeightsError):
        locate_weights("yolov8n.pt", str(tmp_path), baked_dir="")


def test_baked_directory_is_searched_after_the_cache(tmp_path, monkeypatch):
    baked = tmp_path / "baked"
    baked.mkdir()
    f, pin = _pin(baked, name="yolov8n.pt")
    monkeypatch.setitem(KNOWN_WEIGHTS, "yolov8n.pt", pin)
    assert locate_weights("yolov8n.pt", str(tmp_path / "cache"), baked_dir=str(baked)) == f


def test_baked_directory_comes_from_the_environment(tmp_path, monkeypatch):
    baked = tmp_path / "baked"
    baked.mkdir()
    f, pin = _pin(baked, name="yolov8n.pt")
    monkeypatch.setitem(KNOWN_WEIGHTS, "yolov8n.pt", pin)
    monkeypatch.setenv("RACER_WEIGHTS_DIR", str(baked))
    assert locate_weights("yolov8n.pt", str(tmp_path / "cache")) == f


def test_missing_known_weights_are_downloaded_verified_and_kept(tmp_path, monkeypatch):
    _, pin = _pin(tmp_path, name="seed.pt", payload=b"fresh")
    pin = PinnedWeights("yolov8n.pt", pin.sha256, pin.url)
    monkeypatch.setitem(KNOWN_WEIGHTS, "yolov8n.pt", pin)
    calls = []

    def downloader(url, dest):
        calls.append(url)
        dest.write_bytes(b"fresh")

    got = locate_weights("yolov8n.pt", str(tmp_path / "cache"), baked_dir="", downloader=downloader)
    assert got.read_bytes() == b"fresh" and calls == [pin.url]


def test_a_download_that_does_not_match_is_deleted_and_refused(tmp_path, monkeypatch):
    pin = PinnedWeights("yolov8n.pt", "0" * 64, "https://example.invalid/x")
    monkeypatch.setitem(KNOWN_WEIGHTS, "yolov8n.pt", pin)

    def downloader(url, dest):
        dest.write_bytes(b"evil")

    cache = tmp_path / "cache"
    with pytest.raises(WeightsError):
        locate_weights("yolov8n.pt", str(cache), baked_dir="", downloader=downloader)
    assert not (cache / "yolov8n.pt").exists()


def test_a_failed_download_says_how_to_fetch_by_hand(tmp_path):
    def downloader(url, dest):
        raise OSError("no network")

    with pytest.raises(WeightsError, match="by hand"):
        locate_weights("yolov8n.pt", str(tmp_path), baked_dir="", downloader=downloader)


def test_unknown_bare_names_are_never_downloaded(tmp_path):
    def downloader(url, dest):  # pragma: no cover - must not be called
        raise AssertionError("downloaded an unpinned file")

    with pytest.raises(WeightsError, match="not a known pinned"):
        locate_weights("evil.pt", str(tmp_path), baked_dir="", downloader=downloader)
    (tmp_path / "custom.pt").write_bytes(b"mine")
    assert locate_weights("custom.pt", str(tmp_path), baked_dir="") == tmp_path / "custom.pt"


def test_explicit_paths_are_used_and_known_names_still_checked(tmp_path, monkeypatch):
    f, _ = _pin(tmp_path, name="custom.pt")
    assert locate_weights(str(f), "/nowhere") == f
    with pytest.raises(WeightsError, match="does not exist"):
        locate_weights(str(tmp_path / "gone.pt"), "/nowhere")
    sub = tmp_path / "models"
    sub.mkdir()
    known, pin = _pin(sub, name="yolov8n.pt")
    monkeypatch.setitem(KNOWN_WEIGHTS, "yolov8n.pt", pin)
    assert locate_weights(str(known), "/nowhere") == known
    known.write_bytes(b"changed")
    with pytest.raises(WeightsError):
        locate_weights(str(known), "/nowhere")


def test_empty_model_is_refused():
    with pytest.raises(ValueError):
        locate_weights("", "/tmp")


def test_loader_rejects_a_bad_task_before_importing_anything():
    with pytest.raises(DetectorUnavailable, match="task"):
        load_yolo_backend("", "cpu", "/tmp/none", 0.25, "segment")


# -- keypoints and the backend's pose parsing, with fake ultralytics objects ----------------


def test_keypoints_from_flat_with_and_without_confidence():
    xy = [[1.0, 2.0], [3.0, 4.0]]
    assert keypoints_from_flat(xy, [0.5, 0.0])[1].confidence == 0.0
    assert keypoints_from_flat(xy, None)[0].confidence == 1.0
    with pytest.raises(ValueError):
        keypoints_from_flat(xy, [0.5])


def test_coco_names_are_the_17_in_order():
    assert len(COCO17_NAMES) == 17 and COCO17_NAMES[5] == "left_shoulder"
    assert COCO17_NAMES[10] == "right_wrist"


class _T:
    def __init__(self, v):
        self._v = v

    def tolist(self):
        return self._v

    def __getitem__(self, i):
        return _T(self._v[i])


def _fake_result(n_people, with_keypoints=True, with_conf=True):
    boxes = [
        SimpleNamespace(xywh=_T([[100.0 + i, 200.0, 50.0, 120.0]]), cls=[0], conf=[0.9 - 0.1 * i])
        for i in range(n_people)
    ]
    keypoints = None
    if with_keypoints:
        xy = [[[float(10 * i + k), float(k)] for k in range(17)] for i in range(n_people)]
        conf = [[0.5 + 0.01 * k for k in range(17)] for _ in range(n_people)]
        keypoints = SimpleNamespace(xy=_T(xy), conf=_T(conf) if with_conf else None)
    return SimpleNamespace(
        boxes=boxes,
        names={0: "person"},
        keypoints=keypoints,
        speed={"preprocess": 1.0, "inference": 2.0, "postprocess": 3.0},
    )


class _Yolo:
    def __init__(self, result):
        self.result = result

    def predict(self, image, **kw):
        return [self.result]


class _Cv2:
    @staticmethod
    def imdecode(buf, flag):
        return SimpleNamespace(shape=(480, 640, 3))


class _Np:
    uint8 = "u8"

    @staticmethod
    def frombuffer(data, dtype):
        return data


def _backend(result, task):
    return YoloBackend(_Yolo(result), _Cv2, _Np, "cpu", 0.25, task)


def test_pose_backend_returns_keypoints_per_person_in_order():
    b = _backend(_fake_result(2), "pose")
    dets = b.infer(b"jpeg")
    assert len(dets) == 2 and all(len(d.keypoints) == 17 for d in dets)
    assert dets[1].keypoints[3].x_px == pytest.approx(13.0)
    assert dets[0].keypoints[16].confidence == pytest.approx(0.66)
    assert b.image_size == (640, 480)
    assert b.last_speed_ms == {"preprocess": 1.0, "inference": 2.0, "postprocess": 3.0}


def test_pose_backend_without_keypoint_confidence_or_keypoints():
    dets = _backend(_fake_result(1, with_conf=False), "pose").infer(b"j")
    assert dets[0].keypoints[0].confidence == 1.0
    assert _backend(_fake_result(1, with_keypoints=False), "pose").infer(b"j")[0].keypoints == ()


def test_detect_task_ignores_keypoints():
    assert _backend(_fake_result(1), "detect").infer(b"j")[0].keypoints == ()
