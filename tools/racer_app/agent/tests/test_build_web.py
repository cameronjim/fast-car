"""deploy/build-web.sh: the web build the Jetson runs without npm on the host. Run for real in a
temporary copy of the app folder, with a fake docker (and a fake node) on PATH."""

from __future__ import annotations

import os
import shutil
import stat
import subprocess
import time
from pathlib import Path

import pytest

DEPLOY = Path(__file__).resolve().parents[2] / "deploy"
IMAGE = "node:22.12.0-bookworm-slim"

FAKE_DOCKER = """#!/bin/sh
printf '%s\\n' "$(echo "$*" | tr '\\n' ' ')" >> "$FAKE_DOCKER_LOG"
case "$1" in
  image) [ "${FAKE_DOCKER_HAS_IMAGE:-1}" = 1 ]; exit $? ;;
  pull) [ "${FAKE_DOCKER_PULL_OK:-1}" = 1 ]; exit $? ;;
  run)
    for a in "$@"; do case "$a" in *:/app) src="${a%:/app}" ;; esac; done
    mkdir -p "$src/dist.new"
    if [ "${FAKE_DOCKER_BUILD:-ok}" = fail ]; then echo partial > "$src/dist.new/junk"; exit 1; fi
    echo "built-${FAKE_DOCKER_TAG:-1}" > "$src/dist.new/index.html"
    exit 0 ;;
esac
exit 2
"""


class Sandbox:
    def __init__(self, tmp_path: Path) -> None:
        self.app = tmp_path / "racer_app"
        (self.app / "deploy").mkdir(parents=True)
        shutil.copy(DEPLOY / "build-web.sh", self.app / "deploy" / "build-web.sh")
        self.web = self.app / "web"
        (self.web / "src").mkdir(parents=True)
        for name, text in {
            "package-lock.json": "{}",
            "package.json": "{}",
            "index.html": "<html>",
            "vite.config.ts": "",
            "tsconfig.json": "{}",
        }.items():
            (self.web / name).write_text(text)
        (self.web / "src" / "main.ts").write_text("x")
        self.bin = tmp_path / "bin"
        self.bin.mkdir()
        self.log = tmp_path / "docker.log"
        self.script("docker", FAKE_DOCKER)
        self.env = {
            **os.environ,
            "PATH": f"{self.bin}:{os.environ['PATH']}",
            "RACER_DOCKER": str(self.bin / "docker"),
            "FAKE_DOCKER_LOG": str(self.log),
        }
        self.env.pop("RACER_REBUILD_WEB", None)

    def script(self, name: str, text: str) -> Path:
        path = self.bin / name
        path.write_text(text)
        path.chmod(0o755)
        return path

    def run(self, **extra: str) -> subprocess.CompletedProcess:
        env = {**self.env, **extra}
        return subprocess.run(
            ["bash", str(self.app / "deploy" / "build-web.sh")],
            env=env,
            capture_output=True,
            text=True,
            check=False,
        )

    def docker_calls(self) -> list[str]:
        return self.log.read_text().splitlines() if self.log.exists() else []

    def runs(self) -> list[str]:
        return [c for c in self.docker_calls() if c.startswith("run ")]

    @property
    def index(self) -> Path:
        return self.web / "dist" / "index.html"

    def touch_source(self) -> None:
        later = time.time() + 5
        os.utime(self.web / "src" / "main.ts", (later, later))


@pytest.fixture
def sb(tmp_path: Path) -> Sandbox:
    return Sandbox(tmp_path)


def test_builds_in_the_pinned_node_container_with_no_npm_on_the_host(sb):
    r = sb.run(RACER_WEB_BUILDER="docker")
    assert r.returncode == 0, r.stderr
    assert sb.index.read_text().strip() == "built-1"
    (run,) = sb.runs()
    assert IMAGE in run and "--user" in run and f"{sb.web}:/app" in run
    assert "npm_config_cache=/app/.npm-cache" in run
    assert "vite build --outDir dist.new" in run and "npm ci" in run
    assert "tsc" not in run  # the typecheck is CI's job, not the car's
    assert not (sb.web / "dist.new").exists()


def test_a_second_run_with_nothing_changed_does_not_build_again(sb):
    sb.run(RACER_WEB_BUILDER="docker")
    r = sb.run(RACER_WEB_BUILDER="docker")
    assert r.returncode == 0 and "already built and current" in r.stdout
    assert len(sb.runs()) == 1


def test_a_changed_source_rebuilds_and_replaces_dist(sb):
    sb.run(RACER_WEB_BUILDER="docker", FAKE_DOCKER_TAG="1")
    sb.touch_source()
    sb.run(RACER_WEB_BUILDER="docker", FAKE_DOCKER_TAG="2")
    assert sb.index.read_text().strip() == "built-2"
    assert len(sb.runs()) == 2
    assert not (sb.web / "dist.old").exists()


def test_rebuild_can_be_forced(sb):
    sb.run(RACER_WEB_BUILDER="docker")
    sb.run(RACER_WEB_BUILDER="docker", RACER_REBUILD_WEB="1")
    assert len(sb.runs()) == 2


def test_a_failed_build_leaves_the_ui_that_is_serving_untouched(sb):
    sb.run(RACER_WEB_BUILDER="docker", FAKE_DOCKER_TAG="old")
    sb.touch_source()
    r = sb.run(RACER_WEB_BUILDER="docker", FAKE_DOCKER_BUILD="fail")
    assert r.returncode != 0
    assert "build failed" in r.stderr and "unchanged" in r.stderr
    assert sb.index.read_text().strip() == "built-old"
    assert not (sb.web / "dist.new").exists()


def test_a_failed_first_build_leaves_no_half_dist(sb):
    r = sb.run(RACER_WEB_BUILDER="docker", FAKE_DOCKER_BUILD="fail")
    assert r.returncode != 0
    assert not (sb.web / "dist").exists() and not (sb.web / "dist.new").exists()


def test_a_missing_node_image_is_pulled_first(sb):
    r = sb.run(RACER_WEB_BUILDER="docker", FAKE_DOCKER_HAS_IMAGE="0")
    assert r.returncode == 0, r.stderr
    kinds = [c.split()[0] for c in sb.docker_calls()]
    assert kinds == ["image", "pull", "run"]


def test_no_network_for_the_image_says_what_to_do(sb):
    r = sb.run(RACER_WEB_BUILDER="docker", FAKE_DOCKER_HAS_IMAGE="0", FAKE_DOCKER_PULL_OK="0")
    assert r.returncode != 0
    assert "cannot pull" in r.stderr and "internet" in r.stderr and "racer-app-web-dist" in r.stderr
    assert not sb.runs()


def test_no_docker_and_no_new_node_is_a_clear_error(sb):
    r = sb.run(RACER_WEB_BUILDER="docker", RACER_DOCKER="/nonexistent/docker")
    assert r.returncode != 0 and "docker is not installed" in r.stderr


@pytest.mark.skipif(os.geteuid() == 0, reason="root can write anywhere")
def test_files_left_by_an_earlier_root_run_are_named_with_the_fix(sb):
    nm = sb.web / "node_modules"
    nm.mkdir()
    nm.chmod(0o555)
    try:
        r = sb.run(RACER_WEB_BUILDER="docker")
    finally:
        nm.chmod(0o755)
    assert r.returncode != 0
    assert "not writable" in r.stderr and "chown" in r.stderr
    assert not sb.runs()


def test_an_old_host_node_is_not_used_even_if_npm_exists(sb):
    sb.script("node", "#!/bin/sh\necho v18.19.0\n")
    sb.script("npm", "#!/bin/sh\nexit 99\n")
    r = sb.run()  # auto
    assert r.returncode == 0, r.stderr
    assert len(sb.runs()) == 1


def test_a_new_enough_host_node_is_used_with_the_same_build_steps(sb):
    sb.script("node", "#!/bin/sh\necho v22.14.0\n")
    sb.script("sha256sum", '#!/bin/sh\necho "abc123  $1"\n')
    sb.script(
        "npm",
        '#!/bin/sh\necho "$@" >> "$FAKE_DOCKER_LOG.npm"\n'
        "mkdir -p node_modules/.bin\n"
        "printf '#!/bin/sh\\nmkdir -p dist.new\\necho host-built > dist.new/index.html\\n' "
        "> node_modules/.bin/vite\n"
        "chmod +x node_modules/.bin/vite\n",
    )
    r = sb.run()  # auto
    assert r.returncode == 0, r.stderr
    assert sb.index.read_text().strip() == "host-built"
    assert not sb.runs()
    assert "ci --no-audit" in Path(str(sb.log) + ".npm").read_text()
    # node_modules is kept, and its stamp skips `npm ci` on the next rebuild
    assert (sb.web / "node_modules" / ".racer-lock").read_text().strip() == "abc123"
    sb.touch_source()
    sb.run()
    assert Path(str(sb.log) + ".npm").read_text().count("ci --no-audit") == 1


def test_the_script_is_executable_and_install_calls_it():
    assert os.access(DEPLOY / "build-web.sh", os.X_OK)
    mode = (DEPLOY / "build-web.sh").stat().st_mode
    assert mode & stat.S_IXUSR
    install = (DEPLOY / "install.sh").read_text()
    assert "deploy/build-web.sh" in install and "docker run" not in install
