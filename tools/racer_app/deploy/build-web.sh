#!/usr/bin/env bash
# Build tools/racer_app/web/dist, the web UI the agent serves. Called by install.sh; safe to run
# on its own and as often as you like:
#
#     tools/racer_app/deploy/build-web.sh            # builds only if the sources changed
#     RACER_REBUILD_WEB=1 tools/racer_app/deploy/build-web.sh
#
# The Jetson needs no npm: the build runs in a pinned node container (the same image every time,
# so the same bytes as CI). A host node is used only if it is new enough for vite (22.12 or
# later), which JetPack's own nodejs is not. RACER_WEB_BUILDER=docker or npm forces one.
#
# What makes a re-run reliable:
#   * dist/ is replaced only after a build succeeded, in a rename, so a failed build (no
#     network, a bad package) leaves the UI that is serving now untouched.
#   * node_modules/ is kept between runs and `npm ci` only runs when package-lock.json changed
#     (a stamp inside node_modules), so a plain update after `git pull` takes seconds, not minutes.
#   * the npm cache lives in web/.npm-cache, so a retry or a later install works from it.
#   * `npm ci` is retried once (Jetson Wi-Fi), and a missing node image is pulled with a retry.
#   * the typecheck is left to CI (`npm run build` = tsc + vite): CI fails on a type error, so
#     the car's install never stops on one; this runs vite's build only.
#
# Test hooks (the agent's pytest): RACER_DOCKER (the docker command), RACER_WEB_BUILDER.
set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WEB="$APP_DIR/web"
DOCKER="${RACER_DOCKER:-docker}"
NODE_IMAGE="node:22.12.0-bookworm-slim"
BUILDER="${RACER_WEB_BUILDER:-auto}"

say() { printf '==> %s\n' "$*"; }
die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

[ -f "$WEB/package-lock.json" ] || die "cannot find $WEB/package-lock.json"

# -- already built and current? ----------------------------------------------------------------
if [ -f "$WEB/dist/index.html" ] && [ -z "${RACER_REBUILD_WEB:-}" ] \
  && [ -z "$(find "$WEB/src" "$WEB/index.html" "$WEB/package.json" "$WEB/package-lock.json" \
    "$WEB/vite.config.ts" "$WEB/tsconfig.json" -newer "$WEB/dist/index.html" 2> /dev/null | head -1)" ]; then
  say "web UI is already built and current (RACER_REBUILD_WEB=1 forces a rebuild)"
  exit 0
fi

# -- files left behind by an earlier run as another user (sudo, or a root container) ------------
for path in "$WEB/node_modules" "$WEB/dist" "$WEB/dist.new" "$WEB/.npm-cache"; do
  if [ -e "$path" ] && [ ! -w "$path" ]; then
    die "$path is not writable by $(id -un) (an earlier run as root?). Fix: sudo chown -R $(id -un): $WEB"
  fi
done
rm -rf "$WEB/dist.new" "$WEB/dist.old"

# What runs inside the node container (or on a new enough host node), from the web directory.
read -r -d '' BUILD_SCRIPT << 'EOF' || true
set -e
want="$(sha256sum package-lock.json | cut -d' ' -f1)"
if [ ! -x node_modules/.bin/vite ] || [ "$(cat node_modules/.racer-lock 2> /dev/null)" != "$want" ]; then
  echo "installing npm packages from package-lock.json"
  npm ci --no-audit --no-fund --prefer-offline || {
    echo "npm ci failed, retrying once in 5 s"
    sleep 5
    npm ci --no-audit --no-fund --prefer-offline
  }
  echo "$want" > node_modules/.racer-lock
else
  echo "npm packages are current"
fi
echo "building the web UI with vite"
node_modules/.bin/vite build --outDir dist.new --emptyOutDir
EOF

host_node_ok() {
  if ! command -v npm > /dev/null || ! command -v node > /dev/null; then return 1; fi
  local v major minor
  v="$(node --version 2> /dev/null | sed 's/^v//')"
  major="${v%%.*}"
  minor="${v#*.}"
  minor="${minor%%.*}"
  [[ "$major" =~ ^[0-9]+$ && "$minor" =~ ^[0-9]+$ ]] || return 1
  [ "$major" -gt 22 ] || { [ "$major" -eq 22 ] && [ "$minor" -ge 12 ]; }
}

build_on_host() {
  say "building the web UI with the host's node $(node --version)"
  (cd "$WEB" && npm_config_cache="$WEB/.npm-cache" sh -c "$BUILD_SCRIPT")
}

build_in_container() {
  command -v "$DOCKER" > /dev/null || die "docker is not installed, and the host has no node 22.12 or later to build with"
  if ! "$DOCKER" image inspect "$NODE_IMAGE" > /dev/null 2>&1; then
    say "pulling $NODE_IMAGE (needed once, then it is cached)"
    "$DOCKER" pull "$NODE_IMAGE" > /dev/null \
      || { sleep 5; "$DOCKER" pull "$NODE_IMAGE" > /dev/null; } \
      || die "cannot pull $NODE_IMAGE: the Jetson needs internet once to build the web UI (or copy a built web/dist from CI's racer-app-web-dist artifact)"
  fi
  say "building the web UI in the pinned node container ($NODE_IMAGE)"
  "$DOCKER" run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -e npm_config_cache=/app/.npm-cache -e npm_config_update_notifier=false \
    -v "$WEB":/app -w /app "$NODE_IMAGE" sh -c "$BUILD_SCRIPT"
}

ok=0
case "$BUILDER" in
  npm) build_on_host && ok=1 ;;
  docker) build_in_container && ok=1 ;;
  auto) if host_node_ok; then build_on_host && ok=1; else build_in_container && ok=1; fi ;;
  *) die "RACER_WEB_BUILDER must be auto, docker or npm" ;;
esac

if [ "$ok" -ne 1 ] || [ ! -f "$WEB/dist.new/index.html" ]; then
  rm -rf "$WEB/dist.new"
  die "the web UI build failed; the UI that was there (if any) is unchanged. Re-run, or read the output above"
fi

# Swap in the new build. Two renames, so there is no moment without a dist/ for the agent to serve.
if [ -d "$WEB/dist" ]; then mv "$WEB/dist" "$WEB/dist.old"; fi
mv "$WEB/dist.new" "$WEB/dist"
rm -rf "$WEB/dist.old"
say "web UI built: $WEB/dist"
