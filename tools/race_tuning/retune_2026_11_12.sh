#!/usr/bin/env bash
# The 2026-11-12 retune of the race profile for 1.0 to 1.6 m/s (docs/notes/safety-race-retune-
# 2026-11-12.md). The variant list and the rule (tools/race_tuning/selection_gate.py) were committed
# BEFORE the first run. Needs docker and the ros-dev image; the data tree is mounted at /data.
#
#   tools/race_tuning/retune_2026_11_12.sh tuning    # the 16 variants on the 8 wide tuning tracks
#   tools/race_tuning/retune_2026_11_12.sh heldout A  # the held-out wide tracks, 3 repeats pooled
#                                                     # (A = the candidate name the tuning chose)
#
# References (not candidates): the profile as shipped at its own 1.0 m/s, and at 1.6 and 2.08 m/s
# (the 2026-11-12 run, max_speed_mps 2.08, with the old stopping law). Candidates, each at 1.0, 1.3
# and 1.6 m/s top speed, on top of race-2026-11-12:
#   A  free_path_on_published_steering (the stopping law on the arc the car is on)
#   B  A with stopping_margin_m 0.2 (was 0.1)
#   C  B with lane_lookahead_m 0.8 (was 0.6)
#   D  B with a_lat_max_mps2 1.0 (was 1.5)
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."

MODE="${1:?tuning or heldout}"
OUT_ROOT="${OUT_ROOT:-/data/runs/safety-race}"
JOBS="${JOBS:-8}"

variant() { # name speed extra-json-members
  printf -- "%s={\"profile\": \"race-2026-11-12\", \"params\": {\"max_speed_mps\": %s%s}}" "$1" "$2" "$3"
}

refs=(
  "ref10|1.0|"
  "ref16|1.6|"
  "ref208|2.08|"
)
extra() { # candidate letter -> the extra params
  case "$1" in
    A) printf ', "free_path_on_published_steering": true' ;;
    B) printf ', "free_path_on_published_steering": true, "stopping_margin_m": 0.2' ;;
    C) printf ', "free_path_on_published_steering": true, "stopping_margin_m": 0.2, "lane_lookahead_m": 0.8' ;;
    D) printf ', "free_path_on_published_steering": true, "stopping_margin_m": 0.2, "a_lat_max_mps2": 1.0' ;;
  esac
}

args=()
if [ "$MODE" = tuning ]; then
  for r in "${refs[@]}"; do
    IFS='|' read -r name speed more <<<"$r"
    args+=(--variant "$(variant "$name" "$speed" "$more")")
  done
  for k in A B C D; do
    for v in 1.0 1.3 1.6; do
      args+=(--variant "$(variant "${k}_v${v}" "$v" "$(extra "$k")")")
    done
  done
  exec tools/race_tune_docker.sh evaluate --family wide --tuning-tracks --rule gate \
    --out "$OUT_ROOT/tuning" --laps 2 --jobs "$JOBS" "${args[@]}"
fi

if [ "$MODE" = heldout ]; then
  winner="${2:?the winning candidate, e.g. B_v1.6}"
  k="${winner%%_v*}"
  v="${winner##*_v}"
  args+=(--variant "$(variant "ref10" 1.0 "")")
  args+=(--variant "$(variant "ref16" 1.6 "")")
  args+=(--variant "$(variant "$winner" "$v" "$(extra "$k")")")
  exec tools/race_tune_docker.sh evaluate --family wide --rule gate --repeats 3 \
    --out "$OUT_ROOT/heldout" --laps 3 --jobs "$JOBS" "${args[@]}"
fi
echo "mode must be tuning or heldout" >&2
exit 2
