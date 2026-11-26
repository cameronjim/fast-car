# Old vs arc self-labels against the map (2026-11-25): not clearly closer, H not touched

Step 1 of the owner-approved H relabel plan: before any H relabelling, check whether the arc
labels (selflabels-v5-arc, `--arc-length-stations --corner-smooth-m 0.1`) are clearly closer than
the old ones (selflabels-v4-fast) to ground truth the labeller never saw. **They are not clearly
closer, so the plan stops here.** H was not relabelled, no amendment was written, no candidate was
scored. H counts: H-layout 3, H-session 3 (v3, v4, v5).

Tool: `racer_pnp_perception.label_map_check`. Every label station goes into the bag's own
occupancy grid with the localiser pose (`map -> odom -> base_link` at the scan's header stamp).
Only scans labelled in both sets are compared, with the same station count per scan.

## Which ground truth exists

| bag | role | map / pose | scan-to-map residual (median of per-scan medians) | usable |
|---|---|---|---|---|
| 2026-11-13T23-27-23 (tracker laps) | train | saved map, AMCL | 0.034 m (p90 0.049) | yes |
| 2026-11-12T03-04-56 | H-layout | /map and /pose | 0.101 m (p90 0.416) | no, misaligned |
| 2026-11-24T04-36-54 (session 2) | train / H-session | none | none | no |
| 2026-11-16 bags | train | none | none | no |

No measured geometry of the session 2 or 2026-11-16 layouts exists in the docs. That leaves one
dev bag with a trustworthy map, on a wider layout (label lane width about 1.4 m) than session 2.

## Result (2026-11-13T23-27-23, 165 scans in both sets, 2640 stations)

| measure | old | arc |
|---|---|---|
| wall distance error, mean (|hw label - wall distance in map|) | 0.120 m | 0.119 m |
| centring error, mean (|d_left - d_right| / 2 in map) | 0.115 m | 0.112 m |
| stations with plausible walls on both sides (< 0.8 m), centring error | | |
| . where the sets differ by < 0.05 m (545) | 0.022 | 0.021 |
| . where they differ by 0.05 to 0.15 m (50) | 0.061 | 0.056 (closer on 66%) |
| . where they differ by > 0.15 m (22) | 0.061 | 0.056 (closer on 59%) |
| curvature: share of stations with radius < 0.8 x map half width (plausible walls) | 3.6% | 1.1% |
| radius p1 (plausible walls) | 0.49 m | 0.57 m |
| station spacing, p5 / min | 0.188 / 0.110 m | 0.200 / 0.200 m |

## Reading

* **Spacing:** the arc labels meet the 0.20 m contract and the old ones don't. That is true by
  construction and says nothing about where the corridor is.
* **Curvature:** the arc labels have fewer centreline radii tighter than a centreline can
  physically be. That favours arc.
* **Wall distance and centring, the direct position test:** the two sets are within 1 to 3
  percent overall. Where they differ, arc is closer by 0.005 m on 72 stations (59 to 66 percent
  of them). That is consistent with arc being better, but small, on one bag, one layout, and not
  the layout where the square corners mattered most (session 2, which has no map).

That is not "clearly closer to ground truth" for the question H relabelling depends on, so
following the instruction I stop. What would settle it: a map-localised run on the session 2
layout (AMCL or tracker laps, as on 2026-11-13), or a measured drawing of that layout. Then rerun
`label_map_check` on that bag before anything touches H.
