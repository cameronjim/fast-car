# M-B4: label sensitivity, train data

Addendum 1 section 2.4. A measurement on TRAIN data only, produced and committed before any v0.4 fit. No threshold; the gate items under T2s are the guard.

- Segments: the train and internal validation parts of the train segments of 2026-11-14 only; the variants are computed there and not on the older bags.
- Constants file evaluation/prereg/pnp-v0.4.yaml, SHA-256 78510c1423278f80c42511e6caab095818598af6db6a284dd5479f74a03a1d63.

Cells: median over windows of ground distance over wheel distance in 1.0 s windows (T2: wheel seeded chain; s0.80: seeded with 0.80 of the wheel translation; shift: scan clock moved by 10 ms). A stratum with at least 30 windows in every variant whose T2 and s0.80 medians differ by more than 0.03 is reported as "label depends on the seed".

| stratum | windows | T2 | s0.80 | shift+0.010 | shift-0.010 | reading |
|---|---|---|---|---|---|---|
| ccw:0.25-0.6 | 398 | 0.904 | 0.901 | 0.904 | 0.903 |  |
| ccw:0.6-0.85 | 181 | 0.918 | 0.915 | 0.917 | 0.917 |  |
| ccw:0.85+ | 589 | 0.932 | 0.929 | 0.931 | 0.932 |  |
| cw:0.25-0.6 | 153 | 0.890 | 0.885 | 0.890 | 0.890 |  |
| cw:0.6-0.85 | 157 | 0.886 | 0.883 | 0.886 | 0.886 |  |
| cw:0.85+ | 233 | 0.882 | 0.881 | 0.882 | 0.882 |  |

Windows rejected by the ICP quality flags (of the candidate windows whose intervals were all matched), under T2:

| direction | speed bin | candidates | kept | rejected share |
|---|---|---|---|---|
| ccw | 0.25-0.6 | 398 | 398 | 0.000 |
| ccw | 0.6-0.85 | 188 | 181 | 0.037 |
| ccw | 0.85-1.25 | 630 | 582 | 0.076 |
| ccw | 1.25+ | 10 | 7 | 0.300 |
| cw | 0.25-0.6 | 153 | 153 | 0.000 |
| cw | 0.6-0.85 | 157 | 157 | 0.000 |
| cw | 0.85-1.25 | 232 | 223 | 0.039 |
| cw | 1.25+ | 10 | 10 | 0.000 |
