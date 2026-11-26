# trackperc-v9 (2026-11-25): internal screen failed on one item, not scored on H, loop stopped

Decision 38: v9 is screened on the arc labels (selflabels-v5-arc) alone, with the v4-fast numbers
reported; H stays exactly as registered. Prestatement:
`evaluation/prereg/w2-trackperc-v9-2026-11-25.yaml` (f3d441d, committed before any v9 training).
v8's verdict (FAIL) stands and was not rescreened. No v9 bundle: by the stated selection, no
configuration qualified, so no seeds 1 and 2 were trained.

**Held-out scoring count: H-layout 3 (v3, v4, v5), H-session 3 (v3, v4, v5). v9 did not add one.**

## Screen (arc labels, internal w2-v6 part, refined, seed 0 of each configuration)

| config | V s2 | V 11-16 | V old | E_c s2 | E_c 11-16 | E_c old |
|---|---|---|---|---|---|---|
| spacing 0.03, floor 0.15 | 15.37 | 15.23 | 15.71 | 0.1386 | 0.1148 | 0.06679 |
| spacing 0.03, floor 0.20 | 15.38 | 15.24 | 15.71 | 0.1386 | 0.1148 | 0.06679 |
| spacing 0.05, floor 0.15 | 15.44 | 15.29 | 15.73 | 0.1396 | 0.1144 | 0.06677 |
| spacing 0.05, floor 0.20 | 15.44 | 15.29 | 15.73 | 0.1397 | 0.1144 | 0.06677 |
| limits | >= 15.15 | >= 15.07 | >= 15.15 | <= 0.1408 | <= 0.1157 | **<= 0.06629** |

Items 1, 2 and 4 pass everywhere, as does item 3 on session 2 and 2026-11-16. Item 3 on the older
bags (E_c <= v5-dev's 0.06629) fails for all four, by 0.7 percent. v9 FAILS. The sigma floor made
no difference between 0.15 and 0.20 (sigma no longer limits V).

Reported, decides nothing: on the v4-fast labels, spacing 0.05 / floor 0.15 has older-bag E_c
0.0815 (v2 0.0743, v5-dev 0.0751), session 2 0.1678 and 2026-11-16 0.1551.

## Reading, and why the loop stops

Across v8 and v9 the trade is now one number. Every arc-trained configuration with a spacing prior
(0.03 to 0.1) passes V and the new-bag E_c items, but sits 0.7 to 1.5 percent over v5-dev's E_c
on the older bags. That's because it keeps more far stations there (V 15.71 to 15.77 against
15.37). The one without the prior (v8 spacing 0) passes that item but misses V by 0.01 to 0.03.
There is no unexplored setting between 0 and 0.03 that would plausibly clear both by a margin
worth an H scoring. Searching it would be fitting the screen, not improving the net.

Had v9 passed, H would have been scored on the registered v4-fast labels. There an arc-trained net
is about 8 to 10 percent worse on older-bag E_c than v2 internally, so the H-layout E_c item would
very likely have failed. Proposal, not acted on (a prereg change for the coordinator or owner):
score H with arc labels of the H bags, made by the same flagged labeller, for v2 and the candidate
alike.
