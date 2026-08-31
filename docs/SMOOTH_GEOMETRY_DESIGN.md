# §10 Smoothed geometry — design notes (implementation companion)

> Companion to `TURN_REWORK_PLAN.md` §10. Status: implemented on branch
> `turn-planning-rework`. This file records the non-obvious decisions and
> the verification evidence.
>
> **Addendum 2026-08-31 — junction corners replaced by Eckausrundung:**
> the node-centred fillet arcs described below (R = 6 m through the node,
> buffered by a road half-width) are REPLACED. At every degree>=3 corner
> the grass corner where two road EDGES meet is now rounded with a circular
> arc of `config.JUNCTION_CORNER_RADIUS_M` (4 m, fixed; TODO: should depend
> on road class / design vehicle per RASt/RAL "nach Art und Lage der
> Straße"), tangent to both edges - the standard Eckausrundung
> (de.wikipedia.org/wiki/Eckausrundung). The paved fill is the curvilinear
> triangle between corner point and arc, built directly (no buffering).
>
> Why: the buffered node-centred capsules bulged up to ~5.4 m past the
> kerbs where road widths differ (measured at the 3.5 m one-way × 7 m
> crossing, test map tile (2,1)): two of the four corners were buffered by
> the wide road's half-width and two by the narrow one's (seg_a = first
> spoke in angle order), producing asymmetric lobes - visually "the wide
> road bending into the narrow one". Rendering and drivable area were
> affected alike (same shared polygons).
>
> Unchanged: the driving reference line still rounds route corners with
> `BicycleNav.CORNER_RADIUS_M` (6 m through the node); e2e 19/19 green
> with the new paved geometry, so the two remain compatible. The
> `junction_fillets` entries now carry `corner` + `arc` (+ seg_a/seg_b,
> radius_px) instead of t1/t2; consumers: `_build_smoothed_junction_fillets`
> (paved fill) and `obstacles.py` (parked-car alignment along the arc).

## One curve, all consumers

```
RoadNetwork (graph: nodes + segments, UNTOUCHED)
    |
    |  _merge_and_round_lines: linemerge through degree-2 nodes
    |  (merged line = chain of original OSM nodes; junction nodes are
    |   the line ENDS - the graph's degree>=3 nodes)
    v
SmoothedNetwork (built once, cached)
    |
    +-- per merged line: SmoothCurve = centripetal Catmull-Rom THROUGH
    |   the original nodes (interpolating, C1, clamped endpoints so the
    |   end tangent = last-chord direction, per §10)
    |
    +-- per degree>=3 junction: circular fillet arc (R = 6 m, capped by
    |   adjacent edge lengths) between the SAME tangent points the old
    |   _round_polyline_corners used -> tangent-continuous with the
    |   splines (the spline end tangent IS the chord direction)
    |
    +-- resampled dense polylines (0.5 m) of curves + fillets
            |
            +--> paved polygon        (buffer of the SAME lines, as before)
            +--> centerline dashes    (the SAME lines)
            +--> lane markings        (offsets of the SAME lines)
            +--> driving reference    (sub-curves of the SAME lines +
                 line (BicycleNav)         the SAME fillet arcs, lane-offset)
```

The resampled polylines are evaluation caches of the curve functions
(§10: "the only internal artifact is an arc-length lookup table"). No new
graph nodes, no way splits.

## Curvature: geometry-based, never table-based

`SmoothCurve.curvature_at(s)` is a central difference of `point_at`
(window `max(1.0 m, 1% of total)`, same formula the old `RefLine` used).
The per-sample `kap` table of the first `SmoothCurve` draft is GONE — it
was corrupted at every piece junction (duplicate sample → ds=0 → heading
0.0 spike → κ spikes up to ~16 1/m). With the duplicate samples removed
from the arc-length table AND curvature computed from geometry, both
failure modes are impossible.

## Why circular fillets (not the proposed Bézier)

`scripts/visualize_junction_fillets.py` measured the driven line's peak
curvature for the five test-map junction types:

| junction | circular fillet | single cubic Bézier |
|---|---|---|
| 90° corner | R = 7.47 m ok | R = 7.71 m ok |
| T-junction | R = 7.47 m ok | R = 7.71 m ok |
| Y-junction | R = 4.24 m ok | **R = 1.88 m — infeasible** |
| crossroads | R = 7.38 m ok | R = 7.71 m ok |
| sliver | R = 0.31 m (clamped by 4.2 m stub) | R = 0.34 m (same) |

A single cubic Bézier between the fillet tangent points overshoots the
arc's peak curvature for turn angles above ~100° (the Y-junction fork is
~120°). §10's "chained Bézier/arc for large angles" is therefore not
worth the complexity: the circular arc is already tangent-continuous with
the splines (the spline's end tangent is the last-chord direction, exactly
what the fillet is tangent to), feasible wherever the paved corner is,
and is what the renderer already draws. The Bézier idea is retired; the
circular fillet IS the §10 junction connection.

## Feasibility of the driving line

`BicycleNav.MIN_TURN_RADIUS_M = WHEELBASE / tan(MAX_STEER) = 2.7 /
tan(38°) ≈ 3.46 m`.

The driving line's max curvature is checked at every (re)build
(`_check_line_feasibility`, printed once per distinct value). Lines whose
peak κ exceeds 1/3.46 m are inherently tight — the only case on the test
map is the sliver junction, whose paved fillet is clamped by the 4.16 m
approach stub (tangent distance ≤ half the stub). The car's ACTUAL motion
stays feasible: the speed profile caps v at sqrt(A_LAT_MAX/κ) and the
bicycle model's lateral-accel cap + MAX_STEER clamp make the car cut the
corner at its 3.46 m limit (verified: all three sliver tests pass, no
validator violations). Clamping the reference line itself to 3.46 m would
push it OUT of the paved polygon (the pavement is tighter than the car
can turn) — i.e. the "fix" would make the car leave the road. So the
check reports, it does not silently reshape the line; the validator stays
the wall.

## What changed where

- `src/smooth_geometry.py` (NEW): `SmoothCurve` (kap table removed,
  duplicate samples removed, geometry-based `curvature_at`),
  `resample_curve`, `corner_fillet` (extracted from
  `_round_polyline_corners`), `SmoothedNetwork` (per-line curves +
  junction fillets + resampled polylines, cached).
- `src/bicycle_nav.py`: `SmoothCurve` moved out; `RefLine` now wraps a
  `SmoothCurve` (public interface unchanged: `total`, `point_at`,
  `heading_at`, `curvature_at`); `_maybe_rebuild` builds the route line
  from `SmoothedNetwork` sub-curves + fillets instead of
  `_round_polyline_corners` on raw chords; feasibility check added.
- `src/road_network.py`: `_merge_and_round_lines` /
  `_build_road_polygons` / `_build_centerlines` / `_build_lane_markings`
  / `_build_multiway_junction_fillets` consume `SmoothedNetwork` instead
  of `_round_polyline_corners`; `_round_polyline_corners` removed.
- `scripts/visualize_junction_fillets.py`: imports updated.

## Verification

- 18/18 deterministic tests green (0 off-road, 0 snaps, 0 teleports,
  0 wrong segment) — see `tests/turning_results.json`.
- `tests/test_road_network.py` green.
- Curvature sanity: sampled κ of every merged-line curve on the `basic`
  map is smooth (no spikes at piece junctions); the roundabout ring reads
  the true ring curvature (R ≈ 100 m) uniformly instead of 0-on-chord-
  middles / spike-at-kinks.
- Same-curve proof: renderer, on-road check and driving all consume
  `SmoothedNetwork` (single cached instance per `RoadNetwork`).
