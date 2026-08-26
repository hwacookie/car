"""Offline design check for the on-site U-turn (Wenden), DRIVING_MANEUVERS.md §5.

Generates the maneuver's reference line with the SAME kinematics the game
uses (bicycle model) and checks that all four body corners stay on the
pavement for a range of road widths. Pure math - no pygame, no game.

Frame: car starts at the origin travelling SOUTH (heading 180 deg), so
  forward f = (0, -1); right r = (-1, 0) [west]; left = (+1, 0) [east].
The road is a straight strip of width W centred on the y-axis:
  pavement = { |x| <= W/2 }.

Three-point turn (§5b):
  step 1: forward, drift to the right kerb, stop            (A1)
  step 2: forward, LEFT, until heading change = th2         (A2, stop)
  step 3: reverse, RIGHT, until total change = th3          (A3, stop)
  step 4: forward, LEFT, until total change = 180 deg, then
          straighten out and blend to the normal lane offset.

Finding from run 1: at FULL lock (R_rear = 3.46 m) no angle pair fits a
7 m road - the game car's minimum radius is tighter than a real car's, so
the spec's "voll links / voll rechts" must be read as "turn hard". This
version sweeps the steering angle and finds a robust recipe.

All angles in RADIANS internally.
"""
import math

L = 2.7                      # wheelbase (BicycleNav.WHEELBASE)
MAX_STEER = math.radians(38.0)
CAR_LEN, CAR_WID = 4.4, 1.8
FRONT_OVER = CAR_LEN / 2 - 1.364          # 0.836 m ahead of front axle
REAR_OVER = CAR_LEN / 2 - 1.276           # 0.924 m behind rear axle
FRONT = L + FRONT_OVER                    # 3.536 m: rear axle -> front bumper
HALF_W = CAR_WID / 2
KERB_CLEARANCE = 0.35                     # config.KERB_CLEARANCE_M


def kerb_offset(w: float) -> float:
    return max(0.0, w / 2.0 - HALF_W - KERB_CLEARANCE)


def corners(x, y, h):
    """Four body corners (metres) for a rear-axle state; h in radians."""
    f = (math.sin(h), math.cos(h))
    r = (math.cos(h), -math.sin(h))
    out = []
    for lf in (FRONT, -REAR_OVER):
        for lr in (HALF_W, -HALF_W):
            out.append((x + f[0] * lf + r[0] * lr, y + f[1] * lf + r[1] * lr))
    return out


def arc_forward(x, y, h, dh_target, steer, ds=0.025):
    """Forward LEFT arc (steer = magnitude of wheel angle) until heading has
    changed by dh_target (rad)."""
    k = math.tan(steer) / L               # rad per metre
    pts = [(x, y, h)]
    travelled = 0.0
    while travelled < dh_target / k - 1e-9:
        h -= k * ds
        x += math.sin(h) * ds
        y += math.cos(h) * ds
        travelled += ds
        pts.append((x, y, h))
    return x, y, h, pts


def arc_reverse(x, y, h, dh_target_total, current_total, steer, ds=0.025):
    """Reverse RIGHT arc until TOTAL heading change (left, rad) reaches
    dh_target_total."""
    k = math.tan(steer) / L
    pts = [(x, y, h)]
    while current_total < dh_target_total - 1e-9:
        dh = k * ds
        h -= dh                            # reverse + right steer keeps rotating left
        x -= math.sin(h) * ds              # moving BACKWARD
        y -= math.cos(h) * ds
        current_total += dh
        pts.append((x, y, h))
    return x, y, h, pts


def check_line(pts, w):
    limit = w / 2.0
    best_margin = float("inf")
    worst = None
    for (x, y, h) in pts:
        for (cx, cy) in corners(x, y, h):
            m = limit - abs(cx)
            if m < best_margin:
                best_margin = m
                worst = ((x, y, math.degrees(h)), (cx, cy))
    return best_margin, worst


def three_point(w, steer_deg, th2_deg, th3_deg):
    steer = math.radians(steer_deg)
    th2, th3 = math.radians(th2_deg), math.radians(th3_deg)
    c = kerb_offset(w)
    L1 = 10.0
    h0 = math.pi                            # south
    pts = []
    n = int(L1 / 0.25)
    for i in range(n + 1):
        t = i / n
        lat = min(1.75, c) + (c - min(1.75, c)) * t   # blend lane -> kerb (west = -x)
        pts.append((-lat, -t * L1, h0))
    x, y, h = -c, -L1, h0
    x, y, h, p2 = arc_forward(x, y, h, th2, steer)
    pts += p2[1:]
    x, y, h, p3 = arc_reverse(x, y, h, th3, th2, steer)
    pts += p3[1:]
    x, y, h, p4 = arc_forward(x, y, h, math.pi - th3, steer)
    pts += p4[1:]
    # extend 15 m along the new direction; blend to the NORMAL lane offset
    # on the new right side (old left = +x) over ~6 m.
    lat_target = min(1.75, c)
    n = int(15.0 / 0.25)
    for i in range(1, n + 1):
        t = min(1.0, (i * 0.25) / 6.0)
        lat = x + (lat_target - x) * t
        pts.append((lat, y - 0.25 * i, h))
    margin, worst = check_line(pts, w)
    return margin, worst, (x, y, math.degrees(h)), len(pts)


if __name__ == "__main__":
    print(f"full-lock R_rear = {L / math.tan(MAX_STEER):.3f} m\n")

    for w in (7.0,):
        print("=" * 78)
        print(f"road width {w:.1f} m (kerb offset {kerb_offset(w):.2f} m)")
        print("=" * 78)
        rows = []
        for steer_deg in (34, 36, 38):
            for th2 in (55, 60, 65, 70):
                for th3 in (95, 100, 105, 110, 115):
                    margin, worst, end, npts = three_point(w, steer_deg, th2, th3)
                    rows.append((margin, steer_deg, th2, th3, end))
        rows.sort(reverse=True)
        print(f"{'steer':>6} {'th2':>4} {'th3':>4}  min-margin   end (x, y, heading)")
        for margin, sd, th2, th3, end in rows[:10]:
            verdict = "OK " if margin > 0.15 else ("marg" if margin > 0 else "OFF ")
            print(f"{sd:6d} {th2:4d} {th3:4d}   {margin:+7.2f} m [{verdict}]  "
                  f"({end[0]:+6.2f}, {end[1]:+7.2f}, {end[2]:5.1f})")

    # Feasibility of the best recipe across widths
    print()
    print("=" * 78)
    print("Best recipe (from W=7 sweep) checked across road widths")
    print("=" * 78)
    BEST = None
    rows = []
    for steer_deg in (34, 36, 38):
        for th2 in (55, 60, 65, 70):
            for th3 in (95, 100, 105, 110, 115):
                margin, *_ = three_point(7.0, steer_deg, th2, th3)
                rows.append((margin, steer_deg, th2, th3))
    rows.sort(reverse=True)
    BEST = rows[0][1:]
    for w in (3.5, 7.0, 10.0, 14.0):
        margin, worst, end, npts = three_point(w, *BEST)
        verdict = "OK" if margin > 0.15 else ("marginal" if margin > 0 else "OFF-ROAD")
        print(f"  width {w:4.1f} m  min-margin {margin:+6.2f} m [{verdict}]  "
              f"end=({end[0]:+.2f}, {end[1]:+.2f})")
