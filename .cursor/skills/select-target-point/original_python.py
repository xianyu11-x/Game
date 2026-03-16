"""
原始 Python 实现 — get_closest_reachable_target
来源: test.py
此文件仅供移植参考，不参与 UE 构建。
"""
import math


def get_closest_reachable_target(
    circle,
    target_x,
    target_y,
    target_radius,
    all_circles,
    extra_margin=5.0,
    obstacles_list=None,
    attack_range=None,
):
    """Return closest reachable point on an approach circle around the target.

    For ranged units pass *attack_range* (centre-to-centre distance) to prefer
    standing at max attack distance.  Falls back to the collision circle when
    the attack-range circle is fully blocked.

    Fully analytical: blocked arcs from other circles via law of cosines;
    blocked arcs from rectangular obstacles via circle / rounded-rect
    intersection (4 line segments + 4 corner arcs).
    """
    if target_radius <= 0.0:
        return target_x, target_y

    collision_R = circle["radius"] + target_radius + extra_margin
    desired_angle = math.atan2(circle["y"] - target_y, circle["x"] - target_x)
    TWO_PI = 2.0 * math.pi
    cr = circle["radius"]

    # ------------------------------------------------------------------
    def _solve_at_radius(R):
        """Find the best free angle on a circle of radius *R* centred on the
        target.  Returns ``((x, y), blocked_arcs)`` on success, or
        ``(None, blocked_arcs)`` when every angle is blocked."""
        blocked_arcs = []

        for other in all_circles:
            if other is circle:
                continue
            min_dist = cr + other["radius"] + extra_margin
            d = math.hypot(other["x"] - target_x, other["y"] - target_y)
            if d < 1e-9:
                if R < min_dist:
                    return None, blocked_arcs
                continue
            cos_val = (d * d + R * R - min_dist * min_dist) / (2.0 * d * R)
            if cos_val <= -1.0:
                return None, blocked_arcs
            if cos_val >= 1.0:
                continue
            half_angle = math.acos(cos_val)
            center_angle = math.atan2(other["y"] - target_y, other["x"] - target_x)
            blocked_arcs.append((center_angle, half_angle))

        if obstacles_list:
            for obs in obstacles_list:
                hw = obs["w"] / 2.0
                hh = obs["h"] / 2.0
                half_diag = math.hypot(hw, hh)
                d_obs = math.hypot(obs["cx"] - target_x, obs["cy"] - target_y)
                if d_obs - R > half_diag + cr:
                    continue

                obs_angle_rad = math.radians(obs["angle"])
                cos_a = math.cos(-obs_angle_rad)
                sin_a = math.sin(-obs_angle_rad)
                dx = target_x - obs["cx"]
                dy = target_y - obs["cy"]
                ltx = dx * cos_a - dy * sin_a
                lty = dx * sin_a + dy * cos_a

                crossings = []

                # 4 side segments of rounded rect -> circle-line intersections
                for a_val, y_lo, y_hi in ((hw + cr, -hh, hh),
                                           (-(hw + cr), -hh, hh)):
                    disc = R * R - (a_val - ltx) ** 2
                    if disc < 0.0:
                        continue
                    sd = math.sqrt(disc)
                    for y_val in (lty + sd, lty - sd):
                        if y_lo - 1e-9 <= y_val <= y_hi + 1e-9:
                            crossings.append(math.atan2(y_val - lty, a_val - ltx))

                for b_val, x_lo, x_hi in ((hh + cr, -hw, hw),
                                           (-(hh + cr), -hw, hw)):
                    disc = R * R - (b_val - lty) ** 2
                    if disc < 0.0:
                        continue
                    sd = math.sqrt(disc)
                    for x_val in (ltx + sd, ltx - sd):
                        if x_lo - 1e-9 <= x_val <= x_hi + 1e-9:
                            crossings.append(math.atan2(b_val - lty, x_val - ltx))

                # 4 corner arcs -> circle-circle intersections (law of cosines)
                for cx_c, cy_c, qx, qy in ((hw, hh, 1, 1), (-hw, hh, -1, 1),
                                             (-hw, -hh, -1, -1), (hw, -hh, 1, -1)):
                    dd = math.hypot(cx_c - ltx, cy_c - lty)
                    if dd < 1e-9:
                        continue
                    cv = (dd * dd + R * R - cr * cr) / (2.0 * dd * R)
                    if cv < -1.0 or cv > 1.0:
                        continue
                    base = math.atan2(cy_c - lty, cx_c - ltx)
                    ha = math.acos(max(-1.0, min(1.0, cv)))
                    for theta in (base + ha, base - ha):
                        px = ltx + R * math.cos(theta)
                        py = lty + R * math.sin(theta)
                        if qx > 0 and px < hw - 1e-9:
                            continue
                        if qx < 0 and px > -hw + 1e-9:
                            continue
                        if qy > 0 and py < hh - 1e-9:
                            continue
                        if qy < 0 and py > -hh + 1e-9:
                            continue
                        crossings.append(theta)

                def _hit_local(theta):
                    px = ltx + R * math.cos(theta)
                    py = lty + R * math.sin(theta)
                    clx = max(-hw, min(hw, px))
                    cly = max(-hh, min(hh, py))
                    ddx = px - clx
                    ddy = py - cly
                    return ddx * ddx + ddy * ddy < cr * cr

                if not crossings:
                    if _hit_local(0.0):
                        return None, blocked_arcs
                    continue

                crossings.sort()
                filtered = [crossings[0]]
                for c_val in crossings[1:]:
                    if c_val - filtered[-1] > 1e-8:
                        filtered.append(c_val)
                crossings = filtered
                n = len(crossings)

                for i in range(n):
                    a_start = crossings[i]
                    a_end = crossings[(i + 1) % n]
                    if a_end <= a_start:
                        a_end += TWO_PI
                    mid = a_start + (a_end - a_start) / 2.0
                    if _hit_local(mid):
                        w_start = a_start + obs_angle_rad
                        w_end = a_end + obs_angle_rad
                        arc_center = (w_start + w_end) / 2.0
                        arc_half = (w_end - w_start) / 2.0
                        blocked_arcs.append((arc_center, arc_half))

        # --- pick closest free angle to desired_angle ---
        if not blocked_arcs:
            return (target_x + R * math.cos(desired_angle),
                    target_y + R * math.sin(desired_angle)), blocked_arcs

        def _is_blocked(a):
            for c_a, h_a in blocked_arcs:
                if abs((a - c_a + math.pi) % TWO_PI - math.pi) < h_a:
                    return True
            return False

        if not _is_blocked(desired_angle):
            return (target_x + R * math.cos(desired_angle),
                    target_y + R * math.sin(desired_angle)), blocked_arcs

        best_angle = None
        best_diff = float('inf')
        eps = 1e-4
        for c_a, h_a in blocked_arcs:
            for candidate in (c_a + h_a + eps, c_a - h_a - eps):
                if not _is_blocked(candidate):
                    diff = abs((candidate - desired_angle + math.pi) % TWO_PI - math.pi)
                    if diff < best_diff:
                        best_diff = diff
                        best_angle = candidate

        if best_angle is not None:
            return (target_x + R * math.cos(best_angle),
                    target_y + R * math.sin(best_angle)), blocked_arcs

        return None, blocked_arcs
    # ------------------------------------------------------------------

    # --- main: try attack_range circle first, fall back to collision circle ---
    if attack_range is not None and attack_range > collision_R:
        result, b_arcs = _solve_at_radius(attack_range)
        if result is not None:
            return result
        # Attack circle fully blocked -> stay in place
        return circle["x"], circle["y"]

    result, b_arcs = _solve_at_radius(collision_R)
    if result is not None:
        return result
    return target_x, target_y
