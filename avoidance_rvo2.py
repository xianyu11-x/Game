"""
RVO2 avoidance algorithm - faithful port of the official RVO2 Library (Agent.cc).
Reference: https://github.com/snape/RVO2/blob/main/src/Agent.cc
License: Apache-2.0

Key differences from the custom ORCA implementation:
- Correct incremental 2D linear programming (linearProgram1/2/3)
- Proper infeasible constraint relaxation (minimizes max violation)
- No neighbor filtering (all neighbors within range contribute constraints)
- No min_speed forcing, no inertia blending after LP solve
- Obstacle support via line-segment ORCA constraints
"""

import math
import avoidance_config
from avoidance_base import AvoidanceBase
from avoidance_config import AVOIDANCE_CONFIG

RVO_EPSILON = 1e-5


def det(v1, v2):
    return v1[0] * v2[1] - v1[1] * v2[0]


def dot(v1, v2):
    return v1[0] * v2[0] + v1[1] * v2[1]


def abs_sq(v):
    return v[0] * v[0] + v[1] * v[1]


def length(v):
    return math.sqrt(abs_sq(v))


def normalize(v):
    l = length(v)
    if l < RVO_EPSILON:
        return (0.0, 0.0)
    return (v[0] / l, v[1] / l)


def scale(v, s):
    return (v[0] * s, v[1] * s)


def add(v1, v2):
    return (v1[0] + v2[0], v1[1] + v2[1])


def sub(v1, v2):
    return (v1[0] - v2[0], v1[1] - v2[1])


def neg(v):
    return (-v[0], -v[1])


# ---------------------------------------------------------------------------
# LP solver – direct port of RVO2 linearProgram1/2/3
# ---------------------------------------------------------------------------

def linear_program_1(lines, line_no, radius, opt_velocity, direction_opt, result):
    """Solve 1D LP on line `line_no` subject to previous line constraints and speed disc."""
    line = lines[line_no]
    dot_product = dot(line["point"], line["direction"])
    discriminant = dot_product * dot_product + radius * radius - abs_sq(line["point"])

    if discriminant < 0.0:
        return False, result

    sqrt_disc = math.sqrt(discriminant)
    t_left = -dot_product - sqrt_disc
    t_right = -dot_product + sqrt_disc

    for i in range(line_no):
        denom = det(line["direction"], lines[i]["direction"])
        numer = det(lines[i]["direction"], sub(line["point"], lines[i]["point"]))

        if abs(denom) <= RVO_EPSILON:
            if numer < 0.0:
                return False, result
            continue

        t = numer / denom
        if denom >= 0.0:
            t_right = min(t_right, t)
        else:
            t_left = max(t_left, t)

        if t_left > t_right:
            return False, result

    if direction_opt:
        if dot(opt_velocity, line["direction"]) > 0.0:
            result = add(line["point"], scale(line["direction"], t_right))
        else:
            result = add(line["point"], scale(line["direction"], t_left))
    else:
        t = dot(line["direction"], sub(opt_velocity, line["point"]))
        if t < t_left:
            result = add(line["point"], scale(line["direction"], t_left))
        elif t > t_right:
            result = add(line["point"], scale(line["direction"], t_right))
        else:
            result = add(line["point"], scale(line["direction"], t))

    return True, result


def linear_program_2(lines, radius, opt_velocity, direction_opt, result):
    """Solve 2D LP. Returns (fail_line, result). fail_line == len(lines) means success."""
    if direction_opt:
        result = scale(opt_velocity, radius)
    elif abs_sq(opt_velocity) > radius * radius:
        result = scale(normalize(opt_velocity), radius)
    else:
        result = opt_velocity

    for i in range(len(lines)):
        if det(lines[i]["direction"], sub(lines[i]["point"], result)) > 0.0:
            temp_result = result
            ok, result = linear_program_1(lines, i, radius, opt_velocity, direction_opt, result)
            if not ok:
                result = temp_result
                return i, result

    return len(lines), result


def linear_program_3(lines, num_obst_lines, begin_line, radius, result):
    """Handle infeasible LP: minimise maximum constraint violation."""
    distance = 0.0

    for i in range(begin_line, len(lines)):
        if det(lines[i]["direction"], sub(lines[i]["point"], result)) > distance:
            proj_lines = list(lines[:num_obst_lines])

            for j in range(num_obst_lines, i):
                determinant = det(lines[i]["direction"], lines[j]["direction"])

                if abs(determinant) <= RVO_EPSILON:
                    if dot(lines[i]["direction"], lines[j]["direction"]) > 0.0:
                        continue
                    pt = scale(add(lines[i]["point"], lines[j]["point"]), 0.5)
                else:
                    pt = add(
                        lines[i]["point"],
                        scale(
                            lines[i]["direction"],
                            det(lines[j]["direction"], sub(lines[i]["point"], lines[j]["point"])) / determinant
                        )
                    )

                direction = normalize(sub(lines[j]["direction"], lines[i]["direction"]))
                proj_lines.append({"point": pt, "direction": direction})

            temp_result = result
            opt_dir = (lines[i]["direction"][1], -lines[i]["direction"][0])
            fail, result = linear_program_2(proj_lines, radius, opt_dir, True, result)

            if fail < len(proj_lines):
                result = temp_result

            distance = det(lines[i]["direction"], sub(lines[i]["point"], result))

    return result


# ---------------------------------------------------------------------------
# RVO2 Avoidance Plugin
# ---------------------------------------------------------------------------

class RVO2Avoidance(AvoidanceBase):
    """RVO2 avoidance – faithful port of the official RVO2 Library."""

    def __init__(self):
        super().__init__()
        self.config = AVOIDANCE_CONFIG.get("rvo2", {})
        self.name = self.config.get("name", "RVO2")
        self.description = self.config.get("description", "Official RVO2 algorithm port")

    def calculate_avoidance(self, circle, target_x, target_y, all_circles, delta_time):
        if target_x is None or target_y is None:
            if avoidance_config.DEBUG_ENABLED:
                self._clear_debug(circle)
            return self._move_with_current(circle, delta_time)

        max_speed = circle["speed"]
        radius_self = circle["radius"]
        position = (circle["x"], circle["y"])
        velocity = self._get_velocity(circle)

        neighbor_dist = self.config.get("neighbor_dist", 150.0)
        time_horizon = self.config.get("time_horizon", 2.5)
        time_horizon_obst = self.config.get("time_horizon_obst", 0.5)
        max_neighbors = self.config.get("max_neighbors", 10)
        attack_range_hard_stop = self.config.get("attack_range_hard_stop", True)

        pref_velocity = self._preferred_velocity(circle, target_x, target_y, max_speed)

        # In attack range, prefer immediate stop (RTS-style hold position).
        # This avoids oscillating around the target due to reciprocal constraints.
        if attack_range_hard_stop and self._is_target_in_attack_range(circle, target_x, target_y):
            new_velocity = (0.0, 0.0)
            if avoidance_config.DEBUG_ENABLED:
                circle["debug_orca_pref_velocity"] = pref_velocity
                circle["debug_orca_new_velocity"] = new_velocity
                circle["debug_orca_lines"] = []
                circle["debug_orca_num_obst_lines"] = 0
                circle["debug_orca_neighbor_dist"] = neighbor_dist
                circle["debug_orca_velocity_scale"] = 0.5
                circle["debug_orca_time_horizon"] = time_horizon
                circle["debug_detection_radius"] = neighbor_dist
                circle["debug_rvo2_obst_segments"] = []
                circle["debug_orca_stats"] = {
                    "neighbors": 0,
                    "constraints": 0,
                    "obst_constraints": 0,
                    "pref_speed": length(pref_velocity),
                    "new_speed": 0.0,
                    "collision_pred": False,
                    "nearest_dist": 0.0,
                    "target_dist": math.hypot(target_x - circle["x"], target_y - circle["y"]),
                    "filtered_neighbors": 0,
                    "lp_feasible": True,
                    "fail_line": -1,
                    "stuck_retry_used": False,
                    "stuck_retry_fail_line": -1,
                    "attack_hold": True,
                }

            circle["_rvo2_velocity"] = new_velocity
            return (circle["x"], circle["y"], circle["angle"])

        # ----- collect obstacle ORCA lines -----
        orca_lines = []
        if self.obstacles:
            inv_t_obst = 1.0 / time_horizon_obst
            obst_segments = self._obstacles_to_segments()
            for seg_start, seg_end in obst_segments:
                self._add_obstacle_orca_line(
                    position, velocity, radius_self, seg_start, seg_end,
                    inv_t_obst, orca_lines)
        num_obst_lines = len(orca_lines)

        # ----- collect agent ORCA lines -----
        neighbors = self._get_neighbors(
            circle, target_x, target_y, all_circles, neighbor_dist, max_neighbors
        )
        inv_t = 1.0 / time_horizon

        stationary_responsibility = self.config.get("stationary_responsibility", 1.0)
        for other in neighbors:
            other_vel = self._get_velocity(other)
            neighbor_stationary = (abs(other_vel[0]) < RVO_EPSILON and
                                   abs(other_vel[1]) < RVO_EPSILON)
            # If other is stationary, we take full responsibility (1.0);
            # otherwise split equally (0.5) — standard reciprocal assumption.
            responsibility = stationary_responsibility if neighbor_stationary else 0.5

            rel_pos = sub((other["x"], other["y"]), position)
            rel_vel = sub(velocity, other_vel)
            dist_sq = abs_sq(rel_pos)
            combined_r = radius_self + other["radius"]
            combined_r_sq = combined_r * combined_r

            if dist_sq > combined_r_sq:
                w = sub(rel_vel, scale(rel_pos, inv_t))
                w_len_sq = abs_sq(w)
                dot_p = dot(w, rel_pos)

                if dot_p < 0.0 and dot_p * dot_p > combined_r_sq * w_len_sq:
                    w_len = math.sqrt(w_len_sq)
                    if w_len < RVO_EPSILON:
                        continue
                    unit_w = scale(w, 1.0 / w_len)
                    direction = (unit_w[1], -unit_w[0])
                    u = scale(unit_w, combined_r * inv_t - w_len)
                else:
                    dist = math.sqrt(dist_sq)
                    leg = math.sqrt(max(0.0, dist_sq - combined_r_sq))

                    if det(rel_pos, w) > 0.0:
                        direction = (
                            (rel_pos[0] * leg - rel_pos[1] * combined_r) / dist_sq,
                            (rel_pos[0] * combined_r + rel_pos[1] * leg) / dist_sq,
                        )
                    else:
                        direction = (
                            -(rel_pos[0] * leg + rel_pos[1] * combined_r) / dist_sq,
                            -(-rel_pos[0] * combined_r + rel_pos[1] * leg) / dist_sq,
                        )

                    u = sub(scale(direction, dot(rel_vel, direction)), rel_vel)
            else:
                inv_ts = 1.0 / max(delta_time, 1e-6)
                w = sub(rel_vel, scale(rel_pos, inv_ts))
                w_len = length(w)
                if w_len < RVO_EPSILON:
                    continue
                unit_w = scale(w, 1.0 / w_len)
                direction = (unit_w[1], -unit_w[0])
                u = scale(unit_w, combined_r * inv_ts - w_len)

            line_point = add(velocity, scale(u, responsibility))
            orca_lines.append({"point": line_point, "direction": direction})

        # ----- solve LP -----
        new_velocity = (0.0, 0.0)
        fail_line, new_velocity = linear_program_2(orca_lines, max_speed, pref_velocity, False, new_velocity)
        lp_feasible = (fail_line == len(orca_lines))
        if not lp_feasible:
            new_velocity = linear_program_3(orca_lines, num_obst_lines, fail_line, max_speed, new_velocity)

        # ----- stuck mitigation: break symmetric deadlocks -----
        stuck_retry_used = False
        retry_fail_line = -1
        pref_speed = length(pref_velocity)
        new_speed = length(new_velocity)
        target_dist = math.hypot(target_x - circle["x"], target_y - circle["y"])
        stuck_speed_threshold = self.config.get("stuck_speed_threshold", 6.0)
        stuck_pref_speed_min = self.config.get("stuck_pref_speed_min", 20.0)
        stuck_target_dist_min = self.config.get("stuck_target_dist_min", radius_self * 3.0)
        side_bias_ratio = self.config.get("stuck_side_bias", 0.30)

        if (len(neighbors) > 0 and
                target_dist > stuck_target_dist_min and
                pref_speed > stuck_pref_speed_min and
                new_speed < stuck_speed_threshold):
            pref_dir = normalize(pref_velocity)
            if abs(pref_dir[0]) > RVO_EPSILON or abs(pref_dir[1]) > RVO_EPSILON:
                label = str(circle.get("label", ""))
                sign = 1.0 if (sum(ord(ch) for ch in label) % 2 == 0) else -1.0
                side_dir = (-pref_dir[1] * sign, pref_dir[0] * sign)
                biased_pref = add(pref_velocity, scale(side_dir, max_speed * side_bias_ratio))
                if abs_sq(biased_pref) > max_speed * max_speed:
                    biased_pref = scale(normalize(biased_pref), max_speed)

                alt_velocity = (0.0, 0.0)
                fail2, alt_velocity = linear_program_2(orca_lines, max_speed, biased_pref, False, alt_velocity)
                if fail2 < len(orca_lines):
                    alt_velocity = linear_program_3(orca_lines, num_obst_lines, fail2, max_speed, alt_velocity)
                retry_fail_line = fail2

                if length(alt_velocity) > new_speed + 0.5:
                    new_velocity = alt_velocity
                    fail_line = fail2
                    lp_feasible = (fail2 == len(orca_lines))
                    stuck_retry_used = True

        # ----- debug output -----
        if avoidance_config.DEBUG_ENABLED:
            nearest_dist = None
            for other in all_circles:
                if other is circle:
                    continue
                d = math.hypot(other["x"] - circle["x"], other["y"] - circle["y"])
                if nearest_dist is None or d < nearest_dist:
                    nearest_dist = d

            circle["debug_orca_pref_velocity"] = pref_velocity
            circle["debug_orca_new_velocity"] = new_velocity
            circle["debug_orca_lines"] = orca_lines
            circle["debug_orca_num_obst_lines"] = num_obst_lines
            circle["debug_orca_neighbor_dist"] = neighbor_dist
            circle["debug_orca_velocity_scale"] = 0.5
            circle["debug_orca_time_horizon"] = time_horizon
            circle["debug_detection_radius"] = neighbor_dist
            circle["debug_rvo2_obst_segments"] = (
                obst_segments if self.obstacles else []
            )
            circle["debug_orca_stats"] = {
                "neighbors": len(neighbors),
                "constraints": len(orca_lines),
                "obst_constraints": num_obst_lines,
                "pref_speed": pref_speed,
                "new_speed": length(new_velocity),
                "collision_pred": not lp_feasible,
                "nearest_dist": nearest_dist or 0.0,
                "target_dist": target_dist,
                "filtered_neighbors": 0,
                "lp_feasible": lp_feasible,
                "fail_line": fail_line if not lp_feasible else -1,
                "stuck_retry_used": stuck_retry_used,
                "stuck_retry_fail_line": retry_fail_line,
            }

        # ----- apply -----
        circle["_rvo2_velocity"] = new_velocity

        new_x = circle["x"] + new_velocity[0] * delta_time
        new_y = circle["y"] + new_velocity[1] * delta_time
        if abs(new_velocity[0]) > RVO_EPSILON or abs(new_velocity[1]) > RVO_EPSILON:
            new_angle = math.degrees(math.atan2(new_velocity[1], new_velocity[0]))
        else:
            new_angle = circle["angle"]

        return (new_x, new_y, new_angle)

    # ----- helpers -----

    def _preferred_velocity(self, circle, target_x, target_y, max_speed):
        dx = target_x - circle["x"]
        dy = target_y - circle["y"]
        dist = math.hypot(dx, dy)
        if dist < 0.1:
            return (0.0, 0.0)
        speed = min(max_speed, dist / 0.5)  # decel near target
        return (dx / dist * speed, dy / dist * speed)

    def _get_velocity(self, circle):
        stored = circle.get("_rvo2_velocity")
        if stored is not None:
            return stored
        if circle.get("target_x") is None or circle.get("target_y") is None:
            return (0.0, 0.0)
        if circle.get("speed", 0.0) <= 0.01:
            return (0.0, 0.0)
        angle_rad = math.radians(circle["angle"])
        return (math.cos(angle_rad) * circle["speed"],
                math.sin(angle_rad) * circle["speed"])

    def _get_neighbors(self, circle, target_x, target_y, all_circles, neighbor_dist, max_neighbors):
        """All neighbors within range, sorted by distance, excluding current target."""
        neighbors = []
        target_ref = circle.get("target_ref")
        fallback_target = None
        nd_sq = neighbor_dist * neighbor_dist

        # If target_ref is missing, infer a fallback target circle by target position.
        if target_ref is None and target_x is not None and target_y is not None:
            best_d_sq = float("inf")
            for other in all_circles:
                if other is circle:
                    continue
                tx = other["x"] - target_x
                ty = other["y"] - target_y
                d_sq = tx * tx + ty * ty
                if d_sq < best_d_sq:
                    best_d_sq = d_sq
                    fallback_target = other
            # Guard against false matches: require reasonably close to target point.
            if fallback_target is not None:
                reach = max(
                    circle.get("arrival_threshold", 5.0),
                    circle.get("attack_range", circle.get("radius", 25) * 2.0),
                    fallback_target.get("radius", 25.0),
                )
                if best_d_sq > reach * reach:
                    fallback_target = None

        for other in all_circles:
            if other is circle:
                continue
            if target_ref is not None and other is target_ref:
                continue
            if fallback_target is not None and other is fallback_target:
                continue
            dx = other["x"] - circle["x"]
            dy = other["y"] - circle["y"]
            d_sq = dx * dx + dy * dy
            if d_sq < nd_sq:
                neighbors.append((d_sq, other))

        neighbors.sort(key=lambda x: x[0])
        return [n[1] for n in neighbors[:max_neighbors]]

    def _is_target_in_attack_range(self, circle, target_x, target_y):
        """Check whether the current target is already in attack range."""
        target_ref = circle.get("target_ref")
        if target_ref is not None:
            tx, ty = target_ref["x"], target_ref["y"]
        else:
            tx, ty = target_x, target_y
        if tx is None or ty is None:
            return False

        attack_range = circle.get("attack_range", circle.get("radius", 25.0) * 2.0)
        return math.hypot(tx - circle["x"], ty - circle["y"]) <= attack_range

    def _move_with_current(self, circle, delta_time):
        circle["_rvo2_velocity"] = None
        angle_rad = math.radians(circle["angle"])
        dist = circle["speed"] * delta_time
        return (circle["x"] + dist * math.cos(angle_rad),
                circle["y"] + dist * math.sin(angle_rad),
                circle["angle"])

    def _clear_debug(self, circle):
        circle["debug_orca_pref_velocity"] = None
        circle["debug_orca_new_velocity"] = None
        circle["debug_orca_lines"] = []
        circle["debug_orca_num_obst_lines"] = 0
        circle["debug_orca_stats"] = None
        circle["debug_detection_radius"] = None
        circle["debug_rvo2_obst_segments"] = []
        circle["_rvo2_velocity"] = None

    # ----- obstacle segment support -----

    def _obstacles_to_segments(self):
        """Convert rotated-rect obstacles to line segments."""
        segments = []
        for obs in self.obstacles:
            hw, hh = obs["w"] / 2.0, obs["h"] / 2.0
            angle_rad = math.radians(obs["angle"])
            cos_a = math.cos(angle_rad)
            sin_a = math.sin(angle_rad)
            cx, cy = obs["cx"], obs["cy"]

            corners_local = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
            corners = []
            for lx, ly in corners_local:
                wx = cx + lx * cos_a - ly * sin_a
                wy = cy + lx * sin_a + ly * cos_a
                corners.append((wx, wy))

            for i in range(4):
                segments.append((corners[i], corners[(i + 1) % 4]))

        return segments

    def _add_obstacle_orca_line(self, position, velocity, radius_self,
                                seg_start, seg_end, inv_t_obst, orca_lines):
        """Simplified obstacle ORCA line for a single line segment."""
        rel_p1 = sub(seg_start, position)
        rel_p2 = sub(seg_end, position)

        already_covered = False
        for line in orca_lines:
            if (det(sub(scale(rel_p1, inv_t_obst), line["point"]), line["direction"])
                    - inv_t_obst * radius_self >= -RVO_EPSILON and
                det(sub(scale(rel_p2, inv_t_obst), line["point"]), line["direction"])
                    - inv_t_obst * radius_self >= -RVO_EPSILON):
                already_covered = True
                break
        if already_covered:
            return

        dist_sq1 = abs_sq(rel_p1)
        dist_sq2 = abs_sq(rel_p2)
        radius_sq = radius_self * radius_self

        obst_vec = sub(seg_end, seg_start)
        obst_len_sq = abs_sq(obst_vec)
        if obst_len_sq < RVO_EPSILON:
            return

        s = -dot(rel_p1, obst_vec) / obst_len_sq
        dist_sq_line = abs_sq(add(rel_p1, scale(obst_vec, s)))

        if s < 0.0 and dist_sq1 <= radius_sq:
            w = sub(velocity, scale(rel_p1, inv_t_obst))
            w_len = length(w)
            if w_len < RVO_EPSILON:
                return
            unit_w = scale(w, 1.0 / w_len)
            orca_lines.append({
                "point": add(scale(rel_p1, inv_t_obst), scale(unit_w, radius_self * inv_t_obst)),
                "direction": (unit_w[1], -unit_w[0]),
            })
            return

        if s > 1.0 and dist_sq2 <= radius_sq:
            if det(rel_p2, normalize(obst_vec)) >= 0.0:
                w = sub(velocity, scale(rel_p2, inv_t_obst))
                w_len = length(w)
                if w_len < RVO_EPSILON:
                    return
                unit_w = scale(w, 1.0 / w_len)
                orca_lines.append({
                    "point": add(scale(rel_p2, inv_t_obst), scale(unit_w, radius_self * inv_t_obst)),
                    "direction": (unit_w[1], -unit_w[0]),
                })
            return

        if 0.0 <= s <= 1.0 and dist_sq_line <= radius_sq:
            obst_dir = normalize(obst_vec)
            orca_lines.append({
                "point": (0.0, 0.0),
                "direction": neg(obst_dir),
            })
            return

        # No collision — project on nearest feature (cutoff / left leg / right leg)
        if s < 0.0 and dist_sq_line <= radius_sq:
            # left vertex
            leg1 = math.sqrt(max(0.0, dist_sq1 - radius_sq))
            left_leg_dir = (
                (rel_p1[0] * leg1 - rel_p1[1] * radius_self) / dist_sq1,
                (rel_p1[0] * radius_self + rel_p1[1] * leg1) / dist_sq1,
            )
            right_leg_dir = (
                (rel_p1[0] * leg1 + rel_p1[1] * radius_self) / dist_sq1,
                (-rel_p1[0] * radius_self + rel_p1[1] * leg1) / dist_sq1,
            )
        elif s > 1.0 and dist_sq_line <= radius_sq:
            leg2 = math.sqrt(max(0.0, dist_sq2 - radius_sq))
            left_leg_dir = (
                (rel_p2[0] * leg2 - rel_p2[1] * radius_self) / dist_sq2,
                (rel_p2[0] * radius_self + rel_p2[1] * leg2) / dist_sq2,
            )
            right_leg_dir = (
                (rel_p2[0] * leg2 + rel_p2[1] * radius_self) / dist_sq2,
                (-rel_p2[0] * radius_self + rel_p2[1] * leg2) / dist_sq2,
            )
        else:
            if dist_sq1 < RVO_EPSILON or dist_sq2 < RVO_EPSILON:
                return
            leg1 = math.sqrt(max(0.0, dist_sq1 - radius_sq))
            left_leg_dir = (
                (rel_p1[0] * leg1 - rel_p1[1] * radius_self) / dist_sq1,
                (rel_p1[0] * radius_self + rel_p1[1] * leg1) / dist_sq1,
            )
            leg2 = math.sqrt(max(0.0, dist_sq2 - radius_sq))
            right_leg_dir = (
                (rel_p2[0] * leg2 + rel_p2[1] * radius_self) / dist_sq2,
                (-rel_p2[0] * radius_self + rel_p2[1] * leg2) / dist_sq2,
            )

        # Cutoff check
        left_cutoff = scale(rel_p1, inv_t_obst)
        right_cutoff = scale(rel_p2, inv_t_obst)
        cutoff_vec = sub(right_cutoff, left_cutoff)

        obst_same = (abs_sq(sub(seg_start, seg_end)) < RVO_EPSILON)
        t_val = 0.5 if obst_same else dot(sub(velocity, left_cutoff), cutoff_vec) / max(abs_sq(cutoff_vec), RVO_EPSILON)
        t_left = dot(sub(velocity, left_cutoff), left_leg_dir)
        t_right = dot(sub(velocity, right_cutoff), right_leg_dir)

        if (t_val < 0.0 and t_left < 0.0) or (obst_same and t_left < 0.0 and t_right < 0.0):
            unit_w = normalize(sub(velocity, left_cutoff))
            direction = (unit_w[1], -unit_w[0])
            point = add(left_cutoff, scale(unit_w, radius_self * inv_t_obst))
            orca_lines.append({"point": point, "direction": direction})
            return

        if t_val > 1.0 and t_right < 0.0:
            unit_w = normalize(sub(velocity, right_cutoff))
            direction = (unit_w[1], -unit_w[0])
            point = add(right_cutoff, scale(unit_w, radius_self * inv_t_obst))
            orca_lines.append({"point": point, "direction": direction})
            return

        # Project on nearest (cutoff / left leg / right leg)
        dist_sq_cutoff = (float('inf') if (t_val < 0.0 or t_val > 1.0 or obst_same)
                          else abs_sq(sub(velocity, add(left_cutoff, scale(cutoff_vec, t_val)))))
        dist_sq_left = float('inf') if t_left < 0.0 else abs_sq(sub(velocity, add(left_cutoff, scale(left_leg_dir, t_left))))
        dist_sq_right = float('inf') if t_right < 0.0 else abs_sq(sub(velocity, add(right_cutoff, scale(right_leg_dir, t_right))))

        obst_dir = normalize(obst_vec)

        if dist_sq_cutoff <= dist_sq_left and dist_sq_cutoff <= dist_sq_right:
            direction = neg(obst_dir)
            point = add(left_cutoff, scale((direction[1], -direction[0]), radius_self * inv_t_obst))
            orca_lines.append({"point": point, "direction": direction})
        elif dist_sq_left <= dist_sq_right:
            direction = left_leg_dir
            point = add(left_cutoff, scale((-direction[1], direction[0]), radius_self * inv_t_obst))
            orca_lines.append({"point": point, "direction": direction})
        else:
            direction = neg(right_leg_dir)
            point = add(right_cutoff, scale((-direction[1], direction[0]), radius_self * inv_t_obst))
            orca_lines.append({"point": point, "direction": direction})
