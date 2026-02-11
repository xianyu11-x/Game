import pygame
import sys
import math
import avoidance_config
from avoidance_config import AVOIDANCE_CONFIG
from avoidance_manager import AvoidanceManager

# Initialize Pygame
pygame.init()

# Window setup
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 800
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("2D Game - WASD Camera Movement")

# Color definitions
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (100, 149, 237)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)

# Camera settings
camera_x = 0
camera_y = 0
camera_speed = 5

# Store placed circles
circles = []  # Each element: {"x": x, "y": y, "label": label, "target_x": None, "target_y": None, "angle": 0, "speed": 100, "turn_speed": 180, "radius": 25}
circle_counter = 0
selected_circles = []  # Changed to list
global_target = None   # (x, y)
is_dragging = False
drag_start_pos = (0, 0)
obstacles = []  # Each: {"cx": cx, "cy": cy, "w": w, "h": h, "angle": 0}
obstacle_placement_mode = False

# Initialize avoidance manager
avoidance_manager = AvoidanceManager()
plugin_count = avoidance_manager.load_plugins()
print(f"Loaded {plugin_count} avoidance plugins")
print(f"Available algorithms: {avoidance_manager.get_algorithm_names()}")
print(f"Current algorithm: {avoidance_manager.get_algorithm().get_name() if avoidance_manager.get_algorithm() else 'None'}")

# Font setup
font = pygame.font.Font(None, 24)
small_font = pygame.font.Font(None, 18)

# Clock for frame rate control
clock = pygame.time.Clock()

def check_collision(x1, y1, radius1, x2, y2, radius2):
    """Check if two circles collide"""
    distance = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
    return distance < (radius1 + radius2)

def get_collision_response(circle, other_circles):
    """Calculate collision response to push circles apart"""
    import math
    push_x = 0
    push_y = 0
    
    for other in other_circles:
        if other is circle:
            continue
        
        distance = ((other["x"] - circle["x"]) ** 2 + (other["y"] - circle["y"]) ** 2) ** 0.5
        overlap = (circle["radius"] + other["radius"]) - distance
        
        if overlap > 0:
            # Calculate push direction (away from other circle)
            if distance > 0:
                push_x -= (other["x"] - circle["x"]) / distance * overlap * 0.5
                push_y -= (other["y"] - circle["y"]) / distance * overlap * 0.5
    
    return push_x, push_y

def get_closest_reachable_target(
    circle,
    target_x,
    target_y,
    target_radius,
    all_circles,
    extra_margin=5.0,
):
    """Return closest reachable point on the Minkowski circle around the target."""
    if target_radius <= 0.0:
        return target_x, target_y

    base_radius = circle["radius"] + target_radius + extra_margin
    desired_angle = math.atan2(circle["y"] - target_y, circle["x"] - target_x)

    samples = 48
    best_point = None
    best_angle_diff = None

    for i in range(samples):
        angle = (2 * math.pi * i) / samples
        candidate_x = target_x + base_radius * math.cos(angle)
        candidate_y = target_y + base_radius * math.sin(angle)

        ok = True
        for other in all_circles:
            if other is circle:
                continue
            min_dist = circle["radius"] + other["radius"] + extra_margin
            if math.hypot(candidate_x - other["x"], candidate_y - other["y"]) < min_dist:
                ok = False
                break

        if not ok:
            continue

        angle_diff = abs((angle - desired_angle + math.pi) % (2 * math.pi) - math.pi)
        if best_point is None or angle_diff < best_angle_diff:
            best_point = (candidate_x, candidate_y)
            best_angle_diff = angle_diff

    if best_point is not None:
        return best_point

    return target_x, target_y

def move_circle_towards_target(circle, delta_time, all_circles, avoidance_algorithm):
    """Move circle towards its target position with speed and rotation, using avoidance plugin"""
    if circle["target_x"] is None or circle["target_y"] is None:
        return

    extra_margin = 5.0
    target_radius = circle.get("target_radius", 0.0) or 0.0
    base_radius = circle["radius"] + target_radius + extra_margin if target_radius > 0.0 else 0.0
    reach_x, reach_y = get_closest_reachable_target(
        circle,
        circle["target_x"],
        circle["target_y"],
        target_radius,
        all_circles,
        extra_margin=extra_margin,
    )
    if avoidance_config.DEBUG_ENABLED:
        circle["debug_reachable_target"] = (reach_x, reach_y)
    else:
        circle["debug_reachable_target"] = None

    distance_to_target = math.hypot(circle["target_x"] - circle["x"], circle["target_y"] - circle["y"])
    arrival_threshold = circle.get("arrival_threshold", 5)
    if distance_to_target < arrival_threshold:
        circle["target_x"] = None
        circle["target_y"] = None
        circle["target_radius"] = None
        circle["target_ref"] = None
        circle["debug_reachable_target"] = None
        return
    
    # Calculate direction to target
    dx = reach_x - circle["x"]
    dy = reach_y - circle["y"]
    
    # Calculate target angle (in degrees)
    target_angle = math.degrees(math.atan2(dy, dx))
    
    # Normalize angles to -180 to 180
    current_angle = circle["angle"] % 360
    if current_angle > 180:
        current_angle -= 360
    
    target_angle = target_angle % 360
    if target_angle > 180:
        target_angle -= 360
    
    # Calculate angle difference
    angle_diff = target_angle - current_angle
    if angle_diff > 180:
        angle_diff -= 360
    elif angle_diff < -180:
        angle_diff += 360
    
    # Rotate towards target
    max_rotation = circle["turn_speed"] * delta_time
    if abs(angle_diff) < max_rotation:
        circle["angle"] = target_angle
    else:
        circle["angle"] += max_rotation if angle_diff > 0 else -max_rotation
    
    # Use avoidance algorithm if available
    if avoidance_algorithm:
        result = avoidance_algorithm.calculate_avoidance(
            circle, reach_x, reach_y, all_circles, delta_time
        )
        if result:
            new_x, new_y, new_angle = result
            circle["x"] = new_x
            circle["y"] = new_y
            # Update angle from avoidance algorithm for smoother behavior
            circle["angle"] = new_angle
    else:
        # Fallback to simple movement
        move_distance = circle["speed"] * delta_time
        circle["x"] += move_distance * math.cos(math.radians(circle["angle"]))
        circle["y"] += move_distance * math.sin(math.radians(circle["angle"]))
    
    # Apply collision response to separate overlapping circles
    # 只对有目标的圆形应用碰撞响应，避免目标点被推动
    if circle["target_x"] is not None and circle["target_y"] is not None:
        push_x, push_y = get_collision_response(circle, all_circles)
        circle["x"] += push_x
        circle["y"] += push_y

        distance_to_target = math.hypot(circle["target_x"] - circle["x"], circle["target_y"] - circle["y"])
        if distance_to_target < arrival_threshold:
            circle["target_x"] = None
            circle["target_y"] = None
            circle["target_radius"] = None
            circle["target_ref"] = None
            circle["debug_reachable_target"] = None
            return

        if target_radius > 0.0 and distance_to_target < base_radius:
            reach_x, reach_y = get_closest_reachable_target(
                circle,
                circle["target_x"],
                circle["target_y"],
                target_radius,
                all_circles,
                extra_margin=extra_margin,
            )
            if avoidance_config.DEBUG_ENABLED:
                circle["debug_reachable_target"] = (reach_x, reach_y)
            else:
                circle["debug_reachable_target"] = None
            circle["x"] = reach_x
            circle["y"] = reach_y

def draw_grid(surface, camera_x, camera_y):
    """Draw grid background"""
    grid_size = 50
    # Vertical lines
    for x in range(-camera_x % grid_size, WINDOW_WIDTH, grid_size):
        pygame.draw.line(surface, (200, 200, 200), (x, 0), (x, WINDOW_HEIGHT), 1)
    # Horizontal lines
    for y in range(-camera_y % grid_size, WINDOW_HEIGHT, grid_size):
        pygame.draw.line(surface, (200, 200, 200), (0, y), (WINDOW_WIDTH, y), 1)

def draw_allowed_angles(surface, circle, camera_x, camera_y, color=(0, 255, 0, 60)):
    """Draw feasible movement area as translucent sectors on the detection circle."""
    intervals = circle.get("debug_allowed_intervals")
    detection_radius = circle.get("debug_detection_radius")
    if not intervals or not detection_radius:
        return

    screen_x, screen_y = world_to_screen(circle["x"], circle["y"], camera_x, camera_y)
    step = max(2.0, circle.get("debug_angle_step", 10.0) / 2.0)

    overlay = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)

    for start, end in intervals:
        if end <= start:
            continue
        points = [(screen_x, screen_y)]
        angle = start
        while angle <= end:
            end_x = screen_x + detection_radius * math.cos(math.radians(angle))
            end_y = screen_y + detection_radius * math.sin(math.radians(angle))
            points.append((end_x, end_y))
            angle += step

        end_x = screen_x + detection_radius * math.cos(math.radians(end))
        end_y = screen_y + detection_radius * math.sin(math.radians(end))
        points.append((end_x, end_y))

        if len(points) >= 3:
            pygame.draw.polygon(overlay, color, points)

    surface.blit(overlay, (0, 0))

def draw_velocity_direction(surface, circle, camera_x, camera_y, color=(255, 120, 0)):
    """Draw current velocity direction using detection radius as length if available."""
    detection_radius = circle.get("debug_detection_radius") or 40
    screen_x, screen_y = world_to_screen(circle["x"], circle["y"], camera_x, camera_y)
    end_x = screen_x + detection_radius * math.cos(math.radians(circle["angle"]))
    end_y = screen_y + detection_radius * math.sin(math.radians(circle["angle"]))
    pygame.draw.line(surface, color, (int(screen_x), int(screen_y)), (int(end_x), int(end_y)), 2)

def draw_minkowski_circles(surface, circle, camera_x, camera_y, color=(255, 0, 0)):
    """Draw predicted Minkowski circles for detected obstacles."""
    minkowski_circles = circle.get("debug_minkowski_circles")
    if not minkowski_circles:
        return

    for entry in minkowski_circles:
        center_x, center_y = entry["center"]
        radius = entry["radius"]
        screen_x, screen_y = world_to_screen(center_x, center_y, camera_x, camera_y)
        pygame.draw.circle(surface, color, (int(screen_x), int(screen_y)), int(radius), 1)

def draw_reverse_cooldown(surface, circle, camera_x, camera_y, font, color=(160, 0, 160)):
    """Draw reverse cooldown timer for a circle if active."""
    cooldown = circle.get("reverse_cooldown", 0.0)
    if cooldown <= 0.0:
        return

    screen_x, screen_y = world_to_screen(circle["x"], circle["y"], camera_x, camera_y)
    text = f"CD {cooldown:.1f}s x{circle.get('reverse_extreme_count', 0)}"
    label_surface = font.render(text, True, color)
    label_rect = label_surface.get_rect(center=(int(screen_x), int(screen_y + 35)))
    surface.blit(label_surface, label_rect)

def draw_orca_debug(surface, circle, camera_x, camera_y, font,
                    pref_color=(0, 120, 255), new_color=(255, 80, 0),
                    radius_color=(80, 160, 255)):
    """Draw ORCA debug visuals: neighbor range and preferred/selected velocities."""
    neighbor_dist = circle.get("debug_orca_neighbor_dist")
    pref_vel = circle.get("debug_orca_pref_velocity")
    new_vel = circle.get("debug_orca_new_velocity")
    scale = circle.get("debug_orca_velocity_scale", 0.5)
    time_horizon = circle.get("debug_orca_time_horizon")

    if neighbor_dist is None and pref_vel is None and new_vel is None and time_horizon is None:
        return

    screen_x, screen_y = world_to_screen(circle["x"], circle["y"], camera_x, camera_y)

    if neighbor_dist:
        pygame.draw.circle(surface, radius_color, (int(screen_x), int(screen_y)), int(neighbor_dist), 1)

    if pref_vel is not None:
        end_x = screen_x + pref_vel[0] * scale
        end_y = screen_y + pref_vel[1] * scale
        pygame.draw.line(surface, pref_color, (int(screen_x), int(screen_y)), (int(end_x), int(end_y)), 2)
        pygame.draw.circle(surface, pref_color, (int(end_x), int(end_y)), 3, 0)

    if new_vel is not None:
        end_x = screen_x + new_vel[0] * scale
        end_y = screen_y + new_vel[1] * scale
        pygame.draw.line(surface, new_color, (int(screen_x), int(screen_y)), (int(end_x), int(end_y)), 2)
        pygame.draw.circle(surface, new_color, (int(end_x), int(end_y)), 3, 0)

    if time_horizon is not None and font is not None:
        text = f"TH {time_horizon:.2f}s"
        label_surface = font.render(text, True, (40, 40, 40))
        label_rect = label_surface.get_rect(center=(int(screen_x), int(screen_y + 55)))
        surface.blit(label_surface, label_rect)

def draw_orca_constraints(surface, circle, camera_x, camera_y,
                          line_color=(60, 60, 60), normal_color=(120, 0, 200, 160)):
    """Draw ORCA constraint lines and approximate half-planes in velocity space."""
    orca_lines = circle.get("debug_orca_lines")
    if not orca_lines:
        return

    scale = circle.get("debug_orca_velocity_scale", 0.5)
    screen_x, screen_y = world_to_screen(circle["x"], circle["y"], camera_x, camera_y)

    overlay = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)

    for line in orca_lines:
        point = line.get("point")
        direction = line.get("direction")
        if not point or not direction:
            continue

        normal_x, normal_y = direction
        normal_len = math.hypot(normal_x, normal_y)
        if normal_len < 1e-6:
            continue

        normal_x /= normal_len
        normal_y /= normal_len
        tangent_x = -normal_y
        tangent_y = normal_x

        anchor_x = screen_x + point[0] * scale
        anchor_y = screen_y + point[1] * scale

        line_half_len = 120
        half_plane_len = 140

        p1 = (anchor_x + tangent_x * line_half_len, anchor_y + tangent_y * line_half_len)
        p2 = (anchor_x - tangent_x * line_half_len, anchor_y - tangent_y * line_half_len)
        p3 = (p2[0] + normal_x * half_plane_len, p2[1] + normal_y * half_plane_len)
        p4 = (p1[0] + normal_x * half_plane_len, p1[1] + normal_y * half_plane_len)

        pygame.draw.polygon(overlay, normal_color, [p1, p2, p3, p4])
        pygame.draw.line(surface, line_color, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), 2)

        arrow_end = (anchor_x + normal_x * 20, anchor_y + normal_y * 20)
        pygame.draw.line(surface, (120, 0, 200), (int(anchor_x), int(anchor_y)), (int(arrow_end[0]), int(arrow_end[1])), 2)

    surface.blit(overlay, (0, 0))

def _clip_segment_to_circle_vis(x1, y1, x2, y2, cx, cy, r):
    """Return the portion of segment (x1,y1)→(x2,y2) inside circle (cx,cy,r).
    Returns (px1, py1, px2, py2) or None."""
    dx, dy = x2 - x1, y2 - y1
    fx, fy = x1 - cx, y1 - cy
    a = dx * dx + dy * dy
    if a < 1e-12:
        if fx * fx + fy * fy <= r * r:
            return (x1, y1, x2, y2)
        return None
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - r * r
    disc = b * b - 4.0 * a * c
    if disc < 0:
        return None
    sd = math.sqrt(disc)
    t1 = (-b - sd) / (2.0 * a)
    t2 = (-b + sd) / (2.0 * a)
    t_lo = max(0.0, t1)
    t_hi = min(1.0, t2)
    if t_lo > t_hi + 1e-9:
        return None
    return (x1 + t_lo * dx, y1 + t_lo * dy,
            x1 + t_hi * dx, y1 + t_hi * dy)


def draw_obstacle_detection(surface, circle, camera_x, camera_y, obstacles_list):
    """Draw obstacle detection zones using 4-segments + 4-circles decomposition.
    
    Visualises:
      - Rounded-rectangle outline (light blue)
      - Detection radius circle (dashed cyan)
      - Clipped edge segments within detection radius (green lines)
      - Corner circles within detection radius (yellow circles)
      - Blocked angle sector (red wedge) — matching the algorithm
    """
    if not obstacles_list:
        return

    config = AVOIDANCE_CONFIG.get("predictive_scan", {})
    expand = circle["radius"] + config.get("collision_padding", 2.0)

    # Use the detection radius already computed by the avoidance algorithm
    det_r = circle.get("debug_detection_radius")
    if not det_r:
        return

    ax, ay = circle["x"], circle["y"]
    scx, scy = world_to_screen(ax, ay, camera_x, camera_y)

    overlay = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)

    # Draw detection radius circle
    pygame.draw.circle(overlay, (0, 180, 255, 80), (int(scx), int(scy)), int(det_r), 1)

    for obs in obstacles_list:
        hw, hh = obs["w"] / 2, obs["h"] / 2

        # Quick distance check (local space)
        neg_rad = math.radians(-obs["angle"])
        cos_n = math.cos(neg_rad)
        sin_n = math.sin(neg_rad)
        ddx = ax - obs["cx"]
        ddy = ay - obs["cy"]
        lx = ddx * cos_n - ddy * sin_n
        ly = ddx * sin_n + ddy * cos_n

        cp_x = max(-hw, min(hw, lx))
        cp_y = max(-hh, min(hh, ly))
        if math.hypot(lx - cp_x, ly - cp_y) > det_r + expand:
            continue

        e = expand
        obs_rad = math.radians(obs["angle"])
        cos_o = math.cos(obs_rad)
        sin_o = math.sin(obs_rad)

        def to_world(plx, ply):
            return (obs["cx"] + plx * cos_o - ply * sin_o,
                    obs["cy"] + plx * sin_o + ply * cos_o)

        def to_screen_local(plx, ply):
            wx, wy = to_world(plx, ply)
            return world_to_screen(wx, wy, camera_x, camera_y)

        # --- Draw rounded rectangle outline (Minkowski sum) ---
        arc_segments = 8
        outline_pts = []
        corner_signs = [(1, 1), (-1, 1), (-1, -1), (1, -1)]
        arc_start_angles = [0, 90, 180, 270]
        for ci in range(4):
            sx_s, sy_s = corner_signs[ci]
            cx_l = sx_s * hw
            cy_l = sy_s * hh
            sd = arc_start_angles[ci]
            for j in range(arc_segments + 1):
                a = math.radians(sd + 90.0 * j / arc_segments)
                px = cx_l + e * math.cos(a)
                py = cy_l + e * math.sin(a)
                outline_pts.append(to_screen_local(px, py))
        if len(outline_pts) >= 3:
            int_pts = [(int(x), int(y)) for x, y in outline_pts]
            pygame.draw.polygon(overlay, (0, 180, 255, 30), int_pts)
            pygame.draw.polygon(overlay, (0, 180, 255, 100), int_pts, 1)

        # --- 4 edge segments, clipped to detection circle ---
        segs_local = [
            ((-hw, -hh - e), ( hw, -hh - e)),   # bottom
            (( hw + e, -hh), ( hw + e,  hh)),    # right
            (( hw,  hh + e), (-hw,  hh + e)),    # top
            ((-hw - e,  hh), (-hw - e, -hh)),    # left
        ]
        angles = []
        for (sl1, sl2) in segs_local:
            w1 = to_world(*sl1)
            w2 = to_world(*sl2)
            clipped = _clip_segment_to_circle_vis(
                w1[0], w1[1], w2[0], w2[1], ax, ay, det_r)
            if clipped is None:
                continue
            # Draw clipped segment (bright green)
            cs1 = world_to_screen(clipped[0], clipped[1], camera_x, camera_y)
            cs2 = world_to_screen(clipped[2], clipped[3], camera_x, camera_y)
            pygame.draw.line(overlay, (0, 255, 100, 200),
                             (int(cs1[0]), int(cs1[1])),
                             (int(cs2[0]), int(cs2[1])), 3)
            # Draw endpoint dots
            pygame.draw.circle(overlay, (0, 255, 100, 255), (int(cs1[0]), int(cs1[1])), 3, 0)
            pygame.draw.circle(overlay, (0, 255, 100, 255), (int(cs2[0]), int(cs2[1])), 3, 0)
            # Collect angles
            a1 = math.degrees(math.atan2(clipped[1] - ay, clipped[0] - ax))
            a2 = math.degrees(math.atan2(clipped[3] - ay, clipped[2] - ax))
            angles.append(((a1 + 180.0) % 360.0) - 180.0)
            angles.append(((a2 + 180.0) % 360.0) - 180.0)

        # --- 4 corner circles (static: no time sampling, single calculation) ---
        prediction_time = config.get("prediction_time", 0.5)
        self_speed = circle["speed"] * config.get("speed_factor", 1.0)
        self_distance = self_speed * prediction_time
        corner_blocked = []

        corners = [(hw, hh), (-hw, hh), (-hw, -hh), (hw, -hh)]
        for clx, cly in corners:
            wc = to_world(clx, cly)
            sc = world_to_screen(wc[0], wc[1], camera_x, camera_y)
            p0x = wc[0] - ax
            p0y = wc[1] - ay
            base_dist = math.hypot(p0x, p0y)
            if base_dist - e > det_r:
                continue  # corner circle fully outside detection
            # Draw corner circle (yellow)
            pygame.draw.circle(overlay, (255, 220, 0, 120), (int(sc[0]), int(sc[1])), int(e), 1)
            if base_dist <= e or base_dist < 1e-6:
                continue
            # Static obstacle: compute once with max self_distance
            if self_distance <= 1e-6:
                continue
            if base_dist > self_distance + e:
                continue
            if base_dist < abs(self_distance - e):
                continue
            cos_half = (base_dist * base_dist + self_distance * self_distance - e * e) / (2.0 * base_dist * self_distance)
            cos_half = max(-1.0, min(1.0, cos_half))
            half_angle = math.degrees(math.acos(cos_half))
            center_angle = math.degrees(math.atan2(p0y, p0x))
            center_angle = ((center_angle + 180.0) % 360.0) - 180.0
            cs = ((center_angle - half_angle + 180.0) % 360.0) - 180.0
            ce = ((center_angle + half_angle + 180.0) % 360.0) - 180.0
            if cs <= ce:
                corner_blocked.append((cs, ce))
            else:
                corner_blocked.append((cs, 180.0))
                corner_blocked.append((-180.0, ce))

        # --- Combine edge angles + corner blocked intervals → draw blocked sector ---
        # Edge segments → largest-gap → blocked interval
        edge_blocked = []
        if len(angles) >= 2:
            angles.sort()
            n = len(angles)
            max_gap = -1.0
            max_gap_idx = 0
            for i in range(n):
                g = (angles[(i + 1) % n] - angles[i]) if i < n - 1 else (angles[0] + 360.0) - angles[n - 1]
                if g > max_gap:
                    max_gap = g
                    max_gap_idx = i
            b_start = angles[(max_gap_idx + 1) % n]
            b_end = angles[max_gap_idx]
            if b_start <= b_end:
                edge_blocked.append((b_start, b_end))
            else:
                edge_blocked.append((b_start, 180.0))
                edge_blocked.append((-180.0, b_end))

        # Merge edge + corner blocked intervals
        all_blocked = edge_blocked + corner_blocked
        if not all_blocked:
            continue
        all_blocked.sort()
        merged = [all_blocked[0]]
        for s, e_val in all_blocked[1:]:
            ls, le = merged[-1]
            if s <= le + 1e-6:
                merged[-1] = (ls, max(le, e_val))
            else:
                merged.append((s, e_val))

        # Draw all merged blocked sectors
        wedge_radius = min(det_r, 120)
        for b_start, b_end in merged:
            pts = [(int(scx), int(scy))]
            a = b_start
            while a <= b_end:
                ex = scx + wedge_radius * math.cos(math.radians(a))
                ey = scy + wedge_radius * math.sin(math.radians(a))
                pts.append((int(ex), int(ey)))
                a += 3.0
            ex = scx + wedge_radius * math.cos(math.radians(b_end))
            ey = scy + wedge_radius * math.sin(math.radians(b_end))
            pts.append((int(ex), int(ey)))
            if len(pts) >= 3:
                pygame.draw.polygon(overlay, (255, 50, 50, 45), pts)
                pygame.draw.lines(overlay, (255, 50, 50, 120), True, pts, 1)

    surface.blit(overlay, (0, 0))


def draw_orca_stats(surface, circle, camera_x, camera_y, font, color=(30, 30, 30)):
    """Draw ORCA diagnostic stats for stuck investigation."""
    stats = circle.get("debug_orca_stats")
    if not stats or font is None:
        return

    screen_x, screen_y = world_to_screen(circle["x"], circle["y"], camera_x, camera_y)
    lines = [
        f"N {stats.get('neighbors', 0)} / C {stats.get('constraints', 0)}",
        f"V {stats.get('new_speed', 0.0):.1f} / P {stats.get('pref_speed', 0.0):.1f}",
        f"COL {int(stats.get('collision_pred', False))}",
        f"Dmin {stats.get('nearest_dist', 0.0):.1f}",
        f"Tdist {stats.get('target_dist', 0.0):.1f}",
        f"Drop {stats.get('filtered_neighbors', 0)}",
    ]

    for i, text in enumerate(lines):
        label_surface = font.render(text, True, color)
        label_rect = label_surface.get_rect(center=(int(screen_x), int(screen_y + 75 + i * 16)))
        surface.blit(label_surface, label_rect)

def world_to_screen(world_x, world_y, camera_x, camera_y):
    """Convert world coordinates to screen coordinates"""
    screen_x = world_x - camera_x
    screen_y = world_y - camera_y
    return screen_x, screen_y

def screen_to_world(screen_x, screen_y, camera_x, camera_y):
    """Convert screen coordinates to world coordinates"""
    world_x = screen_x + camera_x
    world_y = screen_y + camera_y
    return world_x, world_y

def get_circle_at_position(world_x, world_y, circles, radius=25):
    """Check if there's a circle at the given world position"""
    for circle in circles:
        distance = ((circle["x"] - world_x) ** 2 + (circle["y"] - world_y) ** 2) ** 0.5
        if distance <= radius:
            return circle
    return None

def get_rotated_rect_corners(cx, cy, w, h, angle_deg):
    """Get the 4 corners of a rotated rectangle in world coordinates."""
    angle_rad = math.radians(angle_deg)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    hw, hh = w / 2, h / 2
    corners = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
    return [(cx + x * cos_a - y * sin_a, cy + x * sin_a + y * cos_a) for x, y in corners]

def point_in_rotated_rect(px, py, cx, cy, w, h, angle_deg):
    """Check if a point is inside a rotated rectangle."""
    angle_rad = math.radians(-angle_deg)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    dx = px - cx
    dy = py - cy
    local_x = dx * cos_a - dy * sin_a
    local_y = dx * sin_a + dy * cos_a
    return abs(local_x) <= w / 2 and abs(local_y) <= h / 2

def get_obstacle_at_position(world_x, world_y):
    """Find the obstacle at the given world position (newest first)."""
    for obs in reversed(obstacles):
        if point_in_rotated_rect(world_x, world_y, obs["cx"], obs["cy"], obs["w"], obs["h"], obs["angle"]):
            return obs
    return None

def circle_overlaps_obstacle(cx, cy, radius, obs):
    """Check if a circle overlaps with a rotated rectangle obstacle."""
    angle_rad = math.radians(-obs["angle"])
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    dx = cx - obs["cx"]
    dy = cy - obs["cy"]
    local_x = dx * cos_a - dy * sin_a
    local_y = dx * sin_a + dy * cos_a
    hw, hh = obs["w"] / 2, obs["h"] / 2
    closest_x = max(-hw, min(hw, local_x))
    closest_y = max(-hh, min(hh, local_y))
    diff_x = local_x - closest_x
    diff_y = local_y - closest_y
    return (diff_x * diff_x + diff_y * diff_y) < radius * radius

def get_obstacle_push(circle, obstacles_list):
    """Calculate push vector to resolve circle-obstacle collisions (hard constraint).
    
    Transforms the circle into each obstacle's local space, finds the closest
    point on the rectangle, and pushes the circle out if overlapping.
    
    Uses radius + collision_padding so the circle centre always stays outside
    the Minkowski sum used by the avoidance algorithm.
    """
    total_px, total_py = 0.0, 0.0
    collision_padding = AVOIDANCE_CONFIG.get("predictive_scan", {}).get("collision_padding", 2.0)
    effective_radius = circle["radius"] + collision_padding
    
    for obs in obstacles_list:
        angle_rad = math.radians(-obs["angle"])
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        dx = circle["x"] - obs["cx"]
        dy = circle["y"] - obs["cy"]
        local_x = dx * cos_a - dy * sin_a
        local_y = dx * sin_a + dy * cos_a
        
        hw, hh = obs["w"] / 2, obs["h"] / 2
        closest_x = max(-hw, min(hw, local_x))
        closest_y = max(-hh, min(hh, local_y))
        
        diff_x = local_x - closest_x
        diff_y = local_y - closest_y
        dist_sq = diff_x * diff_x + diff_y * diff_y
        
        if dist_sq >= effective_radius * effective_radius:
            continue
        
        dist = math.sqrt(dist_sq) if dist_sq > 1e-12 else 0.0
        
        if dist > 1e-6:
            # Circle center is outside the rectangle surface
            overlap = effective_radius - dist
            nx = diff_x / dist
            ny = diff_y / dist
        else:
            # Circle center is inside the rectangle - push out via shortest axis
            push_options = [
                (hw - local_x, 1.0, 0.0),
                (hw + local_x, -1.0, 0.0),
                (hh - local_y, 0.0, 1.0),
                (hh + local_y, 0.0, -1.0),
            ]
            min_push, nx, ny = min(push_options, key=lambda t: t[0])
            overlap = min_push + effective_radius
        
        # Transform push direction back to world space
        angle_back = math.radians(obs["angle"])
        cos_b = math.cos(angle_back)
        sin_b = math.sin(angle_back)
        world_nx = nx * cos_b - ny * sin_b
        world_ny = nx * sin_b + ny * cos_b
        
        total_px += world_nx * overlap
        total_py += world_ny * overlap
    
    return total_px, total_py

def draw_obstacles(surface, camera_x, camera_y):
    """Draw all obstacles as rotated rectangles."""
    if not obstacles:
        return
    overlay = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)
    for obs in obstacles:
        corners = get_rotated_rect_corners(obs["cx"], obs["cy"], obs["w"], obs["h"], obs["angle"])
        screen_corners = [world_to_screen(x, y, camera_x, camera_y) for x, y in corners]
        int_corners = [(int(x), int(y)) for x, y in screen_corners]
        pygame.draw.polygon(overlay, (100, 100, 100, 180), int_corners)
        pygame.draw.polygon(overlay, (40, 40, 40, 255), int_corners, 2)
        # Draw rotation indicator (small line from center pointing "up" in local space)
        center_sx, center_sy = world_to_screen(obs["cx"], obs["cy"], camera_x, camera_y)
        ind_len = min(obs["w"], obs["h"]) * 0.3
        angle_rad = math.radians(obs["angle"])
        ind_x = center_sx - math.sin(angle_rad) * ind_len
        ind_y = center_sy + math.cos(angle_rad) * ind_len
        pygame.draw.line(overlay, (200, 60, 60, 200), (int(center_sx), int(center_sy)), (int(ind_x), int(ind_y)), 2)
    surface.blit(overlay, (0, 0))

# Main game loop
running = True
while running:
    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        
        # Mouse handling
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # Left click
                is_dragging = True
                drag_start_pos = pygame.mouse.get_pos()
            
            elif event.button == 3:  # Right click
                if obstacle_placement_mode:
                    # Rotate obstacle under cursor
                    mouse_x, mouse_y = pygame.mouse.get_pos()
                    world_x, world_y = screen_to_world(mouse_x, mouse_y, camera_x, camera_y)
                    obs = get_obstacle_at_position(world_x, world_y)
                    if obs:
                        obs["angle"] = (obs["angle"] + 15) % 360
                elif selected_circles:
                    # Move selected circles to mouse position (Precise Arrival)
                    
                    # Clear global target marking (cancel Z-target)
                    global_target = None
                    
                    mouse_x, mouse_y = pygame.mouse.get_pos()
                    world_x, world_y = screen_to_world(mouse_x, mouse_y, camera_x, camera_y)
                    
                    for circle in selected_circles:
                        circle["target_x"] = world_x
                        circle["target_y"] = world_y
                        circle["target_radius"] = 0.0
                        circle["target_ref"] = None
                        circle["arrival_threshold"] = 5  # Precise arrival
                        
        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:  # Left click release
                is_dragging = False
                drag_end_pos = pygame.mouse.get_pos()
                
                # Check distance to distinguish click vs drag
                dist = math.hypot(drag_end_pos[0] - drag_start_pos[0], drag_end_pos[1] - drag_start_pos[1])
                
                if obstacle_placement_mode:
                    # Obstacle placement mode
                    if dist >= 10:  # Drag to create obstacle (minimum size)
                        start_world = screen_to_world(drag_start_pos[0], drag_start_pos[1], camera_x, camera_y)
                        end_world = screen_to_world(drag_end_pos[0], drag_end_pos[1], camera_x, camera_y)
                        w = abs(end_world[0] - start_world[0])
                        h = abs(end_world[1] - start_world[1])
                        if w > 5 and h > 5:
                            cx = (start_world[0] + end_world[0]) / 2
                            cy = (start_world[1] + end_world[1]) / 2
                            obstacles.append({"cx": cx, "cy": cy, "w": w, "h": h, "angle": 0})
                    else:
                        # Click on obstacle to delete it
                        mouse_x, mouse_y = drag_end_pos
                        world_x, world_y = screen_to_world(mouse_x, mouse_y, camera_x, camera_y)
                        obs = get_obstacle_at_position(world_x, world_y)
                        if obs:
                            obstacles.remove(obs)
                else:
                    # Normal selection mode
                    if dist < 5:  # Click
                        mouse_x, mouse_y = drag_end_pos
                        world_x, world_y = screen_to_world(mouse_x, mouse_y, camera_x, camera_y)
                        clicked_circle = get_circle_at_position(world_x, world_y, circles)
                        
                        if clicked_circle:
                            selected_circles = [clicked_circle]  # Single select
                        else:
                             selected_circles = [] # Deselect if clicking empty space

                    else:
                        # Drag selection - Box select
                        start_world = screen_to_world(drag_start_pos[0], drag_start_pos[1], camera_x, camera_y)
                        end_world = screen_to_world(drag_end_pos[0], drag_end_pos[1], camera_x, camera_y)
                        
                        left = min(start_world[0], end_world[0])
                        right = max(start_world[0], end_world[0])
                        top = min(start_world[1], end_world[1])
                        bottom = max(start_world[1], end_world[1])
                        
                        selected_circles = []
                        for circle in circles:
                            if left < circle["x"] < right and top < circle["y"] < bottom:
                                selected_circles.append(circle)
        
        # Keyboard shortcuts
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                # Place new circle at mouse position
                mouse_x, mouse_y = pygame.mouse.get_pos()
                world_x, world_y = screen_to_world(mouse_x, mouse_y, camera_x, camera_y)
                
                circle_counter += 1
                new_circle = {
                    "x": world_x,
                    "y": world_y,
                    "label": f"Circle{circle_counter}",
                    "target_x": None,
                    "target_y": None,
                    "angle": 0,
                    "speed": 100,
                    "turn_speed": 180,
                    "radius": 25,
                    "arrival_threshold": 5
                }
                
                can_place = True
                for other in circles:
                    if check_collision(new_circle["x"], new_circle["y"], new_circle["radius"],
                                     other["x"], other["y"], other["radius"]):
                        can_place = False
                        break
                if can_place:
                    for obs in obstacles:
                        if circle_overlaps_obstacle(new_circle["x"], new_circle["y"], new_circle["radius"], obs):
                            can_place = False
                            break
                
                if can_place:
                    circles.append(new_circle)

            if event.key == pygame.K_z:
                # Set target to an existing circle
                mouse_x, mouse_y = pygame.mouse.get_pos()
                world_x, world_y = screen_to_world(mouse_x, mouse_y, camera_x, camera_y)
                
                target_unit = get_circle_at_position(world_x, world_y, circles)
                
                if target_unit:
                    global_target = (target_unit["x"], target_unit["y"])
                    print(f"Target set to {target_unit['label']}")
                    
                    if selected_circles:
                        for circle in selected_circles:
                            if circle is target_unit:
                                continue
                            circle["target_x"] = target_unit["x"]
                            circle["target_y"] = target_unit["y"]
                            circle["target_radius"] = target_unit["radius"]
                            circle["target_ref"] = target_unit
                            # Arrive when touching (Radius + Radius + Margin)
                            # 这样可以实现多个单位围住一个目标
                            circle["arrival_threshold"] = circle["radius"] + target_unit["radius"] + 5
                
            if event.key == pygame.K_TAB:
                # Switch to next avoidance algorithm
                avoidance_manager.next_algorithm()
                current_algo = avoidance_manager.get_algorithm()
                print(f"Switched to: {current_algo.get_name() if current_algo else 'None'}")
            if event.key == pygame.K_v:
                # Toggle debug visuals
                avoidance_config.DEBUG_ENABLED = not avoidance_config.DEBUG_ENABLED
                print(f"Debug enabled: {avoidance_config.DEBUG_ENABLED}")
            if event.key == pygame.K_F3:
                # Toggle obstacle placement mode
                obstacle_placement_mode = not obstacle_placement_mode
                print(f"Obstacle placement: {'ON' if obstacle_placement_mode else 'OFF'}")
    
    # Keyboard input - WASD camera movement
    keys = pygame.key.get_pressed()
    if keys[pygame.K_w]:
        camera_y -= camera_speed
    if keys[pygame.K_s]:
        camera_y += camera_speed
    if keys[pygame.K_a]:
        camera_x -= camera_speed
    if keys[pygame.K_d]:
        camera_x += camera_speed
    
    # Update circle movements
    delta_time = clock.get_time() / 1000.0  # Convert to seconds
    current_algorithm = avoidance_manager.get_algorithm()
    # Pass obstacle data to avoidance algorithm (Minkowski blocking, no virtual circles)
    if current_algorithm:
        current_algorithm.set_obstacles(obstacles)
    for circle in circles:
        move_circle_towards_target(circle, delta_time, circles, current_algorithm)
    # Apply obstacle hard collision for ALL circles (even stationary ones)
    if obstacles:
        for circle in circles:
            obs_px, obs_py = get_obstacle_push(circle, obstacles)
            circle["x"] += obs_px
            circle["y"] += obs_py
    
    # Clear screen
    screen.fill(WHITE)
    
    # Draw grid
    draw_grid(screen, camera_x, camera_y)

    # Draw obstacles
    draw_obstacles(screen, camera_x, camera_y)

    # Draw FPS
    fps = clock.get_fps()
    fps_surface = small_font.render(f"FPS: {fps:.1f}", True, (0, 0, 0))
    fps_rect = fps_surface.get_rect(topright=(WINDOW_WIDTH - 10, 10))
    screen.blit(fps_surface, fps_rect)
    
    # Draw all circles
    for circle in circles:
        # Convert to screen coordinates
        screen_x, screen_y = world_to_screen(circle["x"], circle["y"], camera_x, camera_y)
        
        # Only draw circles within screen bounds (optimization)
        if -50 < screen_x < WINDOW_WIDTH + 50 and -50 < screen_y < WINDOW_HEIGHT + 50:
            # Determine color based on selection
            circle_color = GREEN if circle in selected_circles else BLUE
            
            # Draw circle
            pygame.draw.circle(screen, circle_color, (int(screen_x), int(screen_y)), circle["radius"])
            pygame.draw.circle(screen, BLACK, (int(screen_x), int(screen_y)), circle["radius"], 2)
            
            # Draw direction indicator (arrow)
            import math
            arrow_length = 20
            arrow_x = screen_x + arrow_length * math.cos(math.radians(circle["angle"]))
            arrow_y = screen_y + arrow_length * math.sin(math.radians(circle["angle"]))
            pygame.draw.line(screen, RED, (int(screen_x), int(screen_y)), 
                           (int(arrow_x), int(arrow_y)), 3)
            
            # Draw label
            label_surface = font.render(circle["label"], True, BLACK)
            label_rect = label_surface.get_rect(center=(int(screen_x), int(screen_y - 35)))
            screen.blit(label_surface, label_rect)
            
            # Draw target position if exists (individual target)
            if circle["target_x"] is not None and circle["target_y"] is not None:
                target_screen_x, target_screen_y = world_to_screen(
                    circle["target_x"], circle["target_y"], camera_x, camera_y
                )
                # Draw small x marker at target
                pygame.draw.line(screen, (200, 100, 100), 
                               (int(target_screen_x - 5), int(target_screen_y - 5)),
                               (int(target_screen_x + 5), int(target_screen_y + 5)), 1)
                pygame.draw.line(screen, (200, 100, 100),
                               (int(target_screen_x + 5), int(target_screen_y - 5)),
                               (int(target_screen_x - 5), int(target_screen_y + 5)), 1)

                reachable_target = circle.get("debug_reachable_target")
                if avoidance_config.DEBUG_ENABLED and reachable_target:
                    reach_screen_x, reach_screen_y = world_to_screen(
                        reachable_target[0], reachable_target[1], camera_x, camera_y
                    )
                    pygame.draw.circle(screen, (255, 140, 0), (int(reach_screen_x), int(reach_screen_y)), 6, 2)
                    pygame.draw.line(screen, (255, 140, 0),
                                   (int(reach_screen_x - 4), int(reach_screen_y)),
                                   (int(reach_screen_x + 4), int(reach_screen_y)), 2)
                    pygame.draw.line(screen, (255, 140, 0),
                                   (int(reach_screen_x), int(reach_screen_y - 4)),
                                   (int(reach_screen_x), int(reach_screen_y + 4)), 2)

            # Draw feasible angles and velocity direction for selected circles
            if avoidance_config.DEBUG_ENABLED and circle in selected_circles:
                draw_allowed_angles(screen, circle, camera_x, camera_y)
                draw_velocity_direction(screen, circle, camera_x, camera_y)
                draw_minkowski_circles(screen, circle, camera_x, camera_y)
                draw_reverse_cooldown(screen, circle, camera_x, camera_y, small_font)
                draw_orca_debug(screen, circle, camera_x, camera_y, small_font)
                draw_orca_constraints(screen, circle, camera_x, camera_y)
                draw_orca_stats(screen, circle, camera_x, camera_y, small_font)
                draw_obstacle_detection(screen, circle, camera_x, camera_y, obstacles)

    # Draw global target (Z key)
    if global_target:
        gt_screen_x, gt_screen_y = world_to_screen(global_target[0], global_target[1], camera_x, camera_y)
        # Draw a distinctive marker (Magenta X with Circle)
        pygame.draw.circle(screen, (255, 0, 255), (int(gt_screen_x), int(gt_screen_y)), 15, 2)
        pygame.draw.line(screen, (255, 0, 255), (int(gt_screen_x)-20, int(gt_screen_y)-20), (int(gt_screen_x)+20, int(gt_screen_y)+20), 3)
        pygame.draw.line(screen, (255, 0, 255), (int(gt_screen_x)+20, int(gt_screen_y)-20), (int(gt_screen_x)-20, int(gt_screen_y)+20), 3)
        
        # Label for target
        target_label = font.render("Target (Z)", True, (255, 0, 255))
        screen.blit(target_label, (int(gt_screen_x) + 20, int(gt_screen_y) - 20))

    # Draw drag selection box / obstacle preview
    if is_dragging:
        mouse_x, mouse_y = pygame.mouse.get_pos()
        rect_left = min(drag_start_pos[0], mouse_x)
        rect_top = min(drag_start_pos[1], mouse_y)
        rect_width = abs(drag_start_pos[0] - mouse_x)
        rect_height = abs(drag_start_pos[1] - mouse_y)
        
        if obstacle_placement_mode:
            # Orange obstacle preview
            if rect_width > 0 and rect_height > 0:
                selection_rect = pygame.Rect(rect_left, rect_top, rect_width, rect_height)
                pygame.draw.rect(screen, (200, 120, 50), selection_rect, 2)
                s = pygame.Surface((rect_width, rect_height), pygame.SRCALPHA)
                s.fill((200, 120, 50, 60))
                screen.blit(s, (rect_left, rect_top))
        else:
            # Green selection box
            selection_rect = pygame.Rect(rect_left, rect_top, rect_width, rect_height)
            pygame.draw.rect(screen, GREEN, selection_rect, 1)
            if rect_width > 0 and rect_height > 0:
                s = pygame.Surface((rect_width, rect_height))
                s.set_alpha(50)
                s.fill(GREEN)
                screen.blit(s, (rect_left, rect_top))

    # Draw info text
    info_text = [
        "Controls:",
        " [Q] Place Unit",
        " [Left Drag] Select Units",
        " [Right Click] Move Here (Precise)",
        " [Z] Attack/Follow Unit (Touch)",
        " [TAB] Switch Algorithm",
        " [V] Toggle Debug",
        " [F3] Toggle Obstacle Mode",
        " [WASD] Camera",
        "",
        f"Camera: ({camera_x}, {camera_y})",
        f"Circles: {len(circles)}",
        f"Obstacles: {len(obstacles)}",
        f"Selected: {len(selected_circles)}",
        f"Algorithm: {avoidance_manager.get_algorithm().get_name() if avoidance_manager.get_algorithm() else 'None'}",
        f"Debug: {avoidance_config.DEBUG_ENABLED}",
        f"Obstacle Mode: {'ON' if obstacle_placement_mode else 'OFF'}",
    ]
    
    y_offset = 10
    for text in info_text:
        text_surface = font.render(text, True, BLACK)
        text_rect = pygame.Rect(10, y_offset, 300, 30)
        pygame.draw.rect(screen, (255, 255, 255, 200), text_rect)
        screen.blit(text_surface, (15, y_offset))
        y_offset += 25
    
    # Update display
    pygame.display.flip()
    
    # Control frame rate
    clock.tick(60)

# Exit
pygame.quit()
sys.exit()
