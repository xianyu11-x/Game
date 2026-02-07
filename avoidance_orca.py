"""
ORCA (Optimal Reciprocal Collision Avoidance) algorithm
Provides smooth and efficient collision avoidance using velocity obstacles
"""

from avoidance_base import AvoidanceBase
from avoidance_config import normalize_angle, angular_distance
import math

class ORCAAvoidance(AvoidanceBase):
    """ORCA avoidance - optimal reciprocal collision avoidance"""
    
    def __init__(self):
        super().__init__()
        self.name = "ORCA"
        self.description = "Optimal Reciprocal Collision Avoidance"
        # 平衡参数 - 既避免碰撞又能突破包围
        self.time_horizon = 2.0  # 适中的预测时间
        self.max_neighbors = 8  # 考虑更多邻居以避免碰撞
        self.neighbor_dist = 120.0  # 适中的检测范围
        self.max_speed = 200.0  # Maximum speed for agents
        self.min_speed = 15.0  # 适中的最小速度
        self.escape_mode_speed = 50.0  # 逃离模式的速度
        self.circle_id_map = {}  # 用于跟踪每个圆形的状态
        # 方向惩罚与抑制（与 Predictive Scan 对齐的策略）
        self.turn_penalty_weight = 0.0
        self.turn_penalty_threshold_deg = 45.0
        self.opposite_suppression_enabled = True
        self.opposite_angle_threshold_deg = 30.0
        # Reverse cooldown（避免频繁反向）
        self.reverse_cooldown_enabled = True
        self.reverse_cooldown_base = 0.4
        self.reverse_cooldown_max = 5.0
        self.reverse_cooldown_growth = 2.0
        self.reverse_cooldown_decay = 0.8
        self.reverse_extreme_threshold_deg = 120.0
        # Concave detection（卡住时动态加大预测时间）
        self.concave_detection_enabled = True
        self.concave_progress_epsilon = 1.5
        self.concave_time_growth = 0.6
        self.concave_time_max = 4.0
        self.concave_decay = 0.8
        # Velocity inertia (smooth oscillation)
        self.inertia_enabled = True
        self.inertia_weight = 0.7  # 0~1, higher keeps more of previous velocity
        self.inertia_min_speed = 5.0
    
    def _calculate_preferred_velocity(self, circle, target_x, target_y):
        """Calculate the preferred velocity towards the target"""
        dx = target_x - circle["x"]
        dy = target_y - circle["y"]
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.1:
            return (0, 0)
        
        # Normalize and scale by desired speed
        speed = min(circle["speed"], self.max_speed)
        return (dx / distance * speed, dy / distance * speed)
    
    def _get_neighbors(self, circle, all_circles):
        """Get nearby neighbors sorted by urgency (distance and relative velocity)"""
        neighbors = []
        circle_vel = self._get_velocity(circle)
        target_ref = circle.get("target_ref")
        
        for other in all_circles:
            if other is circle:
                continue
            if target_ref is not None and other is target_ref:
                continue
            
            dx = other["x"] - circle["x"]
            dy = other["y"] - circle["y"]
            dist_sq = dx**2 + dy**2
            dist = math.sqrt(dist_sq)
            
            if dist < self.neighbor_dist:
                # Calculate urgency: closer and approaching neighbors are more urgent
                other_vel = self._get_velocity(other)
                rel_vel_x = other_vel[0] - circle_vel[0]
                rel_vel_y = other_vel[1] - circle_vel[1]
                
                # Dot product: negative means approaching
                approach_rate = (dx * rel_vel_x + dy * rel_vel_y) / (dist + 0.01)
                if approach_rate >= 0:
                    continue
                
                # Urgency score: combine distance and approach rate
                # Lower score = more urgent
                urgency = dist - approach_rate * 0.5
                
                neighbors.append((urgency, other))
        
        # Sort by urgency and limit to max_neighbors
        neighbors.sort(key=lambda x: x[0])
        return [n[1] for n in neighbors[:self.max_neighbors]]
    
    def _compute_orca_line(self, circle, neighbor, delta_time, time_horizon):
        """Compute the ORCA line for a neighbor"""
        # Relative position
        rel_pos = (neighbor["x"] - circle["x"], neighbor["y"] - circle["y"])
        
        # Relative velocity (assume neighbor is also trying to avoid)
        circle_vel = self._get_velocity(circle)
        neighbor_vel = self._get_velocity(neighbor)
        rel_vel = (circle_vel[0] - neighbor_vel[0], circle_vel[1] - neighbor_vel[1])
        neighbor_stationary = (
            abs(neighbor_vel[0]) < 1e-6 and abs(neighbor_vel[1]) < 1e-6
        )
        
        # Combined radius (适度的安全边距)
        safety_margin = 1.5  # 适中的安全边距,避免碰撞但不过度保守
        combined_radius = circle["radius"] + neighbor["radius"] + safety_margin
        
        dist_sq = rel_pos[0]**2 + rel_pos[1]**2
        dist = math.sqrt(dist_sq)
        
        if dist < 0.01:
            # Agents are at the same position, push away in arbitrary direction
            # Use a random-ish direction based on neighbor position
            push_dir = (1.0, 0.5)
            push_length = math.sqrt(push_dir[0]**2 + push_dir[1]**2)
            return {
                'point': (0, 0),
                'direction': (push_dir[0]/push_length, push_dir[1]/push_length)
            }
        
        # ORCA computation
        w = (
            rel_vel[0] - rel_pos[0] / time_horizon,
            rel_vel[1] - rel_pos[1] / time_horizon,
        )
        
        # Check if collision is imminent
        w_length_sq = w[0]**2 + w[1]**2
        
        dot_product = rel_pos[0] * w[0] + rel_pos[1] * w[1]
        
        if dot_product < 0 and dot_product**2 > combined_radius**2 * w_length_sq:
            # No collision
            w_length = math.sqrt(w_length_sq)
            if w_length < 0.01:
                return None
            unit_w = (w[0] / w_length, w[1] / w_length)
            
            # ORCA line direction (perpendicular to w)
            line_direction = (unit_w[1], -unit_w[0])
            
            # ORCA line passes through u
            scale = 1.0 if neighbor_stationary else 0.5
            u = (w[0] * scale, w[1] * scale)
            
            return {
                'point': u,
                'direction': line_direction,
                'stationary': neighbor_stationary,
            }
        else:
            # Collision will occur
            # Compute the cutoff circle
            if dist > combined_radius:
                # No collision yet
                w_norm = math.sqrt(w_length_sq)
                if w_norm < 0.01:
                    return None
                
                unit_w = (w[0] / w_norm, w[1] / w_norm)
                
                # Leg direction
                leg_length = math.sqrt(dist_sq - combined_radius**2)
                leg_direction = (
                    (rel_pos[0] * leg_length - rel_pos[1] * combined_radius) / dist_sq,
                    (rel_pos[0] * combined_radius + rel_pos[1] * leg_length) / dist_sq
                )
                
                # Choose the leg closer to w
                dot1 = w[0] * leg_direction[0] + w[1] * leg_direction[1]
                leg_direction2 = (
                    (rel_pos[0] * leg_length + rel_pos[1] * combined_radius) / dist_sq,
                    (-rel_pos[0] * combined_radius + rel_pos[1] * leg_length) / dist_sq
                )
                dot2 = w[0] * leg_direction2[0] + w[1] * leg_direction2[1]
                
                if dot1 > dot2:
                    line_direction = (leg_direction[1], -leg_direction[0])
                else:
                    line_direction = (leg_direction2[1], -leg_direction2[0])
                
                scale = 1.0 if neighbor_stationary else 0.5
                u = (w[0] * scale, w[1] * scale)
                
                return {
                    'point': u,
                    'direction': line_direction,
                    'stationary': neighbor_stationary,
                }
            else:
                # Already colliding, push directly away
                inv_time_step = 1.0 / delta_time
                w = ((rel_pos[0] - rel_vel[0]) * inv_time_step,
                     (rel_pos[1] - rel_vel[1]) * inv_time_step)
                
                w_length = math.sqrt(w[0]**2 + w[1]**2)
                if w_length < 0.01:
                    return None
                
                unit_w = (w[0] / w_length, w[1] / w_length)
                line_direction = (unit_w[1], -unit_w[0])
                scale = 1.0 if neighbor_stationary else 0.5
                u = (w[0] * scale, w[1] * scale)
                
                return {
                    'point': u,
                    'direction': line_direction,
                    'stationary': neighbor_stationary,
                }
    
    def _linear_program(self, orca_lines, pref_velocity, circle):
        """
        Solve the linear program to find the optimal velocity
        Uses a simple 2D linear programming approach with deadlock handling
        """
        if not orca_lines:
            return pref_velocity
        
        # Start with preferred velocity
        candidate_vel = pref_velocity
        
        # Try to satisfy all ORCA constraints
        for i, line in enumerate(orca_lines):
            # Check if candidate velocity satisfies this line constraint
            if not self._satisfies_constraint(candidate_vel, line):
                # Project velocity onto the line
                candidate_vel = self._project_onto_line(candidate_vel, line, circle)
        
        # Check if the resulting velocity is too small (deadlock situation)
        vel_magnitude = math.sqrt(candidate_vel[0]**2 + candidate_vel[1]**2)
        
        if vel_magnitude < self.min_speed:
            # Try alternative: use preferred direction but with minimum speed
            pref_magnitude = math.sqrt(pref_velocity[0]**2 + pref_velocity[1]**2)
            if pref_magnitude > 0.01:
                # Scale to minimum speed
                scale = self.min_speed / pref_magnitude
                candidate_vel = (pref_velocity[0] * scale, pref_velocity[1] * scale)
                
                # Try this velocity with relaxed constraints
                candidate_vel = self._linear_program_relaxed(orca_lines, candidate_vel, circle)
        
        return candidate_vel
    
    def _linear_program_relaxed(self, orca_lines, initial_velocity, circle):
        """
        Linear program with relaxed constraints when deadlock is detected
        Only considers the most important constraints (closest obstacles)
        """
        if not orca_lines:
            return initial_velocity
        
        # 只使用前2个最重要的约束(从3减少到2,更激进)
        max_constraints = min(2, len(orca_lines))
        important_lines = orca_lines[:max_constraints]
        
        candidate_vel = initial_velocity
        
        for line in important_lines:
            if not self._satisfies_constraint(candidate_vel, line):
                candidate_vel = self._project_onto_line(candidate_vel, line, circle)
        
        return candidate_vel
    
    def _satisfies_constraint(self, velocity, line):
        """Check if a velocity satisfies an ORCA line constraint"""
        # The velocity should be on the correct side of the line
        rel_vel = (velocity[0] - line['point'][0], velocity[1] - line['point'][1])
        
        # Dot product with line direction (perpendicular check)
        # If positive, velocity is on the allowed side
        dot = rel_vel[0] * line['direction'][0] + rel_vel[1] * line['direction'][1]
        
        return dot >= -1e-6  # Small epsilon for numerical stability
    
    def _project_onto_line(self, velocity, line, circle):
        """Project velocity onto the allowed side of ORCA line"""
        # Vector from line point to velocity
        rel_vel = (velocity[0] - line['point'][0], velocity[1] - line['point'][1])
        
        # Project onto line direction
        dot = rel_vel[0] * line['direction'][0] + rel_vel[1] * line['direction'][1]
        
        if dot >= 0:
            return velocity
        
        # Project onto the line
        projected = (
            velocity[0] - dot * line['direction'][0],
            velocity[1] - dot * line['direction'][1]
        )
        
        # Clamp to max speed
        speed = math.sqrt(projected[0]**2 + projected[1]**2)
        max_speed = min(circle["speed"], self.max_speed)
        
        if speed > max_speed:
            scale = max_speed / speed
            projected = (projected[0] * scale, projected[1] * scale)
        elif speed < self.min_speed and speed > 0.01:
            # Ensure minimum speed to prevent getting stuck
            scale = self.min_speed / speed
            projected = (projected[0] * scale, projected[1] * scale)
        
        return projected
    
    def _get_velocity(self, circle):
        """Get current velocity from circle's angle and speed"""
        if circle.get("target_x") is None or circle.get("target_y") is None:
            return (0.0, 0.0)
        if circle.get("speed", 0.0) <= 0.01:
            return (0.0, 0.0)
        angle_rad = math.radians(circle["angle"])
        # Note: In many systems, 0 degrees points right and increases counter-clockwise
        return (
            math.cos(angle_rad) * circle["speed"],
            math.sin(angle_rad) * circle["speed"]
        )
    
    def _apply_velocity(self, circle, velocity, delta_time):
        """Apply velocity to circle and return new position and angle"""
        # Calculate new position
        new_x = circle["x"] + velocity[0] * delta_time
        new_y = circle["y"] + velocity[1] * delta_time
        
        # Calculate new angle from velocity
        if abs(velocity[0]) > 0.01 or abs(velocity[1]) > 0.01:
            new_angle = math.degrees(math.atan2(velocity[1], velocity[0]))
        else:
            new_angle = circle["angle"]
        
        return (new_x, new_y, new_angle)
    
    def _angle_to_target(self, circle, target_x, target_y):
        dx = target_x - circle["x"]
        dy = target_y - circle["y"]
        return normalize_angle(math.degrees(math.atan2(dy, dx)))

    def _get_time_horizon(self, circle, target_x, target_y, delta_time):
        base_time = self.time_horizon
        if not self.concave_detection_enabled:
            return base_time

        current_distance = math.hypot(target_x - circle["x"], target_y - circle["y"])
        last_distance = circle.get("concave_last_distance")
        stuck_timer = circle.get("concave_timer", 0.0)
        epsilon = self.concave_progress_epsilon

        if last_distance is None:
            circle["concave_last_distance"] = current_distance
            return base_time

        if current_distance < last_distance - epsilon:
            stuck_timer = max(0.0, stuck_timer - self.concave_decay * delta_time)
        else:
            stuck_timer += delta_time

        circle["concave_last_distance"] = current_distance
        circle["concave_timer"] = stuck_timer

        growth = self.concave_time_growth
        max_time = self.concave_time_max
        return min(base_time * (1.0 + stuck_timer * growth), max_time)

    def _adjust_preferred_velocity(self, circle, pref_velocity):
        pref_speed = math.hypot(pref_velocity[0], pref_velocity[1])
        if pref_speed < 1e-6:
            return pref_velocity

        current_angle = circle["angle"]
        pref_angle = math.degrees(math.atan2(pref_velocity[1], pref_velocity[0]))

        # Turn penalty: bias towards current direction when the turn is large
        if self.turn_penalty_weight > 0.0:
            diff = angular_distance(pref_angle, current_angle)
            if diff > self.turn_penalty_threshold_deg:
                denom = max(1e-6, 180.0 - self.turn_penalty_threshold_deg)
                normalized = (diff - self.turn_penalty_threshold_deg) / denom
                blend = min(1.0, normalized * self.turn_penalty_weight)
                pref_dir = (pref_velocity[0] / pref_speed, pref_velocity[1] / pref_speed)
                current_dir = (
                    math.cos(math.radians(current_angle)),
                    math.sin(math.radians(current_angle)),
                )
                blended = (
                    pref_dir[0] * (1.0 - blend) + current_dir[0] * blend,
                    pref_dir[1] * (1.0 - blend) + current_dir[1] * blend,
                )
                blended_len = math.hypot(blended[0], blended[1])
                if blended_len > 1e-6:
                    pref_velocity = (
                        blended[0] / blended_len * pref_speed,
                        blended[1] / blended_len * pref_speed,
                    )
                    pref_angle = math.degrees(math.atan2(pref_velocity[1], pref_velocity[0]))

        # Opposite suppression when cooldown is active
        if self.opposite_suppression_enabled:
            opposite_dir = normalize_angle(current_angle + 180.0)
            if angular_distance(pref_angle, opposite_dir) <= self.opposite_angle_threshold_deg:
                if self.reverse_cooldown_enabled and circle.get("reverse_cooldown", 0.0) > 0.0:
                    current_dir = (
                        math.cos(math.radians(current_angle)),
                        math.sin(math.radians(current_angle)),
                    )
                    pref_velocity = (current_dir[0] * pref_speed, current_dir[1] * pref_speed)

        return pref_velocity

    def _update_reverse_cooldown(self, circle, selected_angle, target_angle, delta_time):
        if not self.reverse_cooldown_enabled:
            return

        base = self.reverse_cooldown_base
        growth = self.reverse_cooldown_growth
        decay = self.reverse_cooldown_decay
        max_cooldown = self.reverse_cooldown_max
        extreme_threshold = self.reverse_extreme_threshold_deg

        cooldown = circle.get("reverse_cooldown", 0.0)
        extreme_count = circle.get("reverse_extreme_count", 0)

        if angular_distance(selected_angle, target_angle) >= extreme_threshold:
            extreme_count += 1
            cooldown = max(cooldown, min(base * (growth ** (extreme_count - 1)), max_cooldown))
        else:
            extreme_count = max(0, extreme_count - 1)
            cooldown = max(0.0, cooldown - decay * delta_time)

        circle["reverse_cooldown"] = cooldown
        circle["reverse_extreme_count"] = extreme_count
    
    def calculate_avoidance(self, circle, target_x, target_y, all_circles, delta_time):
        """Calculate avoidance using ORCA algorithm with smart escape strategy"""
        # Get circle ID for tracking
        circle_id = circle.get("label", id(circle))

        if target_x is None or target_y is None:
            circle["debug_orca_pref_velocity"] = None
            circle["debug_orca_new_velocity"] = None
            circle["debug_orca_lines"] = []
            circle["debug_orca_stats"] = {
                "neighbors": 0,
                "constraints": 0,
                "pref_speed": 0.0,
                "new_speed": 0.0,
                "collision_pred": False,
            }
            circle["debug_orca_time_horizon"] = None
            circle["debug_orca_neighbor_dist"] = self.neighbor_dist
            circle["debug_orca_velocity_scale"] = 0.5
            circle["debug_detection_radius"] = self.neighbor_dist
            circle["orca_last_velocity"] = (0.0, 0.0)
            return self._apply_velocity(circle, self._get_velocity(circle), delta_time)

        last_target = circle.get("reverse_last_target")
        current_target = (target_x, target_y)
        if last_target is None or last_target != current_target:
            circle["reverse_cooldown"] = 0.0
            circle["reverse_extreme_count"] = 0
            circle["concave_timer"] = 0.0
            circle["concave_last_distance"] = None
            circle["reverse_last_target"] = current_target

        target_angle = self._angle_to_target(circle, target_x, target_y)
        time_horizon = self._get_time_horizon(circle, target_x, target_y, delta_time)
        circle["debug_orca_time_horizon"] = time_horizon
        circle["debug_orca_neighbor_dist"] = self.neighbor_dist
        circle["debug_orca_velocity_scale"] = 0.5
        circle["debug_detection_radius"] = self.neighbor_dist
        
        # Calculate preferred velocity (towards target)
        pref_velocity = self._calculate_preferred_velocity(circle, target_x, target_y)
        pref_velocity = self._adjust_preferred_velocity(circle, pref_velocity)
        circle["debug_orca_pref_velocity"] = pref_velocity
        
        # Get nearby neighbors
        neighbors = self._get_neighbors(circle, all_circles)
        nearest_dist = None
        total_neighbors = 0
        for other in all_circles:
            if other is circle:
                continue
            if circle.get("target_ref") is not None and other is circle.get("target_ref"):
                continue
            total_neighbors += 1
            dist = math.hypot(other["x"] - circle["x"], other["y"] - circle["y"])
            if nearest_dist is None or dist < nearest_dist:
                nearest_dist = dist
        
        if not neighbors:
            # No neighbors, move directly towards target
            return self._apply_velocity(circle, pref_velocity, delta_time)
        
        # 标准ORCA计算
        orca_lines = []
        for neighbor in neighbors:
            line = self._compute_orca_line(circle, neighbor, delta_time, time_horizon)
            if line:
                orca_lines.append(line)
        
        # Find optimal velocity using linear programming
        new_velocity = self._linear_program(orca_lines, pref_velocity, circle)
        circle["debug_orca_new_velocity"] = new_velocity
        circle["debug_orca_lines"] = orca_lines
        circle["debug_orca_stats"] = {
            "neighbors": len(neighbors),
            "constraints": len(orca_lines),
            "pref_speed": math.hypot(pref_velocity[0], pref_velocity[1]),
            "new_speed": math.hypot(new_velocity[0], new_velocity[1]),
            "collision_pred": False,
            "nearest_dist": nearest_dist,
            "target_dist": math.hypot(target_x - circle["x"], target_y - circle["y"]),
            "filtered_neighbors": max(0, total_neighbors - len(neighbors)),
        }

        if self.inertia_enabled:
            last_velocity = circle.get("orca_last_velocity", (0.0, 0.0))
            last_speed = math.hypot(last_velocity[0], last_velocity[1])
            new_speed = math.hypot(new_velocity[0], new_velocity[1])
            if last_speed > self.inertia_min_speed and new_speed > 1e-6:
                blend = max(0.0, min(1.0, self.inertia_weight))
                blended = (
                    new_velocity[0] * (1.0 - blend) + last_velocity[0] * blend,
                    new_velocity[1] * (1.0 - blend) + last_velocity[1] * blend,
                )
                blended_speed = math.hypot(blended[0], blended[1])
                max_speed = min(circle["speed"], self.max_speed)
                if blended_speed > max_speed:
                    scale = max_speed / blended_speed
                    blended = (blended[0] * scale, blended[1] * scale)
                new_velocity = blended
                circle["debug_orca_new_velocity"] = new_velocity
                circle["debug_orca_stats"]["new_speed"] = math.hypot(new_velocity[0], new_velocity[1])
        
        # 碰撞预检测 - 使用严格的检测避免穿透
        test_result = self._apply_velocity(circle, new_velocity, delta_time)
        test_x, test_y, test_angle = test_result
        
        will_collide = False
        for neighbor in neighbors:
            dx = test_x - neighbor["x"]
            dy = test_y - neighbor["y"]
            dist = math.sqrt(dx**2 + dy**2)
            # 严格的碰撞检测,不允许重叠
            if dist < (circle["radius"] + neighbor["radius"] - 0.5):
                will_collide = True
                break
        
        if will_collide:
            # 如果会碰撞,保持线性规划结果
            test_result = self._apply_velocity(circle, new_velocity, delta_time)
            circle["debug_orca_stats"]["collision_pred"] = True

        _, _, new_angle = test_result
        circle["orca_last_velocity"] = new_velocity
        self._update_reverse_cooldown(circle, new_angle, target_angle, delta_time)
        return test_result
    
