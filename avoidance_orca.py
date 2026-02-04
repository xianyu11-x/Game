"""
ORCA (Optimal Reciprocal Collision Avoidance) algorithm
Provides smooth and efficient collision avoidance using velocity obstacles
"""

from avoidance_base import AvoidanceBase
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
        
        for other in all_circles:
            if other is circle:
                continue
            
            dx = other["x"] - circle["x"]
            dy = other["y"] - circle["y"]
            dist_sq = dx**2 + dy**2
            dist = math.sqrt(dist_sq)
            
            if dist < self.neighbor_dist:
                # Calculate urgency: closer and approaching neighbors are more urgent
                other_vel = self._get_velocity(other)
                rel_vel_x = circle_vel[0] - other_vel[0]
                rel_vel_y = circle_vel[1] - other_vel[1]
                
                # Dot product: negative means approaching
                approach_rate = (dx * rel_vel_x + dy * rel_vel_y) / (dist + 0.01)
                
                # Urgency score: combine distance and approach rate
                # Lower score = more urgent
                urgency = dist - approach_rate * 0.5
                
                neighbors.append((urgency, other))
        
        # Sort by urgency and limit to max_neighbors
        neighbors.sort(key=lambda x: x[0])
        return [n[1] for n in neighbors[:self.max_neighbors]]
    
    def _compute_orca_line(self, circle, neighbor, delta_time):
        """Compute the ORCA line for a neighbor"""
        # Relative position
        rel_pos = (neighbor["x"] - circle["x"], neighbor["y"] - circle["y"])
        
        # Relative velocity (assume neighbor is also trying to avoid)
        circle_vel = self._get_velocity(circle)
        neighbor_vel = self._get_velocity(neighbor)
        rel_vel = (circle_vel[0] - neighbor_vel[0], circle_vel[1] - neighbor_vel[1])
        
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
        w = (rel_vel[0] - rel_pos[0] / self.time_horizon,
             rel_vel[1] - rel_pos[1] / self.time_horizon)
        
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
            u = (w[0] * 0.5, w[1] * 0.5)
            
            return {
                'point': u,
                'direction': line_direction
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
                
                u = (w[0] * 0.5, w[1] * 0.5)
                
                return {
                    'point': u,
                    'direction': line_direction
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
                u = (w[0] * 0.5, w[1] * 0.5)
                
                return {
                    'point': u,
                    'direction': line_direction
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
    
    def _calculate_escape_velocity(self, circle, neighbors, pref_velocity):
        """
        当被包围时,计算逃离速度
        策略: 找到最稀疏的方向逃离
        """
        if not neighbors:
            return pref_velocity
        
        # 计算8个候选方向
        num_directions = 16
        best_direction = None
        best_score = -float('inf')
        
        for i in range(num_directions):
            angle = i * 360.0 / num_directions
            angle_rad = math.radians(angle)
            dir_x = math.cos(angle_rad)
            dir_y = math.sin(angle_rad)
            
            # 评分标准:
            # 1. 方向上最近障碍物的距离(越远越好)
            # 2. 方向上障碍物的数量(越少越好)
            # 3. 与目标方向的接近程度(bonus)
            
            min_dist_in_direction = float('inf')
            obstacles_in_direction = 0
            
            for neighbor in neighbors:
                # 计算邻居相对位置
                to_neighbor_x = neighbor["x"] - circle["x"]
                to_neighbor_y = neighbor["y"] - circle["y"]
                dist_to_neighbor = math.sqrt(to_neighbor_x**2 + to_neighbor_y**2)
                
                if dist_to_neighbor > 0.01:
                    # 检查邻居是否在这个方向上
                    to_neighbor_x /= dist_to_neighbor
                    to_neighbor_y /= dist_to_neighbor
                    
                    dot = dir_x * to_neighbor_x + dir_y * to_neighbor_y
                    
                    # 如果在这个方向的前方(夹角<90度)
                    if dot > 0.3:  # 约70度范围内
                        obstacles_in_direction += 1
                        min_dist_in_direction = min(min_dist_in_direction, dist_to_neighbor)
            
            # 计算得分
            if min_dist_in_direction == float('inf'):
                min_dist_in_direction = 1000  # 这个方向没有障碍物,很好
            
            # 距离分数(距离越远越好)
            distance_score = min_dist_in_direction
            
            # 障碍物数量分数(越少越好)
            obstacle_score = -obstacles_in_direction * 20
            
            # 目标方向分数(与目标方向接近有bonus)
            pref_magnitude = math.sqrt(pref_velocity[0]**2 + pref_velocity[1]**2)
            if pref_magnitude > 0.01:
                pref_dir_x = pref_velocity[0] / pref_magnitude
                pref_dir_y = pref_velocity[1] / pref_magnitude
                target_alignment = (dir_x * pref_dir_x + dir_y * pref_dir_y) * 10
            else:
                target_alignment = 0
            
            total_score = distance_score + obstacle_score + target_alignment
            
            if total_score > best_score:
                best_score = total_score
                best_direction = (dir_x, dir_y)
        
        if best_direction is None:
            return pref_velocity
        
        # 使用逃离速度
        escape_speed = self.escape_mode_speed
        return (best_direction[0] * escape_speed, best_direction[1] * escape_speed)
    
    def calculate_avoidance(self, circle, target_x, target_y, all_circles, delta_time):
        """Calculate avoidance using ORCA algorithm with smart escape strategy"""
        # Get circle ID for tracking
        circle_id = circle.get("label", id(circle))
        
        # Calculate preferred velocity (towards target)
        pref_velocity = self._calculate_preferred_velocity(circle, target_x, target_y)
        
        # Get nearby neighbors
        neighbors = self._get_neighbors(circle, all_circles)
        
        if not neighbors:
            # No neighbors, move directly towards target
            return self._apply_velocity(circle, pref_velocity, delta_time)
        
        # 检查是否被严重包围
        close_neighbors = [n for n in neighbors 
                          if math.sqrt((n["x"]-circle["x"])**2 + (n["y"]-circle["y"])**2) 
                          < (circle["radius"] + n["radius"]) * 2.5]
        
        is_surrounded = len(close_neighbors) >= 3
        
        # 如果被包围,使用逃离策略
        if is_surrounded:
            escape_velocity = self._calculate_escape_velocity(circle, close_neighbors, pref_velocity)
            if escape_velocity:
                return self._apply_velocity(circle, escape_velocity, delta_time)
        
        # 标准ORCA计算
        orca_lines = []
        for neighbor in neighbors:
            line = self._compute_orca_line(circle, neighbor, delta_time)
            if line:
                orca_lines.append(line)
        
        # Find optimal velocity using linear programming
        new_velocity = self._linear_program(orca_lines, pref_velocity, circle)
        
        # 检查速度是否过小
        vel_magnitude = math.sqrt(new_velocity[0]**2 + new_velocity[1]**2)
        
        if vel_magnitude < self.min_speed * 0.5:
            # 速度太小,尝试切线运动
            new_velocity = self._get_tangential_velocity(circle, neighbors, pref_velocity)
        
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
            # 如果会碰撞,使用切线运动
            new_velocity = self._get_tangential_velocity(circle, neighbors, pref_velocity)
            test_result = self._apply_velocity(circle, new_velocity, delta_time)
        
        return test_result
    
    def _get_tangential_velocity(self, circle, neighbors, pref_velocity):
        """
        当直线路径被阻挡时,尝试切线运动
        简化版本: 找到最近障碍物,沿其切线移动
        """
        if not neighbors:
            return pref_velocity
        
        # 找最近的邻居
        nearest = None
        min_dist = float('inf')
        for neighbor in neighbors:
            dx = neighbor["x"] - circle["x"]
            dy = neighbor["y"] - circle["y"]
            dist = math.sqrt(dx**2 + dy**2)
            if dist < min_dist:
                min_dist = dist
                nearest = neighbor
        
        if nearest is None:
            return pref_velocity
        
        # 计算到障碍物的向量
        to_obstacle_x = nearest["x"] - circle["x"]
        to_obstacle_y = nearest["y"] - circle["y"]
        dist = math.sqrt(to_obstacle_x**2 + to_obstacle_y**2)
        
        if dist < 0.01:
            return pref_velocity
        
        to_obstacle_x /= dist
        to_obstacle_y /= dist
        
        # 计算两个切线方向(垂直于障碍物方向)
        tangent1 = (-to_obstacle_y, to_obstacle_x)
        tangent2 = (to_obstacle_y, -to_obstacle_x)
        
        # 计算目标方向
        pref_magnitude = math.sqrt(pref_velocity[0]**2 + pref_velocity[1]**2)
        if pref_magnitude > 0.01:
            pref_dir = (pref_velocity[0] / pref_magnitude, pref_velocity[1] / pref_magnitude)
        else:
            pref_dir = (1, 0)
        
        # 选择更接近目标方向的切线
        dot1 = tangent1[0] * pref_dir[0] + tangent1[1] * pref_dir[1]
        dot2 = tangent2[0] * pref_dir[0] + tangent2[1] * pref_dir[1]
        
        best_tangent = tangent1 if dot1 > dot2 else tangent2
        
        # 使用适中的速度
        speed = circle["speed"] * 0.7
        return (best_tangent[0] * speed, best_tangent[1] * speed)
