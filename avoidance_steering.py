"""
Steering Behaviors based avoidance algorithm
Combines multiple steering forces for natural group movement and surrounding
"""

from avoidance_base import AvoidanceBase
import math

class SteeringAvoidance(AvoidanceBase):
    """Steering Behaviors - combines seek, separation, and arrival"""
    
    def __init__(self):
        super().__init__()
        self.name = "Steering Behaviors"
        self.description = "Natural group movement with seek, separation, and arrival"
        
        # Steering parameters
        self.separation_radius = 100.0  # 分离半径 - 多远距离开始互相排斥（增大）
        self.separation_weight = 3.5    # 分离权重 - 越大越不容易重叠（增大）
        self.seek_weight = 0.8          # 寻找目标权重（降低，让分离更重要）
        self.arrival_radius = 200.0     # 到达半径 - 开始减速的距离（从120增大到200）
        self.arrival_weight = 1.2       # 到达权重（降低，避免过早停止）
        self.max_force = 2000.0         # 最大转向力（大幅增加，从600提高到2000）
        self.max_speed = 200.0          # 最大速度
        
        # Orbit/Wander parameters - 绕行参数
        self.orbit_radius = 80.0        # 目标周围的轨道半径
        self.orbit_weight = 1.0         # 绕行权重
        self.crowding_threshold = 1     # 拥挤阈值（降低到1个，只要有邻居就触发）
        
        # Cohesion and alignment (optional, for flocking)
        self.cohesion_radius = 150.0    # 聚合半径
        self.cohesion_weight = 0.1      # 聚合权重（减小，避免过度聚集）
        self.alignment_radius = 120.0   # 对齐半径
        self.alignment_weight = 0.15    # 对齐权重（减小）
    
    def calculate_avoidance(self, circle, target_x, target_y, all_circles, delta_time):
        """Calculate avoidance using steering behaviors"""
        
        # Check if we're crowded near the target
        distance_to_target = math.sqrt((target_x - circle["x"])**2 + (target_y - circle["y"])**2)
        nearby_count = self._count_nearby_at_target(circle, all_circles, target_x, target_y)
        
        # 检查前方是否有障碍物（更重要）
        obstacles_ahead = self._check_obstacles_ahead(circle, target_x, target_y, all_circles)
        
        # 拥挤条件：周围有邻居 OR 前方有障碍
        is_crowded = nearby_count >= self.crowding_threshold or obstacles_ahead
        
        # 调试输出
        if distance_to_target < self.arrival_radius:
            if obstacles_ahead:
                print(f"� {circle.get('label', '?')}: 前方有障碍物!")
            print(f"�📍 {circle.get('label', '?')}: {nearby_count}个邻居, 距目标{distance_to_target:.0f}, 拥挤:{is_crowded}")
        else:
            if obstacles_ahead:
                print(f"🚧 {circle.get('label', '?')}: 距{distance_to_target:.0f}, 但前方有障碍!")
        
        # Calculate all steering forces
        # 统一使用正常分离模式
        separation_force = self._separation(circle, all_circles, gentle_mode=False)
        orbit_force = self._orbit_around_target(circle, target_x, target_y, all_circles)
        
        # Optional: add cohesion and alignment for more natural flocking
        cohesion_force = self._cohesion(circle, all_circles)
        alignment_force = self._alignment(circle, all_circles)
        
        # Adjust behavior based on crowding and distance
        # 如果拥挤且接近目标，完全切换到绕行模式
        if is_crowded and distance_to_target < self.arrival_radius:
            # 绕行模式：大幅增强绕行力，让它能突破分离力
            seek_weight = 0.0  # 完全停止直接寻找
            orbit_weight = 5.0  # 大幅增加绕行引导（从3.0提高到5.0）
            separation_weight = self.separation_weight * 0.8  # 适度降低分离（从1.0降到0.8）
            arrival_weight = 0.0  # 不使用 arrival
            
            # 使用纯绕行+分离
            seek_force = (0, 0)
            arrival_force = (0, 0)
            
            # 详细调试输出
            sep_mag = math.sqrt(separation_force[0]**2 + separation_force[1]**2)
            orb_mag = math.sqrt(orbit_force[0]**2 + orbit_force[1]**2)
            print(f"🔄 绕行模式！{circle.get('label', '?')} - Sep:{sep_mag:.0f}×{separation_weight:.1f} Orbit:{orb_mag:.0f}×{orbit_weight}")
            print(f"   分离力:({separation_force[0]:.1f},{separation_force[1]:.1f}) 绕行力:({orbit_force[0]:.1f},{orbit_force[1]:.1f})")
        elif distance_to_target < self.arrival_radius:
            # 接近模式：减速接近
            seek_force = self._seek(circle, target_x, target_y)
            arrival_force = self._arrival(circle, target_x, target_y)
            seek_weight = 0.5
            orbit_weight = 0.5
            separation_weight = self.separation_weight
            arrival_weight = self.arrival_weight
        else:
            # 正常模式：向目标前进
            seek_force = self._seek(circle, target_x, target_y)
            arrival_force = (0, 0)
            seek_weight = self.seek_weight
            orbit_weight = 0.0  # 远离时不需要绕行
            separation_weight = self.separation_weight
            arrival_weight = 0.0
        
        # Combine all forces with weights
        total_force_x = (
            seek_force[0] * seek_weight +
            separation_force[0] * separation_weight +
            arrival_force[0] * arrival_weight +
            orbit_force[0] * orbit_weight +
            cohesion_force[0] * self.cohesion_weight +
            alignment_force[0] * self.alignment_weight
        )
        total_force_y = (
            seek_force[1] * seek_weight +
            separation_force[1] * separation_weight +
            arrival_force[1] * arrival_weight +
            orbit_force[1] * orbit_weight +
            cohesion_force[1] * self.cohesion_weight +
            alignment_force[1] * self.alignment_weight
        )
        
        # Limit total force
        force_magnitude = math.sqrt(total_force_x**2 + total_force_y**2)
        
        if force_magnitude > self.max_force:
            total_force_x = (total_force_x / force_magnitude) * self.max_force
            total_force_y = (total_force_y / force_magnitude) * self.max_force
        
        # Apply force to velocity
        current_vx = circle["speed"] * math.cos(math.radians(circle["angle"]))
        current_vy = circle["speed"] * math.sin(math.radians(circle["angle"]))
        
        new_vx = current_vx + total_force_x * delta_time
        new_vy = current_vy + total_force_y * delta_time
        
        # Limit speed
        speed = math.sqrt(new_vx**2 + new_vy**2)
        if speed > self.max_speed:
            new_vx = (new_vx / speed) * self.max_speed
            new_vy = (new_vy / speed) * self.max_speed
            speed = self.max_speed
        
        # Update position
        new_x = circle["x"] + new_vx * delta_time
        new_y = circle["y"] + new_vy * delta_time
        
        # Update angle
        if speed > 0.1:
            new_angle = math.degrees(math.atan2(new_vy, new_vx))
        else:
            new_angle = circle["angle"]
        
        return (new_x, new_y, new_angle)
    
    def _seek(self, circle, target_x, target_y):
        """Seek behavior - steer towards target"""
        dx = target_x - circle["x"]
        dy = target_y - circle["y"]
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.1:
            return (0, 0)
        
        # Desired velocity
        desired_vx = (dx / distance) * self.max_speed
        desired_vy = (dy / distance) * self.max_speed
        
        # Current velocity
        current_vx = circle["speed"] * math.cos(math.radians(circle["angle"]))
        current_vy = circle["speed"] * math.sin(math.radians(circle["angle"]))
        
        # Steering force = desired - current
        return (desired_vx - current_vx, desired_vy - current_vy)
    
    def _separation(self, circle, all_circles, gentle_mode=False):
        """Separation behavior - avoid crowding neighbors
        
        Args:
            gentle_mode: If True, use gentler separation for orbit mode
        """
        steer_x = 0
        steer_y = 0
        count = 0
        
        for other in all_circles:
            if other is circle:
                continue
            
            dx = circle["x"] - other["x"]
            dy = circle["y"] - other["y"]
            distance = math.sqrt(dx**2 + dy**2)
            
            # Only consider neighbors within separation radius
            if 0 < distance < self.separation_radius:
                # Weight by distance (closer = stronger repulsion)
                weight = 1.0 - (distance / self.separation_radius)
                
                # Also consider circle sizes
                min_distance = circle["radius"] + other["radius"]
                
                if gentle_mode:
                    # 绕行模式：温和的分离，只避免碰撞
                    if distance < min_distance * 1.5:
                        weight *= 2.0  # 线性增强，不用平方
                else:
                    # 正常模式：强力分离
                    if distance < min_distance * 2.0:
                        overlap_factor = min_distance * 2.0 / distance
                        weight *= overlap_factor ** 2
                
                steer_x += (dx / distance) * weight
                steer_y += (dy / distance) * weight
                count += 1
        
        if count > 0:
            steer_x /= count
            steer_y /= count
            
            # Normalize and scale
            magnitude = math.sqrt(steer_x**2 + steer_y**2)
            if magnitude > 0:
                steer_x = (steer_x / magnitude) * self.max_speed
                steer_y = (steer_y / magnitude) * self.max_speed
        
        return (steer_x, steer_y)
    
    def _arrival(self, circle, target_x, target_y):
        """Arrival behavior - slow down when approaching target"""
        dx = target_x - circle["x"]
        dy = target_y - circle["y"]
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.1:
            return (0, 0)
        
        # Calculate desired speed based on distance
        if distance < self.arrival_radius:
            # Slow down proportionally
            desired_speed = self.max_speed * (distance / self.arrival_radius)
        else:
            desired_speed = self.max_speed
        
        # Desired velocity
        desired_vx = (dx / distance) * desired_speed
        desired_vy = (dy / distance) * desired_speed
        
        # Current velocity
        current_vx = circle["speed"] * math.cos(math.radians(circle["angle"]))
        current_vy = circle["speed"] * math.sin(math.radians(circle["angle"]))
        
        # Steering force
        return (desired_vx - current_vx, desired_vy - current_vy)
    
    def _cohesion(self, circle, all_circles):
        """Cohesion behavior - steer towards average position of neighbors"""
        center_x = 0
        center_y = 0
        count = 0
        
        for other in all_circles:
            if other is circle:
                continue
            
            dx = other["x"] - circle["x"]
            dy = other["y"] - circle["y"]
            distance = math.sqrt(dx**2 + dy**2)
            
            if 0 < distance < self.cohesion_radius:
                center_x += other["x"]
                center_y += other["y"]
                count += 1
        
        if count > 0:
            center_x /= count
            center_y /= count
            
            # Seek towards center
            dx = center_x - circle["x"]
            dy = center_y - circle["y"]
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance > 0.1:
                desired_vx = (dx / distance) * self.max_speed
                desired_vy = (dy / distance) * self.max_speed
                
                current_vx = circle["speed"] * math.cos(math.radians(circle["angle"]))
                current_vy = circle["speed"] * math.sin(math.radians(circle["angle"]))
                
                return (desired_vx - current_vx, desired_vy - current_vy)
        
        return (0, 0)
    
    def _alignment(self, circle, all_circles):
        """Alignment behavior - match velocity with neighbors"""
        avg_vx = 0
        avg_vy = 0
        count = 0
        
        for other in all_circles:
            if other is circle:
                continue
            
            dx = other["x"] - circle["x"]
            dy = other["y"] - circle["y"]
            distance = math.sqrt(dx**2 + dy**2)
            
            if 0 < distance < self.alignment_radius:
                # Get other's velocity
                other_vx = other["speed"] * math.cos(math.radians(other["angle"]))
                other_vy = other["speed"] * math.sin(math.radians(other["angle"]))
                
                avg_vx += other_vx
                avg_vy += other_vy
                count += 1
        
        if count > 0:
            avg_vx /= count
            avg_vy /= count
            
            # Current velocity
            current_vx = circle["speed"] * math.cos(math.radians(circle["angle"]))
            current_vy = circle["speed"] * math.sin(math.radians(circle["angle"]))
            
            # Steering force
            return (avg_vx - current_vx, avg_vy - current_vy)
        
        return (0, 0)
    
    def _count_nearby_at_target(self, circle, all_circles, target_x, target_y):
        """Count how many circles are crowded near the target"""
        count = 0
        for other in all_circles:
            if other is circle:
                continue
            
            # 改进：统计当前圆形周围的邻居数量，而不是目标点周围
            # 这样更能反映当前圆形是否被挤住
            dx = other["x"] - circle["x"]  # 改为相对于当前圆形
            dy = other["y"] - circle["y"]
            distance_to_other = math.sqrt(dx**2 + dy**2)
            
            # 如果邻居在分离半径内，算作拥挤
            if distance_to_other < self.separation_radius:
                count += 1
        
        return count
    
    def _check_obstacles_ahead(self, circle, target_x, target_y, all_circles):
        """检查从当前位置到目标的直线路径上是否有障碍物"""
        dx = target_x - circle["x"]
        dy = target_y - circle["y"]
        distance_to_target = math.sqrt(dx**2 + dy**2)
        
        if distance_to_target < 0.1:
            return False
        
        # 方向向量
        dir_x = dx / distance_to_target
        dir_y = dy / distance_to_target
        
        # 检查路径上的障碍物
        look_ahead_distance = min(distance_to_target, 150.0)  # 前瞻距离
        
        for other in all_circles:
            if other is circle:
                continue
            
            # 计算other到路径的距离
            to_other_x = other["x"] - circle["x"]
            to_other_y = other["y"] - circle["y"]
            
            # 投影到前进方向
            projection = to_other_x * dir_x + to_other_y * dir_y
            
            # 只考虑前方的障碍物
            if 0 < projection < look_ahead_distance:
                # 计算垂直距离
                perpendicular_x = to_other_x - projection * dir_x
                perpendicular_y = to_other_y - projection * dir_y
                perpendicular_dist = math.sqrt(perpendicular_x**2 + perpendicular_y**2)
                
                # 如果障碍物在路径上
                safe_distance = circle["radius"] + other["radius"] + 20  # 安全距离
                if perpendicular_dist < safe_distance:
                    return True
        
        return False
    
    def _orbit_around_target(self, circle, target_x, target_y, all_circles):
        """Orbit behavior - move tangentially around obstacles when blocked"""
        dx = circle["x"] - target_x
        dy = circle["y"] - target_y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.1:
            # 如果已经在目标点上，向外推
            import random
            angle = random.random() * 2 * math.pi
            push_x = math.cos(angle) * self.max_speed
            push_y = math.sin(angle) * self.max_speed
            print(f"  ⚠️ {circle.get('label', '?')} 在目标上，随机推出")
            return (push_x, push_y)
        
        # Only activate orbit when close to target
        if distance > self.arrival_radius:
            return (0, 0)
        
        # 新策略：计算切向力，沿着到目标方向的垂直方向移动
        # 这样可以绕过障碍物而不是被推开
        
        # 到目标的方向
        to_target_x = target_x - circle["x"]
        to_target_y = target_y - circle["y"]
        to_target_dist = math.sqrt(to_target_x**2 + to_target_y**2)
        
        if to_target_dist < 0.1:
            return (0, 0)
        
        # 找到最不拥挤的切向方向（左侧或右侧）
        # 计算垂直方向（切向）
        tangent_left_x = -to_target_y / to_target_dist  # 逆时针90度
        tangent_left_y = to_target_x / to_target_dist
        tangent_right_x = to_target_y / to_target_dist  # 顺时针90度
        tangent_right_y = -to_target_x / to_target_dist
        
        # 检查左右两侧哪边更空旷
        left_crowding = self._check_direction_crowding(circle, tangent_left_x, tangent_left_y, all_circles)
        right_crowding = self._check_direction_crowding(circle, tangent_right_x, tangent_right_y, all_circles)
        
        # 选择更空旷的方向
        if left_crowding < right_crowding:
            tangent_x = tangent_left_x
            tangent_y = tangent_left_y
            direction = "左侧"
        else:
            tangent_x = tangent_right_x
            tangent_y = tangent_right_y
            direction = "右侧"
        
        # 沿切向移动，同时轻微向目标倾斜
        blend = 0.3  # 30%向目标，70%沿切向
        desired_vx = (tangent_x * (1-blend) + to_target_x/to_target_dist * blend) * self.max_speed
        desired_vy = (tangent_y * (1-blend) + to_target_y/to_target_dist * blend) * self.max_speed
        
        current_vx = circle["speed"] * math.cos(math.radians(circle["angle"]))
        current_vy = circle["speed"] * math.sin(math.radians(circle["angle"]))
        
        force = (desired_vx - current_vx, desired_vy - current_vy)
        print(f"  🎯 切向绕行: {direction} 拥挤度(左{left_crowding:.1f}/右{right_crowding:.1f})")
        return force
    
    def _check_direction_crowding(self, circle, dir_x, dir_y, all_circles):
        """Check how crowded it is in a specific direction"""
        crowding = 0
        check_distance = self.separation_radius * 1.5  # 检查前方1.5倍分离半径
        
        # 计算该方向前方的检查点
        check_x = circle["x"] + dir_x * check_distance
        check_y = circle["y"] + dir_y * check_distance
        
        for other in all_circles:
            if other is circle:
                continue
            
            # 计算其他圆形到检查点的距离
            dx = other["x"] - check_x
            dy = other["y"] - check_y
            distance = math.sqrt(dx**2 + dy**2)
            
            # 距离越近，拥挤度越高
            if distance < self.separation_radius:
                crowding += (self.separation_radius - distance) / self.separation_radius
        
        return crowding
    
    def _find_best_orbit_angle(self, circle, target_x, target_y, all_circles):
        """Find the angle around target with least crowding"""
        current_angle = math.atan2(circle["y"] - target_y, circle["x"] - target_x)
        
        # Sample angles around the target (增加采样点)
        num_samples = 24  # 从16增加到24
        best_angle = current_angle
        min_crowding = float('inf')
        
        for i in range(num_samples):
            test_angle = (2 * math.pi * i) / num_samples
            test_x = target_x + self.orbit_radius * math.cos(test_angle)
            test_y = target_y + self.orbit_radius * math.sin(test_angle)
            
            # Calculate crowding score for this position
            crowding = 0
            for other in all_circles:
                if other is circle:
                    continue
                
                dx = other["x"] - test_x
                dy = other["y"] - test_y
                dist = math.sqrt(dx**2 + dy**2)
                
                # 更强的拥挤惩罚
                if dist < self.separation_radius:
                    weight = (1.0 - dist / self.separation_radius)
                    crowding += weight ** 2  # 平方惩罚
                
                # 额外惩罚非常近的位置
                min_safe_dist = circle["radius"] + other["radius"]
                if dist < min_safe_dist * 2:
                    crowding += 10.0  # 大惩罚
            
            # 计算从当前位置到测试点的距离（优先选择近的点）
            dist_to_test = math.sqrt((circle["x"] - test_x)**2 + (circle["y"] - test_y)**2)
            crowding += dist_to_test * 0.01  # 轻微惩罚远的点
            
            # 降低平滑性惩罚，让它更倾向于寻找真正的空位
            angle_diff = abs(test_angle - current_angle)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            crowding += angle_diff * 0.05  # 从0.1再降低到0.05
            
            if crowding < min_crowding:
                min_crowding = crowding
                best_angle = test_angle
        
        return best_angle
    
    def _find_best_orbit_angle_dynamic(self, circle, target_x, target_y, all_circles, orbit_radius):
        """Find the angle around target with least crowding (with dynamic radius)"""
        current_angle = math.atan2(circle["y"] - target_y, circle["x"] - target_x)
        
        # Sample angles around the target
        num_samples = 24
        best_angle = current_angle
        min_crowding = float('inf')
        
        for i in range(num_samples):
            test_angle = (2 * math.pi * i) / num_samples
            test_x = target_x + orbit_radius * math.cos(test_angle)
            test_y = target_y + orbit_radius * math.sin(test_angle)
            
            # Calculate crowding score for this position
            crowding = 0
            for other in all_circles:
                if other is circle:
                    continue
                
                dx = other["x"] - test_x
                dy = other["y"] - test_y
                dist = math.sqrt(dx**2 + dy**2)
                
                # 更强的拥挤惩罚
                if dist < self.separation_radius:
                    weight = (1.0 - dist / self.separation_radius)
                    crowding += weight ** 2
                
                # 额外惩罚非常近的位置
                min_safe_dist = circle["radius"] + other["radius"]
                if dist < min_safe_dist * 2:
                    crowding += 10.0
            
            # 计算从当前位置到测试点的距离（优先选择近的点）
            dist_to_test = math.sqrt((circle["x"] - test_x)**2 + (circle["y"] - test_y)**2)
            crowding += dist_to_test * 0.01
            
            # 轻微的角度惩罚
            angle_diff = abs(test_angle - current_angle)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            crowding += angle_diff * 0.05
            
            if crowding < min_crowding:
                min_crowding = crowding
                best_angle = test_angle
        
        return best_angle
