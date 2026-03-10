"""
Context Steering based avoidance algorithm
Uses interest/danger maps to make steering decisions
基于上下文转向的避障算法，使用兴趣/危险图来做转向决策
"""

from avoidance_base import AvoidanceBase
import math

class ContextSteeringAvoidance(AvoidanceBase):
    """Context Steering - uses direction slots with interest and danger maps"""
    
    def __init__(self):
        super().__init__()
        self.name = "Context Steering"
        self.description = "Direction-based steering using interest/danger context maps"
        
        # Context map parameters
        self.num_directions = 16           # 方向槽位数量（越多越精确）
        self.detection_radius = 120.0      # 检测障碍物的半径
        self.danger_falloff = 1.5          # 危险值衰减指数
        
        # Movement parameters
        self.max_speed = 200.0             # 最大速度
        self.turn_rate = 8.0               # 转向平滑率（每秒可转向的比例，越大转向越快）
        self.arrival_radius = 150.0        # 到达减速半径
        self.arrival_min_speed = 30.0      # 最小到达速度
        
        # Context blending weights
        self.seek_weight = 1.0             # 寻找目标权重
        self.danger_weight = 1.2           # 危险规避权重
        self.momentum_weight = 0.3         # 动量/惯性权重
        self._momentum_enabled = True      # 是否启用冲量（可通过F2切换）
        
        # Slot smoothing
        self.slot_smooth_range = 2         # 平滑相邻槽位的范围
        
        # Pre-calculate direction vectors for each slot
        self._directions = []
        for i in range(self.num_directions):
            angle = (2 * math.pi * i) / self.num_directions
            self._directions.append((math.cos(angle), math.sin(angle)))
    
    def set_momentum_enabled(self, enabled):
        """设置是否启用冲量"""
        self._momentum_enabled = enabled
    
    def calculate_avoidance(self, circle, target_x, target_y, all_circles, delta_time):
        """Calculate avoidance using context steering maps (直接基于速度，无加速度)"""
        
        # 找到目标圆（如果有的话），需要在障碍物检测中排除它
        target_circle = self._find_target_circle(target_x, target_y, all_circles)
        
        # Step 1: Build interest map (目标吸引力)
        interest_map = self._build_interest_map(circle, target_x, target_y)
        
        # Step 2: Build danger map (障碍物危险度)
        danger_map = self._build_danger_map(circle, all_circles, target_circle)
        
        # Step 3: Build momentum map (保持当前方向的惯性)
        if self._momentum_enabled:
            momentum_map = self._build_momentum_map(circle)
        else:
            momentum_map = [0.0] * self.num_directions
        
        # Step 4: Combine maps
        context_map = self._combine_maps(interest_map, danger_map, momentum_map)
        
        # Step 5: Select best direction
        best_direction = self._select_best_direction(context_map)
        
        # Step 6: Apply arrival behavior (接近目标时减速)
        distance_to_target = math.sqrt((target_x - circle["x"])**2 + (target_y - circle["y"])**2)
        desired_speed = self._calculate_arrival_speed(distance_to_target)
        
        # 计算期望速度向量
        desired_vx = best_direction[0] * desired_speed
        desired_vy = best_direction[1] * desired_speed
        
        # 获取当前速度向量
        current_vx = circle.get("vx", 0)
        current_vy = circle.get("vy", 0)
        
        # 直接基于速度的平滑插值（不使用加速度/力）
        # lerp_factor 控制转向的平滑程度，值越大转向越快
        lerp_factor = min(1.0, self.turn_rate * delta_time)
        
        new_vx = current_vx + (desired_vx - current_vx) * lerp_factor
        new_vy = current_vy + (desired_vy - current_vy) * lerp_factor
        
        # 限制速度
        speed = math.sqrt(new_vx**2 + new_vy**2)
        if speed > self.max_speed:
            new_vx = (new_vx / speed) * self.max_speed
            new_vy = (new_vy / speed) * self.max_speed
            speed = self.max_speed
        
        # 更新位置
        new_x = circle["x"] + new_vx * delta_time
        new_y = circle["y"] + new_vy * delta_time
        
        # 更新角度
        if speed > 0.1:
            new_angle = math.degrees(math.atan2(new_vy, new_vx))
        else:
            new_angle = circle["angle"]
        
        # 保存调试信息用于可视化
        self._save_debug_forces(circle, interest_map, danger_map, momentum_map, context_map, 
                                best_direction, desired_speed, new_vx, new_vy)
        
        return (new_x, new_y, new_angle)
    
    def _build_interest_map(self, circle, target_x, target_y):
        """Build interest map based on target direction
        
        使用平滑的分布函数，让侧向方向也有一定兴趣
        这样侧向移动可以从 interest × safety 的组合中自然涌现
        """
        interest_map = [0.0] * self.num_directions
        
        # Calculate direction to target
        dx = target_x - circle["x"]
        dy = target_y - circle["y"]
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.1:
            # Already at target
            return interest_map
        
        # Normalized direction to target
        target_dir_x = dx / distance
        target_dir_y = dy / distance
        
        # Calculate interest for each direction slot
        for i in range(self.num_directions):
            dir_x, dir_y = self._directions[i]
            
            # Dot product: how well does this slot align with target direction?
            # Range: -1 (opposite) to 1 (same direction)
            dot = dir_x * target_dir_x + dir_y * target_dir_y
            
            # 使用平滑的分布：(dot + 1) / 2 将 [-1, 1] 映射到 [0, 1]
            # 这样：
            #   - 目标方向 (dot=1): interest = 1.0
            #   - 侧向 (dot=0): interest = 0.5
            #   - 后方 (dot=-1): interest = 0.0
            # 然后用幂函数调整分布形状
            normalized = (dot + 1.0) / 2.0  # [0, 1]
            interest_map[i] = normalized ** 1.2  # 稍微集中一点，但侧向仍有分数
        
        return interest_map
    
    def _build_danger_map(self, circle, all_circles, target_circle=None):
        """Build danger map based on obstacles
        
        对于每个方向槽位，计算该方向的危险程度
        """
        danger_map = [0.0] * self.num_directions
        
        for other in all_circles:
            if other is circle:
                continue
            # 排除目标圆 - 目标圆不是障碍物
            if target_circle is not None and other is target_circle:
                continue
            
            # Vector to obstacle
            dx = other["x"] - circle["x"]
            dy = other["y"] - circle["y"]
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < 0.1:
                # Very close, maximum danger in all directions
                for i in range(self.num_directions):
                    danger_map[i] = 1.0
                continue
            
            # Only consider obstacles within detection radius
            # 考虑双方半径
            effective_distance = distance - circle["radius"] - other["radius"]
            if effective_distance > self.detection_radius:
                continue
            
            # Normalized direction to obstacle
            obstacle_dir_x = dx / distance
            obstacle_dir_y = dy / distance
            
            # Calculate danger for each direction slot
            for i in range(self.num_directions):
                dir_x, dir_y = self._directions[i]
                
                # Dot product: how much does this slot point towards the obstacle?
                dot = dir_x * obstacle_dir_x + dir_y * obstacle_dir_y
                
                if dot > 0:
                    # Calculate danger based on distance and alignment
                    # Closer = more dangerous, more aligned = more dangerous
                    
                    # Distance factor (0 to 1, closer = higher)
                    if effective_distance <= 0:
                        distance_factor = 1.0
                    else:
                        distance_factor = 1.0 - (effective_distance / self.detection_radius)
                    distance_factor = max(0, min(1, distance_factor))
                    
                    # Alignment factor (boost for direct collision course)
                    alignment_factor = dot ** self.danger_falloff
                    
                    # Combined danger value
                    danger = distance_factor * alignment_factor
                    
                    # Accumulate danger (multiple obstacles can add up)
                    danger_map[i] = max(danger_map[i], danger)
        
        # Smooth the danger map to create softer avoidance
        danger_map = self._smooth_map(danger_map)
        
        return danger_map
    
    def _build_momentum_map(self, circle):
        """Build momentum map based on current velocity direction
        
        鼓励保持当前移动方向，减少抖动
        当速度很低（疑似卡住）时，自动降低momentum强度，让interest主导决策
        """
        momentum_map = [0.0] * self.num_directions
        
        # Get current velocity direction
        vx = circle.get("vx", 0)
        vy = circle.get("vy", 0)
        speed = math.sqrt(vx**2 + vy**2)
        
        if speed < 1.0:
            # Not moving, no momentum preference
            return momentum_map
        
        # 速度很低时降低momentum强度，防止被锁定在错误方向
        # speed_ratio: 0~1，速度越低momentum越弱
        speed_ratio = min(1.0, speed / (self.max_speed * 0.3))
        
        # Normalized current direction
        current_dir_x = vx / speed
        current_dir_y = vy / speed
        
        # Calculate momentum for each direction slot
        for i in range(self.num_directions):
            dir_x, dir_y = self._directions[i]
            
            # Dot product with current direction
            dot = dir_x * current_dir_x + dir_y * current_dir_y
            
            if dot > 0:
                momentum_map[i] = (dot ** 2) * speed_ratio
        
        return momentum_map
    
    def _combine_maps(self, interest_map, danger_map, momentum_map):
        """Combine interest, danger, and momentum maps
        
        混合决策逻辑：
        1. 前30%最危险的方向：直接屏蔽 interest（硬屏蔽）
        2. 剩余方向：interest × safety 软衰减
        3. 检测"目标方向被挡"：若最高interest方向同时有高danger，
           则降低全局interest权重、提升momentum权重，帮助agent坚持绕行
        4. 最终 context = 调整后的interest + 调整后的momentum
        """
        context_map = [0.0] * self.num_directions
        
        # 找到danger的屏蔽阈值：前30%最危险的方向被硬屏蔽
        sorted_dangers = sorted(danger_map, reverse=True)
        cutoff_index = max(1, int(self.num_directions * 0.3)) - 1
        danger_threshold = sorted_dangers[cutoff_index]
        
        # ---- 检测目标方向是否被障碍物阻挡 ----
        # 找到interest最高的那些方向（目标方向附近），检查它们的danger
        max_interest = max(interest_map) if interest_map else 0
        peak_danger = 0.0
        if max_interest > 0:
            for i in range(self.num_directions):
                if interest_map[i] > max_interest * 0.8:
                    peak_danger = max(peak_danger, danger_map[i])
        
        # 目标方向被挡的程度 (0~1)
        # peak_danger > 0.3 时开始生效，越高说明目标方向挡得越死
        if peak_danger > 0.3:
            obstruction = min(1.0, (peak_danger - 0.3) / 0.4)
        else:
            obstruction = 0.0
        
        # 被挡时：降低interest权重，提升momentum权重
        # 这样agent不会被反复拉向被挡的目标方向，而是坚持沿墙壁绕行
        interest_scale = 1.0 - obstruction * 0.6   # 最低降到 0.4
        momentum_scale = 1.0 + obstruction * 3.0   # 最高升到 4.0
        
        for i in range(self.num_directions):
            interest = interest_map[i] * self.seek_weight * interest_scale
            danger = danger_map[i] * self.danger_weight
            momentum = momentum_map[i] * self.momentum_weight * momentum_scale
            
            if danger_threshold > 0 and danger_map[i] >= danger_threshold:
                # 前30%最危险方向：完全屏蔽interest
                interest = 0.0
            else:
                # 剩余方向：用 safety 做软衰减
                safety = max(0.0, 1.0 - danger)
                interest = interest * safety
            
            context_map[i] = interest + momentum
        
        return context_map
    
    def _select_best_direction(self, context_map):
        """Select the best direction from context map
        
        选择得分最高的方向，使用加权平均来平滑结果
        """
        # Find the peak value
        max_value = max(context_map)
        
        if max_value <= 0:
            # No good direction found, use a default or current direction
            return (1.0, 0.0)  # Default to right
        
        # Weighted average of top directions for smoother movement
        threshold = max_value * 0.7  # Consider directions within 70% of best
        
        weighted_x = 0.0
        weighted_y = 0.0
        total_weight = 0.0
        
        for i in range(self.num_directions):
            if context_map[i] >= threshold:
                weight = context_map[i]
                dir_x, dir_y = self._directions[i]
                weighted_x += dir_x * weight
                weighted_y += dir_y * weight
                total_weight += weight
        
        if total_weight > 0:
            weighted_x /= total_weight
            weighted_y /= total_weight
            
            # Normalize
            magnitude = math.sqrt(weighted_x**2 + weighted_y**2)
            if magnitude > 0.01:
                return (weighted_x / magnitude, weighted_y / magnitude)
        
        # Fallback: return the direction with highest value
        best_index = context_map.index(max_value)
        return self._directions[best_index]
    
    def _calculate_arrival_speed(self, distance):
        """Calculate desired speed based on distance to target (arrival behavior)"""
        if distance < self.arrival_radius:
            # Slow down as we approach
            ratio = distance / self.arrival_radius
            return self.arrival_min_speed + (self.max_speed - self.arrival_min_speed) * ratio
        else:
            return self.max_speed
    
    def _smooth_map(self, map_values):
        """Smooth the map by averaging with neighbors
        
        平滑处理，让相邻槽位之间过渡更自然
        """
        smoothed = [0.0] * self.num_directions
        
        for i in range(self.num_directions):
            total = map_values[i]
            count = 1.0
            
            # Add weighted neighbors
            for offset in range(1, self.slot_smooth_range + 1):
                weight = 1.0 / (offset + 1)
                
                # Left neighbor
                left_idx = (i - offset) % self.num_directions
                total += map_values[left_idx] * weight
                count += weight
                
                # Right neighbor
                right_idx = (i + offset) % self.num_directions
                total += map_values[right_idx] * weight
                count += weight
            
            smoothed[i] = total / count
        
        return smoothed
    
    def _find_target_circle(self, target_x, target_y, all_circles):
        """Find the circle at target position (if any)"""
        for other in all_circles:
            dx = other["x"] - target_x
            dy = other["y"] - target_y
            dist = math.sqrt(dx*dx + dy*dy)
            # 如果某个圆非常接近目标点，认为它就是目标
            if dist < other["radius"] + 5:
                return other
        return None
    
    def _save_debug_forces(self, circle, interest_map, danger_map, momentum_map, context_map, 
                           best_direction, desired_speed, velocity_x, velocity_y):
        """Save debug information for visualization"""
        
        # Calculate representative vectors for visualization
        # 将上下文图转换为可视化的向量
        
        # Context vector (最终决策方向 - 已经考虑了危险)
        context_x = 0
        context_y = 0
        for i in range(self.num_directions):
            dir_x, dir_y = self._directions[i]
            context_x += dir_x * context_map[i]
            context_y += dir_y * context_map[i]
        
        # Danger vector (avoidance direction - opposite of danger)
        danger_x = 0
        danger_y = 0
        for i in range(self.num_directions):
            dir_x, dir_y = self._directions[i]
            danger_x -= dir_x * danger_map[i]
            danger_y -= dir_y * danger_map[i]
        
        # Scale for visualization
        scale = 500
        circle["debug_forces"] = {
            "seek": (context_x * scale, context_y * scale),
            "separation": (danger_x * scale, danger_y * scale),
            "arrival": (0, 0),
            "orbit": (best_direction[0] * desired_speed * 2, best_direction[1] * desired_speed * 2),
            "total": (velocity_x * 3, velocity_y * 3)
        }
        
        # 保存原始map数据，用于雷达图可视化
        circle["debug_maps"] = {
            "interest": list(interest_map),
            "danger": list(danger_map),
            "momentum": list(momentum_map),
            "context": list(context_map),
            "num_directions": self.num_directions,
            "directions": [list(d) for d in self._directions],
        }

