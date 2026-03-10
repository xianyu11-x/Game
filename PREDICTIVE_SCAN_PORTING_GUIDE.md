# Predictive Scan Avoidance — 移植指南

> 本文档详细描述 `avoidance_predictive.py` 的算法逻辑、数据结构与参数含义，  
> 目标是让开发者能在 **任意编程语言** 中独立实现等效算法，无需阅读 Python 源码。

---

## 1. 概述

**Predictive Scan** 是一种基于 **速度障碍（Velocity Obstacle）** 思想的局部避障算法。  
核心思路：
1. 在当前帧，以 agent 为圆心、`detection_radius` 为半径圈定邻居与障碍物。
2. 在 `[-180°, 180°)` 的角度空间中，将所有"在 `prediction_time` 内会导致碰撞"的方向标记为 **blocked interval（阻塞区间）**。
3. 对剩余 **allowed interval（允许区间）** 中的候选角度打分，选出最优行进方向。
4. 附带 **反向抑制（opposite suppression）**、**反向冷却（reverse cooldown）**、**凹陷检测（concave detection）** 等辅助机制。

---

## 2. 数据结构

### 2.1 Agent（circle）

```
circle = {
    "x": float,           // 当前 x 坐标
    "y": float,           // 当前 y 坐标
    "radius": float,      // 碰撞半径
    "angle": float,       // 当前朝向角度（度），范围 [-180, 180)
    "speed": float,       // 移动速度（像素/秒）
    "target_x": float?,   // 目标 x（可选，邻居用）
    "target_y": float?,   // 目标 y（可选，邻居用）
    "target_ref": object?, // Z 键标记的攻击目标引用，避障时会排除此对象

    // ---- 以下为算法运行时写入的状态 ----
    "reverse_cooldown": float,       // 反向冷却计时器
    "reverse_extreme_count": int,    // 连续极端偏转计数
    "reverse_last_target": (x, y),   // 上次目标坐标（检测目标切换）
    "concave_timer": float,          // 凹陷检测累计卡住时间
    "concave_last_distance": float?, // 上帧到目标距离
}
```

### 2.2 矩形障碍物（obstacle）

```
obstacle = {
    "cx": float,    // 中心 x
    "cy": float,    // 中心 y
    "w": float,     // 宽度
    "h": float,     // 高度
    "angle": float  // 旋转角度（度）
}
```

### 2.3 Minkowski 圆（内部中间结构）

```
minkowski_circle = {
    "center": (float, float),  // 邻居在 prediction_time 后的预测位置
    "radius": float,           // agent.radius + neighbor.radius + collision_padding
    "source": circle_ref       // 原始邻居引用
}
```

---

## 3. 配置参数详解

| 参数 | 默认值 | 含义 |
|---|---|---|
| **prediction_time** | 0.5 | 基础前瞻时间（秒），agent 预测未来多久的碰撞 |
| **speed_override** | None | 若设为数值，覆盖 agent 的速度用于扫描计算 |
| **speed_factor** | 1.0 | 无 speed_override 时，scan_speed = circle.speed × speed_factor |
| **detection_radius_factor** | 1.5 | 检测半径 = scan_speed × prediction_time × 此因子 |
| **detection_radius_min** | 40.0 | 检测半径下限（像素） |
| **detection_radius_max** | 260.0 | 检测半径上限（像素） |
| **angle_step_deg** | 10.0 | 仅用于 debug 可视化采样步长 |
| **collision_padding** | 2.0 | 碰撞检测时额外膨胀距离（像素） |
| **neighbor_speed_factor** | 1.0 | 预测邻居速度 = neighbor.speed × 此因子 |
| **time_samples** | 16 | 时间采样数量：将 [0, prediction_time] 等分为此数份，逐时刻检测碰撞 |
| **turn_penalty_weight** | 0.6 | 转向惩罚权重，使 agent 倾向于保持当前航向 |
| **turn_penalty_threshold_deg** | 45.0 | 转向惩罚死区，偏转 ≤ 此角度不施加惩罚 |
| **opposite_suppression_enabled** | true | 是否启用反向抑制（避免选择与当前方向相反的角度） |
| **opposite_angle_threshold_deg** | 30.0 | 与"当前角度+180°"偏差 ≤ 此值的角度被视为"反向" |
| **opposite_preference_margin** | 15.0 | 非反向最优比反向最优的分差容忍度；反向角度只有好出此值时才被采纳 |
| **reverse_cooldown_enabled** | true | 是否启用反向冷却（抑制震荡） |
| **reverse_cooldown_base** | 0.4 | 首次极端偏转的冷却基准时长（秒） |
| **reverse_cooldown_max** | 5.0 | 冷却上限（秒） |
| **reverse_cooldown_growth** | 2.0 | 连续极端偏转时，冷却指数增长底数 |
| **reverse_cooldown_decay** | 0.8 | 每秒冷却消退速率 |
| **reverse_extreme_threshold_deg** | 120.0 | 选择角度与目标角度之差 ≥ 此值 → 视为"极端偏转" |
| **concave_detection_enabled** | true | 是否启用凹陷检测（当 agent 被困时自动增大前瞻时间） |
| **concave_progress_epsilon** | 1.5 | 距离减少 > epsilon 才算"有进展" |
| **concave_time_growth** | 0.6 | 卡住每秒 prediction_time 增长系数 |
| **concave_time_max** | 4.0 | prediction_time 的增长上限 |
| **concave_decay** | 0.8 | 有进展时 stuck_timer 每秒衰减系数 |
| **angle_selection_method** | "closest_to_target" | 角度选择策略，可选: `closest_to_target` / `closest_to_current` / `weighted_target_current` |
| **fallback_method** | "keep_current" | 全部角度被阻塞时的行为: `keep_current`(保持航向) / `stop`(停止) / `target`(朝目标走) |

---

## 4. 算法主流程

```
function calculate_avoidance(circle, target_x, target_y, all_circles, delta_time):
    // 1. 无目标 → 按当前朝向直行
    if target is null:
        return move_with_angle(circle, circle.angle, delta_time)

    // 2. 目标切换时重置状态
    if target changed:
        reset reverse_cooldown, extreme_count, concave_timer, concave_last_distance

    // 3. 计算指向目标的角度
    target_angle = atan2(target_y - circle.y, target_x - circle.x) → degrees → normalize

    // 4. 动态前瞻时间（凹陷检测）
    prediction_time = get_prediction_time(circle, target_x, target_y, delta_time)

    // 5. 扫描速度
    speed = speed_override ?? (circle.speed * speed_factor)

    // 6. 检测半径
    detection_radius = clamp(
        min(distance_to_target, speed * prediction_time * detection_radius_factor),
        detection_radius_min,
        detection_radius_max
    )

    // 7. 收集检测半径内的邻居（排除自身和 target_ref）
    neighbors = get_neighbors_in_radius(circle, all_circles, detection_radius)

    // 8. 构建 Minkowski 圆（用于 debug 可视化）
    minkowski_circles = get_minkowski_circles(circle, neighbors, prediction_time)

    // 9. 计算阻塞角度区间
    blocked = get_blocked_intervals(circle, neighbors, speed, prediction_time)
    blocked += get_obstacle_blocked_intervals(circle, detection_radius, speed, prediction_time)
    blocked = merge_intervals(blocked)

    // 10. 计算允许区间 = [-180, 180] \ blocked
    allowed = get_allowed_intervals(blocked)

    // 11. 若无可用角度 → 降级策略
    if allowed is empty:
        return apply_fallback(circle, target_angle, delta_time)

    // 12. 在允许区间中选择最优角度
    selected = select_angle_from_intervals(allowed, target_angle, circle.angle, ...)

    // 13. 更新反向冷却状态
    update_reverse_cooldown(circle, selected, target_angle, delta_time)

    // 14. 以选定角度移动
    return move_with_angle(circle, selected, delta_time)
```

---

## 5. 核心子算法详解

### 5.1 凹陷检测 — `get_prediction_time`

用于检测 agent 是否在凹形障碍物中卡住。若持续无法接近目标，逐步增大前瞻时间以"看得更远"。

```
function get_prediction_time(circle, target_x, target_y, delta_time):
    base_time = config.prediction_time
    if not concave_detection_enabled: return base_time

    current_distance = distance(circle, target)
    last_distance = circle.concave_last_distance
    stuck_timer = circle.concave_timer  // 默认 0

    if last_distance is null:
        circle.concave_last_distance = current_distance
        return base_time

    if current_distance < last_distance - concave_progress_epsilon:
        // 有进展 → 衰减卡住计时
        stuck_timer = max(0, stuck_timer - concave_decay * delta_time)
    else:
        // 没有进展 → 累加
        stuck_timer += delta_time

    circle.concave_last_distance = current_distance
    circle.concave_timer = stuck_timer

    return min(base_time * (1 + stuck_timer * concave_time_growth), concave_time_max)
```

### 5.2 邻居阻塞区间 — `get_blocked_intervals`

对每个邻居，在时间轴 `[0, prediction_time]` 上等分 `time_samples` 个采样点，逐一判断：agent 以速度 `self_speed` 沿某角度走 t 秒的弧是否与邻居预测轨迹的 Minkowski 膨胀圆相交。

**核心几何：两圆相交的角度张角**

```
对每个邻居 other:
    other_speed = get_neighbor_speed(other)
    v_other = (cos(other.angle) * other_speed, sin(other.angle) * other_speed)
    p0 = other.position - circle.position  // 相对位置

    for i = 1 to time_samples:
        t = prediction_time * i / time_samples

        // 邻居在 t 时刻的相对位置
        qx = p0.x + v_other.x * t
        qy = p0.y + v_other.y * t
        distance = hypot(qx, qy)          // 邻居到 agent 的相对距离
        self_distance = self_speed * t      // agent 在 t 时刻能走的距离

        combined_radius = circle.radius + other.radius + collision_padding

        // 两圆不相交 → 跳过
        if distance > self_distance + combined_radius: continue
        // 一个圆完全包含另一个
        if distance < |self_distance - combined_radius|:
            if combined_radius >= self_distance + distance:
                return [(-180, 180)]  // 全阻塞
            continue

        // 余弦定理求半角
        cos_half = (distance² + self_distance² - combined_radius²) / (2 * distance * self_distance)
        cos_half = clamp(cos_half, -1, 1)
        half_angle = acos(cos_half) → degrees

        // 阻塞区间
        center_angle = normalize(atan2(qy, qx) → degrees)
        blocked_start = normalize(center_angle - half_angle)
        blocked_end   = normalize(center_angle + half_angle)

        // 处理跨 ±180° 的情况
        if start <= end:
            blocked.add((start, end))
        else:
            blocked.add((start, 180))
            blocked.add((-180, end))

    return merge_intervals(blocked)
```

### 5.3 矩形障碍物阻塞区间 — `get_obstacle_blocked_intervals`

将矩形障碍物的 **Minkowski 和**（agent 半径 + padding 膨胀后的圆角矩形）分解为：
- **4 条直边段**（膨胀后的矩形边）
- **4 个圆角**（以矩形四角为圆心、膨胀值为半径的圆）

#### 5.3.1 坐标变换

```
// 将 agent 坐标变换到障碍物的局部坐标系
neg_rad = radians(-obstacle.angle)
lx = (agent.x - obstacle.cx) * cos(neg_rad) - (agent.y - obstacle.cy) * sin(neg_rad)
ly = (agent.x - obstacle.cx) * sin(neg_rad) + (agent.y - obstacle.cy) * cos(neg_rad)
```

#### 5.3.2 Agent 在膨胀矩形内部

```
expand = circle.radius + collision_padding
hw = obstacle.w / 2
hh = obstacle.h / 2

inside = false
if |lx| <= hw and |ly| <= hh + expand: inside = true
if |lx| <= hw + expand and |ly| <= hh: inside = true
else:
    cdx = |lx| - hw
    cdy = |ly| - hh
    if cdx > 0 and cdy > 0 and cdx² + cdy² <= expand²: inside = true

if inside:
    // 只允许逃离方向（障碍物反方向的 180° 扇区）
    escape_angle = atan2(agent.y - obs.cy, agent.x - obs.cx) → degrees → normalize
    block 与 escape_angle 相对的 180° 半球
    continue to next obstacle
```

#### 5.3.3 边段裁剪

4 条膨胀边段（局部坐标）：

| 边 | 起点 | 终点 |
|---|---|---|
| bottom | (-hw, -hh - expand) | (hw, -hh - expand) |
| right  | (hw + expand, -hh)  | (hw + expand, hh)  |
| top    | (hw, hh + expand)   | (-hw, hh + expand)  |
| left   | (-hw - expand, hh)  | (-hw - expand, -hh) |

每条边段：
1. 转换到世界坐标。
2. 用 `clip_segment_to_circle` 裁剪到检测圆内。
3. 记录裁剪后两端点相对于 agent 的角度。

所有边段角度收集后，用 **最大间隙法** 确定阻塞区间（详见 5.3.5）。

#### 5.3.4 圆角处理

4 个角的局部坐标：`(±hw, ±hh)`，转世界坐标后作为静态圆（半径 = expand）。

使用与邻居圆相同的 **两圆相交张角** 公式，但因障碍物静止，只需计算一次（取 `self_distance = self_speed × prediction_time`）。

#### 5.3.5 最大间隙法（边段角度 → 阻塞区间）

```
// angles: 所有边段端点角度（已 normalize 到 [-180, 180)）
angles.sort()
找到相邻角度之间的最大间隙（gap）
最大间隙对应的区域 = 允许区域
其余区域 = 阻塞区间 = (gap 后一个角度, gap 前一个角度)
```

### 5.4 区间合并 — `merge_intervals`

```
function merge_intervals(intervals):
    sort intervals by start
    merged = [intervals[0]]
    for each (start, end) in intervals[1:]:
        if start <= merged.last.end + epsilon:
            merged.last.end = max(merged.last.end, end)
        else:
            merged.add((start, end))
    return merged
```

### 5.5 允许区间 — `get_allowed_intervals`

```
function get_allowed_intervals(blocked):
    if blocked is empty: return [(-180, 180)]
    if blocked == [(-180, 180)]: return []

    allowed = []
    cursor = -180
    for (start, end) in blocked:
        if start > cursor:
            allowed.add((cursor, start))
        cursor = max(cursor, end)
    if cursor < 180:
        allowed.add((cursor, 180))
    return allowed
```

### 5.6 角度选择 — `select_angle_from_intervals`

**候选角度** = 每个允许区间的两端点 + target_angle（若在区间内）+ current_angle（若在区间内）。

**打分函数**：

```
penalty(angle) =
    diff = angular_distance(angle, current_angle)
    if diff <= turn_penalty_threshold_deg: 0
    else: (diff - threshold) * turn_penalty_weight

score(angle) = angular_distance(angle, target_angle) + penalty(angle)
```

选择 `score` 最低的候选角度为 `best_angle`。

#### 5.6.1 反向抑制

```
opposite_dir = normalize(current_angle + 180)

if reverse_cooldown > 0:
    // 冷却期间，排除反向候选
    safe_candidates = [a for a in candidates if angular_distance(a, opposite_dir) > opposite_angle_threshold_deg]
    if safe_candidates:
        best_angle = min(safe_candidates, key=score)

// 即使非冷却期，也偏好非反向角度
non_opposite = [a for a in candidates if angular_distance(a, opposite_dir) > opposite_angle_threshold_deg]
best_non_opposite = min(non_opposite, key=score)

// 只有反向角度显著更优时才选它
if score(best_angle) + opposite_preference_margin < score(best_non_opposite):
    return best_angle
else:
    return best_non_opposite
```

### 5.7 反向冷却更新 — `update_reverse_cooldown`

每帧调用，防止 agent 在两个方向间频繁震荡。

```
function update_reverse_cooldown(circle, selected_angle, target_angle, delta_time):
    if angular_distance(selected_angle, target_angle) >= reverse_extreme_threshold_deg:
        // 极端偏转
        extreme_count += 1
        cooldown = max(cooldown, min(base * growth^(extreme_count-1), max_cooldown))
    else:
        extreme_count = max(0, extreme_count - 1)
        cooldown = max(0, cooldown - decay * delta_time)
```

---

## 6. 工具函数

### 6.1 normalize_angle

```
function normalize_angle(angle_deg) → float:
    result = angle_deg mod 360
    if result >= 180: result -= 360
    return result
    // 输出范围: [-180, 180)
```

### 6.2 angular_distance

```
function angular_distance(a, b) → float:
    return abs(normalize_angle(a - b))
    // 输出范围: [0, 180]
```

### 6.3 move_with_angle

```
function move_with_angle(circle, angle_deg, delta_time) → (new_x, new_y, angle):
    dist = circle.speed * delta_time
    new_x = circle.x + dist * cos(radians(angle_deg))
    new_y = circle.y + dist * sin(radians(angle_deg))
    return (new_x, new_y, angle_deg)
```

### 6.4 clip_segment_to_circle

将线段 `(x1,y1)→(x2,y2)` 裁剪到以 `(cx,cy)` 为圆心、`r` 为半径的圆内。

```
function clip_segment_to_circle(x1,y1, x2,y2, cx,cy, r):
    dx, dy = x2-x1, y2-y1
    fx, fy = x1-cx, y1-cy
    a = dx² + dy²
    if a ≈ 0:
        if fx² + fy² <= r²: return (x1,y1,x2,y2)
        else: return null
    b = 2*(fx*dx + fy*dy)
    c = fx² + fy² - r²
    disc = b² - 4ac
    if disc < 0: return null
    sd = sqrt(disc)
    t1 = (-b - sd) / (2a)
    t2 = (-b + sd) / (2a)
    t_lo = max(0, t1)
    t_hi = min(1, t2)
    if t_lo > t_hi + 1e-9: return null
    return (x1 + t_lo*dx, y1 + t_lo*dy,
            x1 + t_hi*dx, y1 + t_hi*dy)
```

---

## 7. 邻居速度预测

```
function get_neighbor_speed(neighbor) → float:
    // 没有目标或速度极小 → 视为静止
    if neighbor.target_x is null or neighbor.target_y is null: return 0
    if neighbor.speed <= 0.01: return 0
    return neighbor.speed * neighbor_speed_factor
```

邻居预测位置（匀速直线模型）：

```
function predict_neighbor_position(neighbor, t) → (x, y):
    speed = get_neighbor_speed(neighbor)
    dx = cos(radians(neighbor.angle)) * speed * t
    dy = sin(radians(neighbor.angle)) * speed * t
    return (neighbor.x + dx, neighbor.y + dy)
```

---

## 8. 降级策略 — `apply_fallback`

当所有方向均被阻塞时：

| fallback_method | 行为 |
|---|---|
| `"keep_current"` | 保持当前朝向继续前进 |
| `"stop"` | 原地停止，返回当前坐标 |
| `"target"` | 朝目标方向前进（忽略碰撞） |

---

## 9. 调用接口摘要

```
class PredictiveScanAvoidance:
    obstacles: List[Obstacle]    // 通过 set_obstacles() 设置

    function set_obstacles(obstacles)

    function calculate_avoidance(
        circle: Agent,          // 当前单位
        target_x: float?,       // 目标 x（null = 无目标）
        target_y: float?,       // 目标 y（null = 无目标）
        all_circles: List[Agent], // 场景中所有单位
        delta_time: float       // 帧间隔（秒）
    ) → (new_x, new_y, new_angle)
```

---

## 10. 移植注意事项

1. **角度制**：全部使用 **度（degrees）**，范围 `[-180, 180)`；内部三角函数调用前转弧度。
2. **坐标系**：标准数学坐标系（x 右, y 上；逆时针为正角度）。若目标引擎是屏幕坐标系（y 向下），需翻转 y 轴或调整 `atan2` 参数顺序。
3. **帧间状态**：`reverse_cooldown`、`reverse_extreme_count`、`concave_timer`、`concave_last_distance`、`reverse_last_target` 存储在每个 agent 上，帧间保持。
4. **target_ref 排除**：agent 的攻击目标不应被当做障碍物。
5. **浮点精度**：合并区间使用 `1e-6` 容差；线段裁剪使用 `1e-9` 和 `1e-12`。
6. **性能**：每帧每 agent 复杂度 ≈ O(N × time_samples)，N 为检测半径内邻居数。障碍物额外产生常数级计算。
7. **可选并行**：`time_samples` 个时间切片独立，可并行计算（对应源码的 `parallel` 变体）。

