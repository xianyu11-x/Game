# Predictive Scan Avoidance

## 简介
`PredictiveScanAvoidance` 通过“预测 + 扫描角度”的方式进行避障：在给定的预测时间窗口内，估计邻居的未来位置，并在可达范围内扫描候选角度，屏蔽会产生碰撞的方向，从而选择一个安全且尽量接近目标方向的角度。

## 核心流程（概览）
1. **目标与状态初始化**：当目标变化时，重置反向冷却与凹形卡住检测计时。
2. **预测时间**：基础预测时间可根据“是否卡住（凹形区域）”动态放大。
3. **检测半径**：根据速度与预测时间生成检测半径，并进行上下限裁剪。
4. **邻居收集**：只考虑检测半径内的邻居。
5. **阻塞区间计算**：对时间样本分段，估算在各时间点与邻居的几何碰撞区间（角度范围）。
6. **可行区间**：从 [-180°, 180°] 中扣除阻塞区间，得到可行角度区间。
7. **角度选择**：从可行区间端点/目标角/当前角构造候选，按“接近目标 + 转向惩罚”评分；必要时抑制反向选择；并带反向冷却控制。
8. **回退策略**：若无可用角度，按配置选择保持当前角度/停止/朝目标方向。

## 主要输入/输出（行为契约）
- **输入**：
  - 当前个体 `circle`（位置、速度、角度、半径等）
  - 目标点 `(target_x, target_y)`
  - 全部实体 `all_circles`
  - `delta_time`
- **输出**：
  - 新位置与新角度 `(new_x, new_y, angle)`
- **失败/回退**：
  - 没有可用角度时，采用 `fallback_method` 处理

## 边界情况
- 无目标：直接保持当前角度移动。
- 无邻居：所有角度可行。
- 与邻居重叠（距离≈0）：直接判定全角度阻塞。
- 预测时间为 0：有效采样将自动跳过。

## 可调整参数与含义
来源：`avoidance_config.py` 的 `AVOIDANCE_CONFIG["predictive_scan"]`

### 基础/预测相关
- `prediction_time`：基础预测时长（秒）。
- `speed_override`：若设置为数值，直接替换 `circle["speed"]` 作为扫描速度。
- `speed_factor`：未设置 `speed_override` 时，速度倍率。

### 检测半径
- `detection_radius_factor`：检测半径计算倍率（$speed \times prediction\_time$ 的系数）。
- `detection_radius_min`：检测半径下限。
- `detection_radius_max`：检测半径上限。

### 角度扫描
- `angle_step_deg`：角度采样步进（度，仅用于 Debug 可视化采样，不参与避障计算）。

### 碰撞/邻居预测
- `collision_padding`：碰撞安全边距。
- `neighbor_speed_factor`：邻居速度倍率（用于预测邻居未来位置）。
- `time_samples`：在预测时间内离散采样次数（越大越精细，但耗时更高）。

### 转向惩罚
- `turn_penalty_weight`：转向惩罚权重。
- `turn_penalty_threshold_deg`：超过此角度才开始施加惩罚。

### 反向抑制
- `opposite_suppression_enabled`：是否抑制与当前方向相反的选择。
- `opposite_angle_threshold_deg`：判定为“反向”的角度阈值。
- `opposite_preference_margin`：只有当反向选择明显更优时才允许（评分差阈值）。

### 反向冷却（抖动抑制）
- `reverse_cooldown_enabled`：是否启用反向冷却。
- `reverse_cooldown_base`：初始冷却时长。
- `reverse_cooldown_max`：冷却时长上限。
- `reverse_cooldown_growth`：连续极端转向时的冷却增长倍率。
- `reverse_cooldown_decay`：冷却衰减速率。
- `reverse_extreme_threshold_deg`：判定“极端转向”的角度差阈值。

### 凹形（卡住）检测
- `concave_detection_enabled`：是否启用卡住检测。
- `concave_progress_epsilon`：判定“有进展”的距离阈值。
- `concave_time_growth`：卡住时预测时间增长系数。
- `concave_time_max`：预测时间上限。
- `concave_decay`：卡住计时衰减系数。

### 角度选择与回退
- `angle_selection_method`：角度选择策略：
  - `closest_to_target`
  - `closest_to_current`
  - `weighted_target_current`
- `fallback_method`：无可行角度时的策略：
  - `keep_current`
  - `stop`
  - `target`

## 相关文件
- `avoidance_predictive.py`：算法实现
- `avoidance_config.py`：可调参数配置
- `test_predictive_scan.py`：相关测试
