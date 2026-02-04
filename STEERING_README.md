# Steering Behaviors 避障算法说明

## 概述

Steering Behaviors（转向行为）是一种基于力的运动控制算法，特别适合**多单位包围目标**的场景。

## 🎯 为什么适合包围场景？

### 核心优势

1. **自然分散** - Separation（分离）行为让单位自动保持距离
2. **平滑到达** - Arrival（到达）行为让单位接近目标时减速
3. **群体协调** - 可选的 Cohesion 和 Alignment 让群体更协调
4. **力的组合** - 多种行为可以灵活组合，权重可调

### 与 ORCA 对比

| 特性 | ORCA | Steering Behaviors |
|------|------|-------------------|
| **包围效果** | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| **自然分散** | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| **到达减速** | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| **计算复杂度** | O(n log n) | O(n) |
| **参数调节** | 较复杂 | 简单直观 |
| **物理真实性** | 高 | 中等 |

## 核心行为说明

### 1. Seek（寻找）
- **作用**: 向目标点移动
- **权重**: `seek_weight = 1.0`
- **公式**: `desired_velocity - current_velocity`

### 2. Separation（分离）⭐ 核心
- **作用**: 与其他单位保持距离，避免拥挤
- **权重**: `separation_weight = 2.0`（最重要）
- **半径**: `separation_radius = 80.0` 像素
- **特点**: 距离越近，排斥力越强

### 3. Arrival（到达）⭐ 核心
- **作用**: 接近目标时逐渐减速
- **权重**: `arrival_weight = 1.5`
- **半径**: `arrival_radius = 100.0` 像素
- **效果**: 在半径内速度按比例递减

### 4. Cohesion（聚合）
- **作用**: 向群体中心移动，保持聚集
- **权重**: `cohesion_weight = 0.3`（较小，避免过度聚集）
- **半径**: `cohesion_radius = 150.0` 像素

### 5. Alignment（对齐）
- **作用**: 与邻近单位速度方向一致
- **权重**: `alignment_weight = 0.2`
- **半径**: `alignment_radius = 120.0` 像素

## 参数调优

### 包围场景推荐参数

```python
# 分离参数（最重要）
self.separation_radius = 80.0   # 多大范围内开始分离
self.separation_weight = 2.0    # 分离的优先级

# 到达参数
self.arrival_radius = 100.0     # 多远开始减速
self.arrival_weight = 1.5       # 减速的强度

# 寻找参数
self.seek_weight = 1.0          # 向目标移动的基础权重
```

### 调优建议

#### 想要更分散的包围？
```python
self.separation_radius = 120.0  # 增大分离范围
self.separation_weight = 3.0    # 增大分离权重
self.cohesion_weight = 0.1      # 减小聚合权重
```

#### 想要更紧密的包围？
```python
self.separation_radius = 60.0   # 减小分离范围
self.separation_weight = 1.5    # 减小分离权重
self.cohesion_weight = 0.5      # 增大聚合权重
```

#### 想要更平滑的到达？
```python
self.arrival_radius = 150.0     # 增大减速范围
self.arrival_weight = 2.0       # 增大减速权重
```

## 使用方法

### 1. 启动测试程序
```bash
python test.py
```

### 2. 切换到 Steering Behaviors
- 按 `TAB` 键多次，直到显示 "Steering Behaviors"

### 3. 测试包围效果
1. 在屏幕上放置 10-15 个圆形（左键点击）
2. 右键点击屏幕中央某个位置作为共同目标
3. 观察圆形如何自然分散并包围目标

### 预期效果
- ✅ 圆形会保持适当距离
- ✅ 接近目标时会自动减速
- ✅ 形成自然的包围圈
- ✅ 不会全部挤在同一位置

## 行为组合示例

### 场景 1: 简单包围
```python
# 只使用核心行为
seek_weight = 1.0
separation_weight = 2.0
arrival_weight = 1.5
cohesion_weight = 0.0  # 不需要聚合
alignment_weight = 0.0  # 不需要对齐
```

### 场景 2: 群体包围（类似鸟群）
```python
# 使用所有行为
seek_weight = 1.0
separation_weight = 2.0
arrival_weight = 1.5
cohesion_weight = 0.3   # 轻微聚合
alignment_weight = 0.2  # 方向对齐
```

### 场景 3: 松散包围
```python
# 强调分离
seek_weight = 1.0
separation_weight = 3.0  # 强分离
arrival_weight = 2.0
cohesion_weight = 0.0
alignment_weight = 0.0
```

## 优化技巧

### 1. 性能优化
- 使用空间分区（四叉树）来减少邻居检测
- 只考虑最近的 N 个邻居

### 2. 避免震荡
```python
# 适当限制最大转向力
self.max_force = 500.0  # 不要太大
```

### 3. 避免堆叠
```python
# 在 separation 中增加距离惩罚
if distance < min_distance * 1.5:
    weight *= 2.0  # 太近时加倍排斥
```

## 常见问题

### Q: 单位会抖动？
**A**: 减小 `max_force` 或增大 `separation_radius`

### Q: 包围不够紧密？
**A**: 减小 `separation_radius` 或 `arrival_radius`

### Q: 到达目标后还在动？
**A**: 增大 `arrival_weight` 或在代码中添加停止逻辑

### Q: 单位速度不一致？
**A**: 增大 `alignment_weight`，让速度更一致

## 算法原理

### 力的计算流程

```
1. 计算各个转向力
   ├─ Seek Force: 指向目标
   ├─ Separation Force: 远离邻居
   ├─ Arrival Force: 到达减速
   ├─ Cohesion Force: 向群体中心
   └─ Alignment Force: 匹配邻居速度

2. 加权组合
   Total Force = Σ(Force_i × Weight_i)

3. 限制力的大小
   if |Total Force| > max_force:
       Total Force = normalize(Total Force) × max_force

4. 更新速度
   new_velocity = current_velocity + Total Force × delta_time

5. 限制速度
   if |new_velocity| > max_speed:
       new_velocity = normalize(new_velocity) × max_speed

6. 更新位置
   new_position = current_position + new_velocity × delta_time
```

## 与其他算法对比

### 适合 Steering Behaviors 的场景
- ✅ 多单位包围目标
- ✅ 群体移动（鸟群、鱼群）
- ✅ 需要自然减速到达
- ✅ 希望单位保持分散

### 不适合的场景
- ❌ 需要严格无碰撞保证（用 ORCA）
- ❌ 复杂的动态避障（用 ORCA）
- ❌ 高精度路径规划（用 A* + ORCA）

## 总结

**Steering Behaviors 是包围场景的最佳选择！**

优点：
- 🎯 完美的包围效果
- 🌊 平滑自然的运动
- ⚡ 高效的计算
- 🎛️ 简单直观的参数

特别适合：
- 游戏中的 AI 群体
- 敌人包围玩家
- 部队包围目标
- 鸟群/鱼群模拟

现在就试试吧！按 `TAB` 切换到 "Steering Behaviors" 并测试包围效果！🚀
