# Steering Behaviors 绕行优化说明

## 🎯 问题描述

之前当多个圆形向同一目标移动时，它们会**聚集在目标点**，而不是自动**绕行到空位**包围目标。

## ✅ 解决方案

添加了**智能绕行系统**，包含以下改进：

### 1. **增强分离力（Separation）**
```python
# 原参数
separation_radius = 80.0
separation_weight = 2.0

# 新参数
separation_radius = 100.0  # 增大 25%
separation_weight = 3.5    # 增大 75%
```

**改进**：
- 使用**指数级排斥力**：距离越近，排斥力呈平方增长
- 当距离 < 2倍直径时，触发强力排斥

```python
if distance < min_distance * 2.0:
    overlap_factor = min_distance * 2.0 / distance
    weight *= overlap_factor ** 2  # 平方增强！
```

### 2. **轨道绕行行为（Orbit Around）** ⭐ 新功能

当检测到目标点附近**拥挤**时（>= 3个邻居），自动激活绕行模式：

```python
# 绕行参数
orbit_radius = 80.0           # 轨道半径
orbit_weight = 1.0            # 绕行权重
crowding_threshold = 3        # 拥挤阈值
```

**工作原理**：
1. 检测目标周围的拥挤度
2. 在目标周围采样 16 个角度位置
3. 计算每个位置的拥挤分数
4. 选择最不拥挤的位置
5. 向该位置移动

```
     [空位]        [拥挤]
        ↑            ↓
    ●   |   ●    → 圆形会自动
  ●  [目标]  ●   → 向空位移动
    ●     ●
```

### 3. **动态权重调整**

根据拥挤情况自动调整行为权重：

| 情况 | Seek | Orbit | Separation |
|------|------|-------|------------|
| **正常** | 0.8 | 1.0 | 3.5 |
| **拥挤** | 0.3 ⬇️ | 1.5 ⬆️ | 5.25 ⬆️ |

**效果**：拥挤时优先绕行和分离，而不是硬挤向目标。

### 4. **优化其他参数**

```python
# 降低聚合，避免过度聚集
cohesion_weight = 0.1   # 从 0.3 → 0.1
alignment_weight = 0.15 # 从 0.2 → 0.15

# 增大到达半径，更早减速
arrival_radius = 120.0  # 从 100 → 120

# 增大最大转向力，反应更快
max_force = 600.0       # 从 500 → 600
```

## 🔍 算法流程

```
1. 检测拥挤度
   └─ 统计目标周围的圆形数量

2. 计算基础行为力
   ├─ Seek（寻找）
   ├─ Separation（分离）★
   ├─ Arrival（到达）
   └─ Orbit（绕行）★ 新增

3. 动态调整权重
   IF 拥挤:
      ├─ 降低 Seek 权重
      ├─ 增加 Orbit 权重
      └─ 增强 Separation 权重
   ELSE:
      └─ 使用默认权重

4. 力的组合与限制
   └─ Total = Σ(Force × Weight)

5. 更新速度和位置
```

## 📊 效果对比

### 之前的问题
```
目标 (●)
  
  ●●●●   <- 所有圆形挤在一起
  ●●●●
    ●
```

### 现在的效果
```
    ●
  ●   ●   <- 自动分散到空位
●   ●   ●  <- 形成均匀包围
  ●   ●
    ●
```

## 🎮 测试方法

1. **运行程序**
   ```bash
   python test.py
   ```

2. **切换到 Steering Behaviors**
   - 按 `TAB` 键，直到看到 "Steering Behaviors"

3. **测试包围场景**
   - 在一侧放置 8-12 个圆形（左键点击）
   - 在另一侧放置 1 个圆形作为目标
   - 右键点击目标圆形，将所有圆形的目标设置为它

4. **观察效果**
   - ✅ 圆形会自动绕行
   - ✅ 不会堆在一起
   - ✅ 形成自然的包围圈
   - ✅ 接近时平滑减速

## 🎛️ 高级调优

### 更分散的包围
```python
separation_radius = 120.0
separation_weight = 4.0
orbit_radius = 100.0
```

### 更紧密的包围
```python
separation_radius = 80.0
separation_weight = 2.5
orbit_radius = 60.0
```

### 更积极的绕行
```python
orbit_weight = 2.0
crowding_threshold = 2  # 更容易触发绕行
```

### 更平滑的到达
```python
arrival_radius = 150.0
arrival_weight = 2.0
```

## 🔬 技术细节

### 空位搜索算法

使用**圆周采样**方法：

```python
# 在目标周围采样 16 个位置
for i in range(16):
    angle = (2π × i) / 16
    test_position = target + (orbit_radius × [cos(angle), sin(angle)])
    
    # 计算该位置的拥挤分数
    crowding = Σ(1 - distance/separation_radius)
    
    # 选择最不拥挤的位置
    if crowding < min_crowding:
        best_position = test_position
```

### 平滑性优化

添加了**角度惩罚**，避免急转弯：

```python
angle_diff = abs(test_angle - current_angle)
crowding += angle_diff × 0.3  # 大转弯会增加分数
```

这确保圆形倾向于**平滑绕行**而不是突然改变方向。

## 🐛 已知限制

1. **极端拥挤** - 如果圆形数量 >> 目标周围空间，仍可能部分重叠
2. **性能** - 每个圆形需要采样 16 个角度，圆形很多时可能影响性能
3. **死锁** - 极少情况下可能出现两个圆形互相阻挡

## 🚀 未来改进方向

- [ ] 添加**梯度场**引导绕行方向
- [ ] 使用**空间哈希**优化邻居搜索
- [ ] 添加**预测性避障**，提前规划路径
- [ ] 支持**动态目标**（目标也在移动）

## 📝 总结

通过以下改进，Steering Behaviors 现在能够：

✅ **智能绕行** - 自动寻找空位  
✅ **强力分离** - 不会挤在一起  
✅ **动态调整** - 根据拥挤情况调整策略  
✅ **平滑运动** - 自然的包围效果  

**这是多单位包围场景的最佳解决方案！** 🎯
