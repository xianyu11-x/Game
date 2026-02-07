# ORCA (Optimal Reciprocal Collision Avoidance) 算法说明

## 概述

ORCA算法是一种高效的多智能体碰撞避免算法,已成功集成到你的避障插件系统中。

## 算法特点

### ✅ 核心优势

1. **互惠避让**: 假设所有智能体都会相互避让,产生更自然的避障行为
2. **无碰撞保证**: 在理想条件下可以保证智能体之间不会发生碰撞
3. **平滑运动**: 生成的路径比传统方法更加平滑和自然
4. **高效计算**: O(n)的时间复杂度,n为邻近智能体数量
5. **预测性**: 通过时间范围(time horizon)预测未来碰撞

### 🎯 适用场景

- ✓ 多个智能体同时移动
- ✓ 需要平滑、自然的避障效果
- ✓ 密集人群或机器人群导航
- ✓ 游戏中的NPC群体移动
- ✓ 动态环境中的路径规划

## 参数说明

```python
self.time_horizon = 2.0      # 碰撞预测时间范围(秒)
self.max_neighbors = 10      # 考虑的最大邻居数量
self.neighbor_dist = 150.0   # 邻居检测距离(像素)
self.max_speed = 200.0       # 最大速度限制(像素/秒)
self.turn_penalty_weight = 0.6         # 大转向惩罚权重
self.turn_penalty_threshold_deg = 45.0 # 超过该角度才惩罚
self.opposite_suppression_enabled = True
self.opposite_angle_threshold_deg = 30.0
self.reverse_cooldown_enabled = True
self.reverse_cooldown_base = 0.4
self.reverse_cooldown_max = 5.0
self.reverse_cooldown_growth = 2.0
self.reverse_cooldown_decay = 0.8
self.reverse_extreme_threshold_deg = 120.0
self.concave_detection_enabled = True
self.concave_progress_epsilon = 1.5
self.concave_time_growth = 0.6
self.concave_time_max = 4.0
self.concave_decay = 0.8
```

### 参数调优建议

- **time_horizon**: 
  - 增大 → 更早开始避让,运动更平滑,但可能过于保守
  - 减小 → 避让反应更快,但可能来不及避让
  - 推荐值: 1.5 - 3.0秒

- **max_neighbors**: 
  - 增大 → 考虑更多障碍物,更安全,但计算量增加
  - 减小 → 计算更快,但可能忽略某些障碍物
  - 推荐值: 5 - 15

- **neighbor_dist**: 
  - 增大 → 更远距离开始避让
  - 减小 → 仅避让近距离障碍物
  - 推荐值: 智能体速度 × time_horizon × 2

- **max_speed**: 
  - 应与智能体的实际速度范围匹配
  - 过大会导致速度不受限制
  - 过小会限制智能体移动

- **turn_penalty_weight / turn_penalty_threshold_deg**:
  - 抑制大幅度转向,减少抖动
  - threshold 越小,越容易触发惩罚

- **opposite_suppression_enabled / opposite_angle_threshold_deg**:
  - 抑制与当前方向相反的突变

- **reverse_cooldown_***:
  - 频繁反向时触发冷却,降低“左右摇摆”

- **concave_***:
  - 卡住时自动增大 time_horizon,提升绕出凹形区域的成功率

## 使用方法

### 在游戏中使用

1. **启动程序**: 运行 `python test.py`
2. **切换算法**: 按 `TAB` 键在不同算法间切换
3. **ORCA默认已启用**: 程序启动时ORCA是第一个加载的算法

### 测试场景

#### 场景1: 对向移动
- 放置两个圆形(左键点击)
- 选择第一个圆(点击它)
- 右键点击第二个圆的右侧设置目标
- 选择第二个圆
- 右键点击第一个圆的左侧设置目标
- **观察**: 两个圆会相互避让,而不是直接碰撞

#### 场景2: 交叉路径
- 放置4个圆形在十字路口四个方向
- 让它们向中心点移动
- **观察**: ORCA会计算最优路径让所有圆形安全通过

#### 场景3: 密集人群
- 放置10-20个圆形
- 设置它们向相反方向移动
- **观察**: 形成类似人群的避让行为

### 与其他算法对比

按 `TAB` 键切换算法,对比不同效果:

1. **ORCA**: 平滑、预测性避让,适合多智能体
2. **Repulsion**: 斥力场,简单快速但可能抖动
3. **Simple Sliding**: 沿障碍物滑动,适合静态障碍物

## 算法实现细节

### 核心步骤

1. **计算期望速度**: 朝向目标的直线速度
2. **获取邻居**: 找到附近的智能体
3. **计算ORCA线**: 为每个邻居计算半平面约束
4. **线性规划**: 在约束下找到最优速度
5. **应用速度**: 更新位置和角度

### ORCA线计算

ORCA线定义了"禁止速度区域"的边界:
- 线的一侧是安全速度
- 线的另一侧会导致碰撞
- 算法确保选择的速度在所有ORCA线的安全侧

### 碰撞预测

使用相对速度和时间范围预测:
```
future_collision = current_position + relative_velocity × time_horizon
```

## 性能考虑

### 计算复杂度
- **每帧每个智能体**: O(n)，n为邻居数量
- **总计算量**: O(m × n)，m为智能体总数

### 优化建议

1. **限制邻居数量**: 使用 `max_neighbors` 限制
2. **空间分区**: 对大量智能体使用四叉树或网格
3. **异步更新**: 不需要每帧更新所有智能体
4. **距离筛选**: 只考虑 `neighbor_dist` 范围内的邻居

## 局限性

⚠️ **注意事项**:

1. **假设互惠**: 所有智能体都必须使用ORCA才能保证无碰撞
2. **局部最优**: 可能陷入局部死锁(但概率很小)
3. **动态障碍物**: 主要针对其他智能体,静态障碍物需要额外处理
4. **计算成本**: 比简单避障算法计算量大

## 调试和故障排除

### 问题: 智能体仍然碰撞

**可能原因**:
- `time_horizon` 太小
- `neighbor_dist` 太小,未检测到邻居
- 智能体速度超过 `max_speed`
- 碰撞响应系统与ORCA冲突

**解决方案**:
```python
# 在 avoidance_orca.py 中调整参数
self.time_horizon = 3.0  # 增大预测时间
self.neighbor_dist = 200.0  # 增大检测范围
```

### 问题: 运动不够平滑

**可能原因**:
- `time_horizon` 太大
- 邻居太多导致过度约束

**解决方案**:
```python
self.time_horizon = 1.5  # 减小预测时间
self.max_neighbors = 5  # 减少邻居数量
```

### 问题: 智能体移动缓慢

**可能原因**:
- 速度被过度限制
- 约束太多

**解决方案**:
- 检查 `max_speed` 设置
- 减少 `max_neighbors`
- 增大 `neighbor_dist` 让避让更早开始

## 代码示例

### 自定义ORCA参数

编辑 `avoidance_orca.py`:

```python
def __init__(self):
    super().__init__()
    self.name = "ORCA"
    self.description = "Optimal Reciprocal Collision Avoidance"
    
    # 自定义参数
    self.time_horizon = 2.5  # 根据需求调整
    self.max_neighbors = 8
    self.neighbor_dist = 120.0
    self.max_speed = 150.0
```

### 在其他项目中使用

```python
from avoidance_manager import AvoidanceManager

# 初始化管理器
manager = AvoidanceManager()
manager.load_plugins()

# 选择ORCA算法
manager.set_algorithm("ORCA")
orca = manager.get_algorithm()

# 使用ORCA计算避障
result = orca.calculate_avoidance(
    circle=my_agent,
    target_x=target_x,
    target_y=target_y,
    all_circles=all_agents,
    delta_time=0.016  # 60 FPS
)

if result:
    new_x, new_y, new_angle = result
    my_agent["x"] = new_x
    my_agent["y"] = new_y
```

## 参考资料

- **论文**: "Reciprocal Velocity Obstacles for Real-Time Multi-Agent Navigation" (ICRA 2008)
- **扩展**: "Reciprocal n-Body Collision Avoidance" (ISRR 2009)
- **开源实现**: RVO2 Library

## 总结

ORCA算法是你的避障系统中最先进的选项,特别适合:
- 🎮 游戏中的群体AI
- 🤖 机器人群体导航  
- 👥 人群模拟
- 🚗 自动驾驶车辆编队

通过 `TAB` 键可以轻松在ORCA和其他算法间切换,找到最适合你场景的方案!
