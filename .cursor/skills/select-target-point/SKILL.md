---
name: select-target-point
description: Porting guide for the analytical approach-circle target point selection algorithm from Python to UE C++. Use when implementing or modifying FindValidTargetPointLocation, approach circle logic, blocked arc computation, or ranged/melee unit positioning in Unreal Engine.
---

# 目标点选择算法 — Python→UE C++ 移植指南

## 算法概述

给定一个 **施法者 (Caster)** 接近 **目标 (Target)**，在接近圆（远程为攻击距离圆，近战为碰撞圆）上找到一个不被其他实体或矩形障碍物阻挡的最优站位点。

### 核心几何关系

- **碰撞圆** `MinRadius = CasterRadius + TargetRadius + ExtraMargin` — 物理不可穿透的最小距离
- **攻击圆** `DectionRadius = AttackRange`（中心到中心距离）— 远程单位的首选站位距离
- **理想角度** = 从目标中心指向施法者中心的方向 (`atan2(Cy-Ty, Cx-Tx)`)
- **阻挡弧** = 每个障碍物在接近圆上遮挡的角度范围，用 `(中心角, 半宽)` 表示

### 决策流程

```
如果 bUseCasterAttackRange 且 DectionRadius > MinRadius:
    在 DectionRadius 圆上寻找自由角度
        → 找到 → 返回攻击圆上的点
        → 全部被挡 → 返回施法者当前位置（原地不动）
否则（近战）:
    在 MinRadius 圆上寻找自由角度
        → 找到 → 返回碰撞圆上的点
        → 全部被挡 → 返回目标位置（兜底）
```

**额外规则**：远程单位已在攻击范围内（`Distance <= DectionRadius`）时，直接返回自身位置，不移动。

### 阻挡弧的两个来源

1. **圆形实体** — 余弦定理（精确解析）：
   `cos_val = (d² + R² - minDist²) / (2·d·R)` → `halfAngle = acos(cos_val)`

2. **矩形障碍物** — 圆与圆角矩形的解析交点：
   - 4 条边（膨胀 CasterRadius 后的直线段）→ 圆-线交点（一元二次方程）
   - 4 个角（半径为 CasterRadius 的四分之一圆弧）→ 圆-圆交点（余弦定理）
   - 所有交点排序 → 中点测试判定阻挡/自由 → 转换回世界坐标

## Python → UE C++ 关键映射表

| Python 原版 | UE C++ 对应 |
|-------------|-------------|
| `circle["x"], circle["y"]` | `Caster->GetActorLocation()` (XY 平面) |
| `circle["radius"]` | `Caster->GetBoxRadius()` |
| `target_x, target_y` | `Target->GetActorLocation()` (XY 平面) |
| `target_radius` | `Target->GetBoxRadius()` |
| `all_circles` 遍历所有圆 | `UWorldEntityUtil::GetActorsInSphere(...)` 过滤出 `AWorldBaseEntity*` |
| `obstacles_list` 障碍物列表 | 同一球形查询过滤出 `AMapCollisionBox*` |
| `obs["cx"], obs["cy"]` 障碍物中心 | `CollisionBox->GetActorLocation()` |
| `obs["angle"]` (角度，度) | `CollisionBox->GetActorRotation().Yaw` (度，需转弧度) |
| `obs["w"], obs["h"]` 宽高 | `CollisionBox->GetCollisionBoxSize()` (X=宽, Y=高，是全尺寸) |
| `math.atan2(y, x)` | `FMath::Atan2(Y, X)` (弧度) |
| `math.acos(v)` | `FMath::Acos(FMath::Clamp(v, -1.f, 1.f))` |
| `math.hypot(dx, dy)` | `FVector2D(dx, dy).Size()` |
| `extra_margin = 5.0` | `EXTRA_MARGIN = 5.0f` (常量) |

## UE 平台关键差异

1. **3D→2D**：算法在 XY 平面工作。使用 `FVector::DistXY`，结果 Z 坐标取目标的 Z。
2. **旋转角度**：`FRotator::Yaw` 是度，需 `FMath::DegreesToRadians` 转弧度。
3. **GetCollisionBoxSize()**：返回完整尺寸。半尺寸 = Size / 2。
4. **球形查询**：`GetActorsInSphere(Target, QueryRadius)` 替代 Python 的遍历全列表。查询半径应为 `DectionRadius + CasterRadius + ExtraMargin` 以确保覆盖所有可能的阻挡物。
5. **ExtraMargin**：Python 中硬编码为 5.0。C++ 中为常量 `EXTRA_MARGIN`，可改为可配置属性。

## 代码结构

### 函数分解

| 函数 | 职责 |
|------|------|
| `FindValidTargetPointLocation` | 公共入口：收集数据、决策流程、调用 SolveAtRadius |
| `SolveAtRadius(R)` | 在半径 R 的圆上求解：计算阻挡弧 + 找最优自由角 |
| `ComputeEntityArcs` | 计算圆形实体的阻挡弧（余弦定理） |
| `ComputeObstacleArcs` | 计算单个矩形障碍物的阻挡弧（圆角矩形交点） |
| `IsAngleBlocked` | 检查某角度是否被任何弧段覆盖 |
| `FindBestFreeAngle` | 从阻挡弧边界处找到离理想角度最近的自由角 |

### 数据结构

| 结构体 | 用途 |
|--------|------|
| `FSelectTargetPointResult` | 返回值：是否有效 + 目标点坐标 |
| `FBlockedArc` | 单个阻挡弧段：`{中心角, 半宽}` (弧度) |
| `FObstacleData2D` | 预处理后的障碍物数据：中心、旋转角、半宽、半高 |

## 文件列表

- [original_python.py](original_python.py) — Python 原始实现（移植参考，不参与 UE 构建）
- [SelectTargetPointUtils.h](SelectTargetPointUtils.h) — C++ 头文件：结构体定义 + 函数声明
- [SelectTargetPointUtils.cpp](SelectTargetPointUtils.cpp) — C++ 完整实现

## 集成检查清单

- [ ] 用新实现替换现有 `FindValidTargetPointLocation` 函数体
- [ ] 确认 `GetCollisionBoxSize()` 返回的是完整尺寸（宽×高），不是半尺寸
- [ ] 确认 `GetBoxRadius()` 返回用于碰撞检测的圆半径
- [ ] 测试用例：无阻挡物、单个圆形阻挡、单个矩形障碍、混合场景、全部被挡
- [ ] 远程单位已在攻击范围内时，调用方应直接跳过（函数内部也有兜底判断）
- [ ] 检查球形查询返回的 Actor 列表是否包含所需类型（`AWorldBaseEntity` 和 `AMapCollisionBox`）
