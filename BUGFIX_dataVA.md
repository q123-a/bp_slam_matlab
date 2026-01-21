# Bug 修复说明：dataVA 索引错误

## 问题描述

运行 `testbed_iterative.m` 时出现错误：
```
索引超过数组元素的数量。索引不能超过 2。

出错 initAnchors (第 21 行)
anchorPositions = dataVA{sensor}.positions(:,parameters.priorKnownAnchors{sensor});
```

## 根本原因

`testbed_iterative.m` 缺少了对 `dataVA` 的**可见性初始化**步骤。

从 `scenarioCleanM2_new.mat` 加载的 `dataVA` 包含：
- Sensor 1: 6个锚点，visibility 矩阵大小为 (6, 1001)
- Sensor 2: 5个锚点，visibility 矩阵大小为 (5, 1001)

但是 `testbed_iterative.m` 设置了 `parameters.maxSteps = 900`，需要将 visibility 矩阵裁剪到 900 步。

## 解决方案

在 `testbed_iterative.m` 第 30-34 行添加：

```matlab
% 初始化 dataVA 的可见性（关键步骤！）
[numSensors, ~] = size(dataVA);
for sensor = 1:numSensors
    dataVA{sensor}.visibility = ones(size(dataVA{sensor}.visibility, 1), parameters.maxSteps);
end
```

这段代码：
1. 获取传感器数量
2. 为每个传感器设置可见性矩阵为全1（所有锚点在所有时间步都可见）
3. 确保可见性矩阵的列数与 `maxSteps` 匹配

## 参考

这个初始化步骤在原始的 `testbed.m` 第 12-15 行中存在，但在创建 `testbed_iterative.m` 时被遗漏了。

---

**修复日期**: 2025-01-19
**状态**: ✅ 已修复
