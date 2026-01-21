# BP-SLAM 后向粒子平滑功能使用说明
# Backward Particle Smoothing for BP-SLAM - User Guide

## 概述 (Overview)

本文档介绍如何在 MATLAB 版本的 BP-SLAM 代码中使用后向粒子平滑功能。

### 什么是后向粒子平滑？

后向粒子平滑（Forward-Filtering Backward-Simulation, FFBS）是一种利用未来信息改善轨迹估计的算法。它在前向滤波完成后，从最后时刻开始反向采样，结合运动模型约束，得到更平滑、更准确的轨迹估计。

### 主要改进

- **多轨迹平均**：采样 10 条轨迹并取平均，降低估计方差
- **MMSE 估计**：获得最小均方误差估计
- **数值稳定**：使用伪逆处理奇异 Q 矩阵，对数域计算防止下溢

---

## 文件说明 (Files)

### 新增文件

1. **runBackwardSmoothing.m**
   - 后向粒子平滑主函数
   - 输入：历史粒子、历史权重、参数
   - 输出：平滑后的轨迹

2. **analyzeSmoothing.m**
   - 平滑性能分析脚本
   - 生成 6 个子图的详细分析
   - 保存为 `smoothing_analysis.png`

### 修改文件

1. **BPbasedMINTSLAMnew.m**
   - 添加历史数据保存功能
   - 返回 `historyParticles` 和 `historyWeights`

2. **testbed.m**
   - 集成平滑功能调用
   - 添加误差分析和结果保存

---

## 快速开始 (Quick Start)

### 步骤 1：运行主程序

```matlab
% 在 MATLAB 命令窗口中运行
testbed
```

这将：
1. 运行 BP-SLAM 前向滤波
2. 自动执行后向粒子平滑
3. 显示误差对比结果
4. 保存结果到 `results_matlab.mat`

### 步骤 2：分析结果

```matlab
% 分析平滑性能
analyzeSmoothing('results_matlab.mat')
```

这将生成包含 6 个子图的分析图表：
1. 轨迹对比
2. 误差随时间变化
3. 误差改善量
4. 误差分布直方图
5. 累积分布函数 (CDF)
6. 统计摘要

---

## 详细使用说明 (Detailed Usage)

### 自定义轨迹数量

默认使用 10 条轨迹进行平均。可以在 `testbed.m` 中修改：

```matlab
% 修改第 100 行
smoothedTrajectory = runBackwardSmoothing(historyParticles, historyWeights, parameters, 20);  % 使用 20 条轨迹
```

**建议**：
- 快速测试：5 条轨迹
- 标准使用：10 条轨迹（默认）
- 高精度：20-50 条轨迹

### 单独调用平滑函数

如果已经有保存的结果，可以单独调用平滑函数：

```matlab
% 加载结果
load('results_matlab.mat');

% 调用平滑函数
smoothedTrajectory = runBackwardSmoothing(historyParticles, historyWeights, parameters, 10);

% 计算误差
smoothErrors = sqrt(sum((trueTrajectory(1:2,:) - smoothedTrajectory(1:2,:)).^2, 1));
smoothRMSE = sqrt(mean(smoothErrors.^2));
fprintf('平滑 RMSE: %.6f m\n', smoothRMSE);
```

---

## 参数说明 (Parameters)

### runBackwardSmoothing 函数参数

```matlab
smoothedTrajectory = runBackwardSmoothing(historyParticles, historyWeights, parameters, numTrajectories)
```

**输入**：
- `historyParticles`: cell 数组 {1:numSteps}，每个元素是 (4, numParticles) 矩阵
- `historyWeights`: cell 数组 {1:numSteps}，每个元素是 (numParticles, 1) 向量
- `parameters`: 参数结构体，包含 `scanTime` 和 `drivingNoiseVariance`
- `numTrajectories`: 采样轨迹数量（可选，默认 10）

**输出**：
- `smoothedTrajectory`: (4, numSteps) 矩阵，平滑后的状态轨迹 [x, y, vx, vy]

### 关键参数

**drivingNoiseVariance**：
- 当前值：`(v_max / 3 / scanTime)^2`
- 影响：控制运动模型的信任度
- 调整：如果平滑效果不佳，可以尝试增大此值

```matlab
% 在 testbed.m 中修改（第 26 行）
parameters.drivingNoiseVariance = (v_max / 2 / scanTime)^2;  % 增大噪声
```

---

## 算法原理 (Algorithm Principles)

### FFBS 算法流程

1. **前向滤波**：运行粒子滤波，保存每个时刻的粒子和权重
2. **反向采样**：
   - 从最后时刻按权重采样一个粒子
   - 反向递归，根据运动模型计算后向权重
   - 采样父节点粒子
3. **多轨迹平均**：重复步骤 2 多次，对所有轨迹取平均

### 关键公式

**后向权重**：
```
w_smooth(i) ∝ w_filter(i) × p(x_{t+1} | x_t^(i))
```

**运动模型**：
```
x_{t+1} = A × x_t + W × n_a
```

其中 Q = W × σ² × W^T 是奇异矩阵（秩为 2），使用伪逆处理。

---

## 常见问题 (FAQ)

### Q1: 为什么平滑后误差反而增加了？

**可能原因**：
1. 前向滤波已经很准确（RMSE < 10cm），改善空间有限
2. 运动模型不完全匹配真实轨迹（如有加速度变化）
3. 采样方差（可以增加轨迹数量）

**解决方法**：
- 增加轨迹数量到 20-50
- 调整 `drivingNoiseVariance` 参数
- 检查真实轨迹是否符合常速度假设

### Q2: 计算时间太长怎么办？

**优化方法**：
- 减少轨迹数量（如改为 5）
- 减少粒子数量（在 testbed.m 第 49 行修改）
- 减少时间步数（在 testbed.m 第 19 行修改）

### Q3: 如何禁用平滑功能？

在 `testbed.m` 中注释掉第 92-102 行：

```matlab
% % ---------------------------
% % 7.5 执行后向粒子平滑（可选）
% % ---------------------------
% fprintf('\n========================================\n');
% fprintf('开始执行后向粒子平滑...\n');
% fprintf('========================================\n');
%
% % 调用平滑函数（默认使用10条轨迹）
% smoothedTrajectory = runBackwardSmoothing(historyParticles, historyWeights, parameters, 10);
%
% fprintf('平滑完成！\n\n');
```

---

## 技术细节 (Technical Details)

### 内存使用

- **历史数据**：约 3.6 GB（900 步 × 100,000 粒子 × 4 维 × 8 字节）
- **平滑计算**：额外约 360 MB（10 条轨迹）

### 计算时间

- **前向滤波**：约 10-15 分钟（900 步）
- **后向平滑**：约 30-60 秒（10 条轨迹）

### 数值稳定性

1. **Q 矩阵奇异性**：使用 `pinv()` 计算伪逆
2. **权重下溢**：对数域计算 + Log-Sum-Exp 技巧
3. **归一化**：每次采样前检查权重和

---

## 参考文献 (References)

1. Kok, M., et al. (2024). "Particle Smoothing for SLAM Applications"
2. Meyer, F., & Leitinger, E. (2017). "BP-based MINT-SLAM"
3. Python 版本实现：`bp_slam/core/smoothing.py`

---

## 联系与支持 (Contact & Support)

如有问题或建议，请参考：
- Python 版本文档：`SMOOTHING_IMPROVEMENT.md`
- 原始 BP-SLAM 文档：`Readme.md`

---

**版本**：1.0
**日期**：2025-01
**作者**：基于 Python 版本移植
