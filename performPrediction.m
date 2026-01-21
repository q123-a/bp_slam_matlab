function [predictedParticles] = performPrediction(oldParticles, parameters)
% performPrediction - 根据运动模型和过程噪声预测下一时刻粒子状态
%
% 输入：
%   oldParticles - 旧时刻粒子状态矩阵（状态维度 × 粒子数），如4×N
%   parameters   - 参数结构体，包含运动模型参数和噪声方差等
%
% 输出：
%   predictedParticles - 预测的粒子状态矩阵（同尺寸）

% 采样时间间隔
scanTime = parameters.scanTime;
% 过程噪声方差（假设均匀，标量）
drivingNoiseVariance = parameters.drivingNoiseVariance;

[~, numParticles] = size(oldParticles); % 粒子数量

% 获取状态转移矩阵A和过程噪声输入矩阵W
[A, W] = getTransitionMatrices(scanTime);

% 根据运动模型进行状态预测，并加入过程噪声
% randn(2, numParticles) 产生2维加速度噪声样本（加速度输入）
predictedParticles = A * oldParticles + W * sqrt(drivingNoiseVariance) * randn(2, numParticles);

end
