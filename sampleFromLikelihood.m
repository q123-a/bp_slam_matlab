function [samples] = sampleFromLikelihood(measurementToAnchor, measurementVariance, agentPosition, numParticles)
% sampleFromLikelihood - 根据测距似然采样锚点位置粒子
%
% 输入：
%   measurementToAnchor - 观测到的距离（标量）
%   measurementVariance - 测距方差（标量）
%   agentPosition       - 移动体粒子位置矩阵，2×numParticles
%   numParticles        - 需要采样的粒子数
%
% 输出：
%   samples - 新锚点粒子位置，2×numParticles矩阵

samples = zeros(2, numParticles); % 预分配样本矩阵

% 采样距离r：观测距离加高斯噪声
r = measurementToAnchor + sqrt(measurementVariance) * randn(1, numParticles);

% 采样角度phi：均匀分布[0, 2*pi)
phi = 2 * pi * rand(1, numParticles);

% 计算采样锚点的笛卡尔坐标
samples(1,:) = agentPosition(1,:) + r .* cos(phi);
samples(2,:) = agentPosition(2,:) + r .* sin(phi);

end
