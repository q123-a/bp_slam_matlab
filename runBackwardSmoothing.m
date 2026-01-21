% 后向粒子平滑算法 (Forward-Filtering Backward-Simulation)
% Backward Particle Smoothing Algorithm
%
% 基于 Kok et al. (2024) 的理论框架，利用前向滤波的粒子和权重，
% 结合运动模型的物理约束，计算平滑后的轨迹估计。
%
% 改进：使用多轨迹平均 (MMSE 估计)
% - 单次采样方差大，尤其在粒子分散时（轨迹前半段）
% - 多次采样取平均可以降低方差，获得更稳定的估计
% - 类似于蒙特卡洛积分，样本数越多越接近真实期望
%
% 输入:
%   historyParticles    - 历史粒子集合，cell数组 {1:numSteps}
%                        每个元素是 (4, numParticles) 的矩阵
%   historyWeights      - 历史权重集合，cell数组 {1:numSteps}
%                        每个元素是 (numParticles, 1) 的向量
%   parameters          - 参数结构体，包含:
%                        .scanTime: 采样时间间隔
%                        .drivingNoiseVariance: 过程噪声方差
%   numTrajectories     - 采样轨迹数量（默认10）
%                        增加此值可降低方差，但计算时间线性增加
%
% 输出:
%   smoothedTrajectory  - 平滑后状态轨迹 (4, numSteps)
%                        包含 [x, y, vx, vy] 四个维度
%
% Author: 2025
% Converted from Python version

function smoothedTrajectory = runBackwardSmoothing(historyParticles, historyWeights, parameters, numTrajectories)

% 默认参数
if nargin < 4
    numTrajectories = 10;
end

% 0. 基础检查
numSteps = length(historyParticles);
if numSteps == 0
    warning('历史粒子为空，无法执行平滑');
    smoothedTrajectory = [];
    return;
end

if numSteps ~= length(historyWeights)
    error('粒子数量 (%d) 与权重数量 (%d) 不匹配', numSteps, length(historyWeights));
end

% 获取维度信息
numParticles = size(historyParticles{1}, 2);
stateDim = 4;  % [x, y, vx, vy]

fprintf('\n开始后向平滑...\n');
fprintf('  时间步数: %d\n', numSteps);
fprintf('  粒子数量: %d\n', numParticles);
fprintf('  采样轨迹数: %d (多轨迹平均以降低方差)\n', numTrajectories);

% 1. 准备运动模型矩阵
% 我们需要 p(x_{t+1} | x_t) = N(x_{t+1}; A*x_t, Q)
scanTime = parameters.scanTime;
drivingNoiseVariance = parameters.drivingNoiseVariance;

[A, W] = getTransitionMatrices(scanTime);

% 计算过程噪声协方差矩阵 Q = W * sigma^2 * W^T
% 注意：Q 是 (4, 4) 矩阵，但秩只有 2（因为噪声只在加速度空间）
Q = W * (drivingNoiseVariance * eye(2)) * W';

fprintf('  过程噪声方差: %.6e\n', drivingNoiseVariance);
fprintf('  Q 矩阵秩: %d (理论值: 2)\n', rank(Q));

% 2. 处理 Q 矩阵的奇异性
% 由于 Q 是秩亏的（rank=2 < dim=4），不能直接求逆
% 使用伪逆 (Moore-Penrose inverse) 来处理
invQ = pinv(Q);

% 验证伪逆的有效性
reconstructionError = norm(Q * invQ * Q - Q, 'fro');
fprintf('  使用伪逆，重构误差: %.6e\n', reconstructionError);

% 3. 多轨迹采样与平均 (MMSE 估计)
% 存储所有采样轨迹
allTrajectories = zeros(numTrajectories, stateDim, numSteps);

fprintf('  开始采样 %d 条轨迹...\n', numTrajectories);

% 对每条轨迹进行采样
for trajIdx = 1:numTrajectories
    % 为每条轨迹初始化临时数组
    singleTrajectory = zeros(stateDim, numSteps);

    % 3.1 初始化：从最后时刻 (T) 采样一个粒子
    finalWeights = historyWeights{numSteps};

    % 归一化校验（防止浮点误差）
    weightSum = sum(finalWeights);
    if abs(weightSum - 1.0) > 1e-6
        finalWeights = finalWeights / weightSum;
    end

    % 采样最后一个粒子
    idx = randsample(numParticles, 1, true, finalWeights);
    singleTrajectory(:, numSteps) = historyParticles{numSteps}(:, idx);

    % 3.2 反向递归循环 (从 T-1 到 1)
    for t = (numSteps-1):-1:1
        % 当前时刻 t 的粒子群 (潜在的父亲)
        particlesT = historyParticles{t};  % (4, numParticles)
        weightsFilterT = historyWeights{t};  % (numParticles, 1)

        % 下一时刻 t+1 已经确定的平滑状态 (孩子)
        xNextSmoothed = singleTrajectory(:, t+1);  % (4, 1)

        % --- 核心：计算后向权重 ---
        % w_smooth(i) ∝ w_filter(i) * p(x_next | x_t(i))

        % A. 预测所有粒子从 t 到 t+1 的位置: mu = A * x_t
        muT = A * particlesT;  % (4, numParticles)

        % B. 计算残差: diff = x_next - mu
        diff = xNextSmoothed - muT;  % (4, numParticles)

        % C. 计算马氏距离平方
        % mahalanobis_sq(i) = diff(:,i)' * invQ * diff(:,i)
        mahalanobisSq = sum((invQ * diff) .* diff, 1)';  % (numParticles, 1)

        % D. 在对数域计算权重 (防止下溢出)
        logWeightsFilter = log(weightsFilterT + 1e-300);
        logWeightsSmooth = logWeightsFilter - 0.5 * mahalanobisSq;

        % E. Log-Sum-Exp 归一化技巧
        maxLogW = max(logWeightsSmooth);
        weightsSmooth = exp(logWeightsSmooth - maxLogW);
        weightsSum = sum(weightsSmooth);

        % F. 采样父节点
        if weightsSum > 0
            weightsSmooth = weightsSmooth / weightsSum;
            idx = randsample(numParticles, 1, true, weightsSmooth);
            singleTrajectory(:, t) = particlesT(:, idx);
        else
            % 备用方案：如果数值异常，选择概率最大的那个
            [~, idx] = max(logWeightsSmooth);
            singleTrajectory(:, t) = particlesT(:, idx);
        end
    end

    % 保存这条轨迹
    allTrajectories(trajIdx, :, :) = singleTrajectory;

    % 打印进度
    if mod(trajIdx, max(1, floor(numTrajectories/10))) == 0 || trajIdx == numTrajectories
        fprintf('    已完成 %d/%d 条轨迹\n', trajIdx, numTrajectories);
    end
end

% 4. 计算所有轨迹的平均值 (MMSE 估计)
fprintf('  计算 %d 条轨迹的平均值...\n', numTrajectories);
smoothedTrajectory = squeeze(mean(allTrajectories, 1));

fprintf('  后向平滑完成！\n');

end
