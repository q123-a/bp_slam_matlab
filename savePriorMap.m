function priorMap = savePriorMap(estimatedAnchors, posteriorParticlesAnchorsstorage, numSensors)
% savePriorMap - 保存 Round 1 的地图作为 Round 2 的先验
%
% 输入:
%   estimatedAnchors                - Round 1 估计的锚点状态 cell 数组 [numSensors, numSteps]
%   posteriorParticlesAnchorsstorage- Round 1 锚点的粒子集合存储 cell 数组
%   numSensors                      - 传感器数量
%
% 输出:
%   priorMap - 先验地图结构体，包含每个锚点的均值、方差和存在概率
%
% 作者: 2025-01
% 版本: 1.1 (修复存储结构访问)

fprintf('  保存 Round 1 地图作为先验...\n');

% 从存储中提取最后一个快照
if ~isempty(posteriorParticlesAnchorsstorage)
    posteriorParticlesAnchors = posteriorParticlesAnchorsstorage{end};
else
    posteriorParticlesAnchors = [];
end

priorMap = cell(numSensors, 1);

for sensor = 1:numSensors
    % 获取该传感器最后时刻的锚点估计
    finalAnchors = estimatedAnchors{sensor, end};

    if isempty(finalAnchors)
        priorMap{sensor} = {};
        continue;
    end

    numAnchors = length(finalAnchors);
    numParticleAnchors = length(posteriorParticlesAnchors{sensor});

    % 只保存那些在粒子集合中存在的锚点
    validAnchors = min(numAnchors, numParticleAnchors);
    priorMap{sensor} = cell(1, validAnchors);

    for anchor = 1:validAnchors
        % 保存锚点位置均值
        priorMap{sensor}{anchor}.mu = finalAnchors{anchor}.x;

        % 计算粒子方差
        particles = posteriorParticlesAnchors{sensor}{anchor}.x;  % (2, numParticles)
        weights = posteriorParticlesAnchors{sensor}{anchor}.w;    % (numParticles, 1)

        % 加权协方差计算
        priorMap{sensor}{anchor}.Sigma = weightedCov(particles, weights);

        % 保存存在概率
        priorMap{sensor}{anchor}.existence = finalAnchors{anchor}.posteriorExistence;

        % 保存生成时刻（用于调试）
        priorMap{sensor}{anchor}.generatedAt = finalAnchors{anchor}.generatedAt;
    end

    fprintf('    传感器 %d: 保存 %d 个锚点 (共 %d 个估计锚点)\n', sensor, validAnchors, numAnchors);
end

fprintf('  地图先验保存完成\n');

end


function Sigma = weightedCov(particles, weights)
% weightedCov - 计算加权粒子的协方差矩阵
%
% 输入:
%   particles - (2, N) 粒子位置矩阵
%   weights   - (N, 1) 归一化权重向量
%
% 输出:
%   Sigma - (2, 2) 协方差矩阵

% 确保权重是列向量
if size(weights, 2) > size(weights, 1)
    weights = weights';
end

% 归一化权重
weights = weights / sum(weights);

% 计算加权均值
mu = particles * weights;  % (2, 1)

% 计算加权协方差
diff = particles - mu;  % (2, N)
Sigma = (diff .* weights') * diff';  % (2, 2)

% 确保协方差矩阵是对称正定的
Sigma = (Sigma + Sigma') / 2;

% 添加小的正则化项，防止奇异
Sigma = Sigma + 1e-6 * eye(2);

end
