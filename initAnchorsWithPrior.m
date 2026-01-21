function [estimatedAnchors, posteriorParticlesAnchors] = ...
    initAnchorsWithPrior(parameters, dataVA, numSteps, numSensors, priorMap)
% initAnchorsWithPrior - 使用先验地图初始化锚点
%
% 核心思想: 保留 Round 1 的地图，但放大方差以避免过度自信
%
% 输入:
%   parameters  - 参数结构体
%   dataVA      - 虚拟锚点数据
%   numSteps    - 时间步总数
%   numSensors  - 传感器数量
%   priorMap    - Round 1 的先验地图
%
% 输出:
%   estimatedAnchors         - 初始化的锚点估计
%   posteriorParticlesAnchors- 初始化的锚点粒子
%
% 作者: 2025-01
% 版本: 1.0

fprintf('  使用先验地图初始化锚点...\n');

% 关键参数
varianceInflation = 2.0;      % 方差放大因子 (避免过度自信)
existenceDecay = 0.9;         % 存在概率衰减 (给虚假锚点缓刑期)

numParticles = parameters.numParticles;
posteriorParticlesAnchors = cell(numSensors, 1);
estimatedAnchors = cell(numSensors, numSteps);

for sensor = 1:numSensors
    if isempty(priorMap{sensor})
        % 如果没有先验锚点，使用标准初始化
        posteriorParticlesAnchors{sensor} = {};
        estimatedAnchors{sensor, 1} = {};
        continue;
    end

    numPriorAnchors = length(priorMap{sensor});
    posteriorParticlesAnchors{sensor} = cell(1, numPriorAnchors);
    estimatedAnchors{sensor, 1} = cell(1, numPriorAnchors);

    for anchor = 1:numPriorAnchors
        % 获取先验均值和方差
        mu_prior = priorMap{sensor}{anchor}.mu;
        Sigma_prior = priorMap{sensor}{anchor}.Sigma;

        % 放大方差 (关键步骤!)
        Sigma_inflated = varianceInflation * Sigma_prior;

        % 从放大后的分布采样粒子
        posteriorParticlesAnchors{sensor}{anchor}.x = ...
            mvnrnd(mu_prior, Sigma_inflated, numParticles)';

        % 初始权重均匀分配
        posteriorParticlesAnchors{sensor}{anchor}.w = ...
            ones(numParticles, 1) / numParticles;

        % 继承存在概率，但略微降低 (给虚假锚点淘汰机会)
        priorExistence = priorMap{sensor}{anchor}.existence;
        posteriorParticlesAnchors{sensor}{anchor}.posteriorExistence = ...
            existenceDecay * priorExistence;

        % 初始化估计
        estimatedAnchors{sensor, 1}{anchor}.x = mu_prior;
        estimatedAnchors{sensor, 1}{anchor}.posteriorExistence = ...
            existenceDecay * priorExistence;
        estimatedAnchors{sensor, 1}{anchor}.generatedAt = ...
            priorMap{sensor}{anchor}.generatedAt;
    end

    fprintf('    传感器 %d: 加载 %d 个先验锚点 (方差放大 %.1fx)\n', ...
        sensor, numPriorAnchors, varianceInflation);
end

fprintf('  先验地图加载完成\n');

end
