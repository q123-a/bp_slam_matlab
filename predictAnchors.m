% Florian Meyer, 08/01/16.
%
% predictAnchors - 对锚点粒子进行预测，加入随机漂移并考虑存活概率
%
% 输入：
%   posteriorParticlesAnchors - 当前锚点粒子集合，cell数组，每个元素包含结构体
%                              .x: 2×numParticles锚点粒子位置
%                              .w: numParticles×1权重向量
%   parameters               - 参数结构体，包含粒子数、锚点漂移噪声方差、存活概率等
%
% 输出：
%   predictedParticlesAnchors - 预测的锚点粒子位置，3维数组 (2 × numParticles × numAnchors)
%   weightsAnchor             - 预测锚点粒子权重，矩阵 (numParticles × numAnchors)

function [predictedParticlesAnchors, weightsAnchor] = predictAnchors(posteriorParticlesAnchors, parameters)

numParticles = parameters.numParticles;                      % 粒子数量
anchorNoiseVariance = parameters.anchorRegularNoiseVariance; % 锚点位置漂移噪声方差
survivalProbability = parameters.survivalProbability;        % 锚点存活概率

numAnchors = size(posteriorParticlesAnchors, 2);             % 锚点数量

% 预分配预测粒子位置数组和权重矩阵
predictedParticlesAnchors = zeros(2, numParticles, numAnchors);
weightsAnchor = zeros(numParticles, numAnchors);

for anchor = 1:numAnchors
    % 根据存活概率调整粒子权重
    weightsAnchor(:, anchor) = survivalProbability * posteriorParticlesAnchors{anchor}.w;
    
    % 生成二维高斯噪声，模拟锚点位置的随机漂移
    anchorNoise = sqrt(anchorNoiseVariance) * [randn(1, numParticles); randn(1, numParticles)];
    
    % 预测锚点粒子位置为先验位置加上漂移噪声
    predictedParticlesAnchors(:, :, anchor) = posteriorParticlesAnchors{anchor}.x + anchorNoise;
end

end
