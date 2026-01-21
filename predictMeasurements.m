% Florian Meyer, 08/01/16.
%
% predictMeasurements - 根据预测的移动体粒子和锚点位置粒子，计算距离测量的预测均值和不确定度
%
% 输入：
%   predictedParticles - 预测的移动体粒子状态，维度4×numParticles（只用前2维位置）
%   anchorPositions    - 锚点粒子位置，维度2×numParticles×numAnchors
%   weightsAnchor      - 对应锚点粒子的权重矩阵，维度numParticles×numAnchors
%
% 输出：
%   predictedMeans       - 每个锚点预测测距的加权平均值（numAnchors×1）
%   predictedUncertainties - 每个锚点测距的加权方差（numAnchors×1）
%   predictedRange       - 所有粒子对应的距离矩阵，维度numParticles×numAnchors

function [predictedMeans, predictedUncertainties, predictedRange] = predictMeasurements(predictedParticles, anchorPositions, weightsAnchor)

[~, numParticles, numAnchors] = size(anchorPositions); % 粒子数和锚点数

% 预分配输出变量
predictedMeans = zeros(numAnchors,1);
predictedUncertainties = zeros(numAnchors,1);
predictedRange = zeros(numParticles, numAnchors);

for anchor = 1:numAnchors
    % 计算每个粒子对应的移动体位置与锚点位置间的欧氏距离 (numParticles×1向量)
    predictedRange(:,anchor) = sqrt( ...
        (predictedParticles(1,:) - anchorPositions(1,:,anchor)).^2 + ...
        (predictedParticles(2,:) - anchorPositions(2,:,anchor)).^2 )';
    
    % 计算加权平均距离（预测测量均值）
    predictedMeans(anchor) = (predictedRange(:,anchor)' * weightsAnchor(:,anchor)) / sum(weightsAnchor(:,anchor));
    
    % 计算加权方差（预测测量不确定度）
    diff = predictedRange(:,anchor) - predictedMeans(anchor);
    predictedUncertainties(anchor) = ( (diff .* diff)' * weightsAnchor(:,anchor) ) / sum(weightsAnchor(:,anchor));
end

end
