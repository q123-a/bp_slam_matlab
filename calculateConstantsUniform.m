function [constants] = calculateConstantsUniform(predictedParticlesAgent, newMeasurements, parameters)
% calculateConstantsUniform - 计算新锚点生成时的归一化常数
%
% 输入:
%   predictedParticlesAgent - 预测的移动体粒子位置 (4 x N粒子)，只用前2维位置
%   newMeasurements         - 新测量值矩阵（距离及方差，2 x M测量数）
%   parameters              - 参数结构体，包含区域大小、上采样因子、粒子数等
%
% 输出:
%   constants - 每个测量对应的归一化常数向量 (M x 1)

% 定义感兴趣区域面积（假设是一个边长为2*regionOfInterestSize的正方形）
regionOfInterest = (2*parameters.regionOfInterestSize)^2;

% 上采样因子（提升粒子数以提高估计精度）
upSamplingFactor = parameters.upSamplingFactor;

% 计算总粒子数
numParticles = parameters.numParticles * upSamplingFactor;

% 复制移动体粒子位置，使其与上采样粒子数匹配
predictedParticlesAgent = repmat(predictedParticlesAgent, [1, upSamplingFactor]);

% 在感兴趣区域内均匀采样随机粒子（2维位置）
particles = 2*parameters.regionOfInterestSize*rand(2,numParticles) - parameters.regionOfInterestSize;

% 均匀分布权重常数（区域面积的倒数）
constantWeight = 1 / regionOfInterest;

% 测量数目
numMeasurements = length(newMeasurements(1,:));

% 初始化常数向量
constants = zeros(numMeasurements,1);

% 计算所有均匀采样点到移动体粒子预测位置的预测距离（欧氏距离）
predictedRange = sqrt( (particles(1,:) - predictedParticlesAgent(1,:)).^2 + (particles(2,:) - predictedParticlesAgent(2,:)).^2 );

% 遍历每个测量，计算归一化常数
for measurement = 1:numMeasurements
    % 计算测量方差对应的高斯似然常数因子
    constantLikelihood = 1 / sqrt(2*pi*newMeasurements(2,measurement));
    
    % 计算该测量与预测距离的高斯似然概率，并对所有粒子求平均（Monte Carlo积分）
    constants(measurement) = sum( (1/numParticles) * constantLikelihood * ...
        exp( (-1/2) * (repmat(newMeasurements(1,measurement),1,numParticles) - predictedRange).^2 / newMeasurements(2,measurement) ) );
end

% 除以均匀分布权重，完成归一化
constants = constants / constantWeight;

end
