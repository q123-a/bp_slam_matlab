% Florian Meyer, Erik Leitinger, 20/05/17.
%
% 根据新测量生成新锚点粒子及对应的信念传播输入消息
%
% 输入：
%   newMeasurements          - 新测量矩阵，2×M（距离+测量方差）
%   undetectedTargetsIntensity - 未检测锚点强度（先验出生率）
%   predictedParticlesAgent  - 预测的移动体粒子状态，4×numParticles
%   parameters              - 参数结构体，含杂波强度、检测概率、粒子数等
%
% 输出：
%   newParticlesAnchors      - 新锚点粒子结构体数组，每个包含状态x、权重w、存在概率posteriorExistence
%   inputBP                 - 对应新锚点的输入消息，用于信念传播数据关联

function [newParticlesAnchors,inputBP] = generateNewAnchors(newMeasurements, undetectedTargetsIntensity, predictedParticlesAgent, parameters)

clutterIntensity = parameters.clutterIntensity;      % 杂波强度 λ_c
numParticles = parameters.numParticles;              % 粒子数
numMeasurements = length(newMeasurements(1,:));     % 新测量数量 M
detectionProbability = parameters.detectionProbability; % 检测概率 Pd

inputBP = zeros(numMeasurements,1);                   % 初始化新锚点输入消息
newParticlesAnchors = [];

% 如果存在测量，则初始化结构体数组
if(numMeasurements)
    newParticlesAnchors = struct('x', zeros(4,numParticles), 'w', zeros(numParticles,1), 'posteriorExistence', 0);
end

% 计算每个新测量的归一化常数，用于权重计算（基于均匀分布蒙特卡洛积分）
constants = calculateConstantsUniform(predictedParticlesAgent, newMeasurements, parameters);

for measurement = 1:numMeasurements
    % 计算新锚点的信念传播输入消息
    % Williams风格（注释）：inputBP(measurement) = clutterIntensity + constants(measurement) * undetectedTargetsIntensity * detectionProbability;
    % BP风格（归一化）：下式更稳定，数值表现更好
    inputBP(measurement) = 1 + (constants(measurement) * undetectedTargetsIntensity * detectionProbability) / clutterIntensity;
    
    % 提取测量距离及方差
    measurementToAnchor = newMeasurements(1,measurement);
    measurementVariance = newMeasurements(2,measurement);
    
    % 根据测量似然采样新锚点粒子状态（4维，含位置和速度）
    newParticlesAnchors(measurement).x = sampleFromLikelihood(measurementToAnchor, measurementVariance, predictedParticlesAgent, numParticles);
    
    % 计算常数（用于存在概率计算）
    newParticlesAnchors(measurement).constant = constants(measurement) * undetectedTargetsIntensity * detectionProbability / clutterIntensity;
    
    % 初始化粒子权重均匀分布
    newParticlesAnchors(measurement).w = ones(numParticles,1) / numParticles;
end

end
