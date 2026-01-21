% Florian Meyer, 20/05/17.
%
% 计算测量与锚点之间的关联概率，采用高斯近似 (GA)
%
% 输入参数：
%   measurements         - 当前时刻的测量值矩阵（2×M），第一行为测距，第二行为测距方差
%   predictedMeasurements- 预测的锚点对应的测距（1×L）
%   predictedUncertaintys - 预测测量的不确定度（方差，1×L）
%   weightsAnchor        - 锚点粒子的权重（存在概率的粒子权重和，NxL）
%   newInputBP           - 新锚点的输入消息，用于处理新锚点关联
%   parameters           - 参数结构体，包含检测概率Pd、杂波强度等
%
% 输出参数：
%   associationProbabilities    - 现有锚点与测量的关联概率矩阵
%   associationProbabilitiesNew - 新锚点与测量的关联概率矩阵
%   messagelegacy              - 发送给锚点的消息（旧锚点）
%   messagesNew                - 发送给新锚点的消息

function [associationProbabilities, associationProbabilitiesNew, messagelegacy, messagesNew] = calculateAssociationProbabilitiesGA(measurements, predictedMeasurements, predictedUncertaintys, weightsAnchor, newInputBP, parameters)

% 读取参数
detectionProbability = parameters.detectionProbability;   % 检测概率 Pd
clutterIntensity = parameters.clutterIntensity;           % 杂波强度 λ_c

[~, numMeasurements] = size(measurements);                 % 测量数目 M
numAnchors = length(predictedMeasurements);                % 锚点数目 L

% 计算每个锚点的存在概率（所有粒子权重之和）
predictedExistence = sum(weightsAnchor,1);

% 初始化输入消息矩阵 inputBP (M+1)×L 
% 第一行对应锚点未被检测（未关联任何测量）
inputBP = zeros(numMeasurements+1,numAnchors);
inputBP(1,:) = (1-detectionProbability); % 未检测概率

% 遍历所有锚点
for anchor =  1:numAnchors
  % 遍历所有测量
  for measurement = 1:numMeasurements
    % 预测测量和实际测量的联合方差（不确定度相加）
    predictedUncertaintyTmp = predictedUncertaintys(anchor) + measurements(2,measurement);
    
    % 计算高斯似然概率密度（归一化后）
    % Williams风格(注释)：factor = 1/sqrt(2*pi*predictedUncertaintyTmp)*detectionProbability;
    % BP风格（归一化了杂波强度，效果更稳定）
    factor = 1/sqrt(2*pi*predictedUncertaintyTmp)*detectionProbability/clutterIntensity;
    
    % 根据距离差计算似然概率，存入消息矩阵
    inputBP(measurement+1,anchor) = factor * exp(-1/(2*predictedUncertaintyTmp)*(measurements(1,measurement)-predictedMeasurements(anchor)).^2);
  end
  
  % 结合锚点存在概率，调用辅助函数getInputBP，计算该锚点的最终输入消息
  inputBP(:,anchor) = getInputBP(predictedExistence(anchor), inputBP(:,anchor));
end

% 调用信念传播函数，进行迭代推断，得到关联概率和消息
% 参数：最大迭代次数30，收敛阈值10^-6，消息最大值10^6避免数值溢出
[associationProbabilities, associationProbabilitiesNew, messagelegacy, messagesNew] = dataAssociationBP( inputBP, newInputBP, 30, 1e-6, 1e6 );

end
