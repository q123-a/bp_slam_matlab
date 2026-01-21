% Florian Meyer, 20/06/15
%
% 生成带有杂波和漏检的测量数据（仿真环境）
%
% 输入：
%   trueMeasurementsCell - 真实测量数据的cell数组，大小为[numSteps, numSensors]，
%                          每个元素是2×numAnchors矩阵（距离+方差）
%   parameters           - 参数结构体，包括测量方差、检测概率、杂波均值、区域大小等
%
% 输出：
%   clutteredMeasurements - 加入误报和漏检后的测量数据cell数组，大小同输入

function [clutteredMeasurements] = generateClutteredMeasurements(trueMeasurementsCell, parameters)

% 读取参数
measurementVarianceRange = parameters.measurementVariance; % 测距方差
detectionProbability = parameters.detectionProbability;     % 检测概率 Pd
meanNumberOfClutter = parameters.meanNumberOfClutter;       % 杂波（误报）均值λ_c
maxRange = parameters.regionOfInterestSize;                 % 区域最大测距范围

[numSteps, numSensors] = size(trueMeasurementsCell);        % 时间步和传感器数

% 初始化输出cell数组
clutteredMeasurements = cell(numSteps, numSensors);

% 遍历每个传感器和时间步
for sensor = 1:numSensors
    for step = 1:numSteps
        trueMeasurements = trueMeasurementsCell{step,sensor}; % 真实测量（2×锚点数）
        [~, numAnchors] = size(trueMeasurements);
        
        % 按检测概率随机决定哪些锚点被检测到（漏检处理）
        detectionIndicator = (rand(numAnchors,1) < detectionProbability);
        
        % 提取被检测到的测量
        detectedMeasurements = squeeze(trueMeasurements(:,detectionIndicator));
        
        % 生成误报（杂波）数量，符合泊松分布
        numFalseAlarms = poissrnd(meanNumberOfClutter);
        
        % 生成误报测量，初始化为0
        falseAlarms = zeros(2,numFalseAlarms);
        if(~isempty(falseAlarms))
            % 误报距离均匀分布在0到maxRange
            falseAlarms(1,:) = maxRange * rand(numFalseAlarms,1);
            % 误报测量方差为测距方差
            falseAlarms(2,:) = measurementVarianceRange;
        end
        
        % 将误报和真实检测测量拼接
        clutteredMeasurement = [falseAlarms, detectedMeasurements];
        
        % 随机打乱测量顺序，模拟实际测量的无序性
        clutteredMeasurement = clutteredMeasurement(:, randperm(numFalseAlarms + sum(detectionIndicator)));
        
        % 保存当前时间步传感器的测量
        clutteredMeasurements{step,sensor} = clutteredMeasurement;
    end
end

end
