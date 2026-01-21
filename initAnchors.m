function [ estimatedAnchors, posteriorParticlesAnchors ] = initAnchors( parameters, dataVA, numSteps, numSensors )
% initAnchors - 初始化物理锚点和几何锚点的粒子集及状态估计
%
% 输入：
%   parameters          - 参数结构体，包含粒子数、先验协方差、已知锚点索引等
%   dataVA              - 虚拟锚点数据结构体数组，包含锚点位置
%   numSteps            - 时间步总数
%   numSensors          - 传感器数量
%
% 输出：
%   estimatedAnchors         - 估计的锚点状态cell数组，大小为[numSensors,numSteps]
%   posteriorParticlesAnchors- 每个传感器锚点的粒子集合cell数组

numParticles = parameters.numParticles;  % 粒子数
posteriorParticlesAnchors = cell(numSensors,1); % 用于存储各传感器的锚点粒子
estimatedAnchors = cell(numSensors,numSteps);   % 用于存储各时间步锚点估计

% 初始化锚点cell结构，分配空间
for sensor = 1:numSensors
    % 获取该传感器已知锚点的初始位置
    anchorPositions = dataVA{sensor}.positions(:,parameters.priorKnownAnchors{sensor});
    [~, numAnchors] = size(anchorPositions);
    posteriorParticlesAnchors{sensor} = cell(1,numAnchors);
    estimatedAnchors{sensor,1} = cell(1,numAnchors);
end

priorCovarianceAnchor = parameters.priorCovarianceAnchor; % 先验协方差，用于采样

% 遍历每个传感器和锚点，初始化粒子集和估计
for sensor = 1:numSensors
    anchorPositions = dataVA{sensor}.positions(:,parameters.priorKnownAnchors{sensor});
    [~, numAnchors] = size(anchorPositions);
    for anchor = 1:numAnchors
        % 初始化粒子状态矩阵，维度2×numParticles（只含位置）
        posteriorParticlesAnchors{sensor}{anchor}.x = zeros(2,numParticles);
        % 初始化粒子权重向量
        posteriorParticlesAnchors{sensor}{anchor}.w = zeros(numParticles,1);
        % 初始锚点存在概率为1（完全存在）
        posteriorParticlesAnchors{sensor}{anchor}.posteriorExistence = 1;
        
        % 权重均匀分配
        posteriorParticlesAnchors{sensor}{anchor}.w(:) = posteriorParticlesAnchors{sensor}{anchor}.posteriorExistence / numParticles * ones(numParticles,1);
        
        if(anchor == 1)
            % 第一个锚点为物理锚点（PA）
            % 根据先验均值和协方差采样粒子位置，二维正态分布
            posteriorParticlesAnchors{sensor}{anchor}.x = mvnrnd(anchorPositions(:,anchor), priorCovarianceAnchor, numParticles)';
            % 估计位置为先验均值
            estimatedAnchors{sensor,1}{anchor}.x = anchorPositions(:,anchor);
            estimatedAnchors{sensor,1}{anchor}.posteriorExistence = posteriorParticlesAnchors{sensor}{anchor}.posteriorExistence;
            estimatedAnchors{sensor,1}{anchor}.generatedAt = 1; % 生成时间为初始时刻
        else
            % 几何锚点（虚拟锚点VA）
            % 先对先验均值采样一次扰动，增加随机性
            anchorPositions(:,anchor) = mvnrnd(anchorPositions(:,anchor), priorCovarianceAnchor, 1)';
            % 再基于扰动均值采样粒子
            posteriorParticlesAnchors{sensor}{anchor}.x = mvnrnd(anchorPositions(:,anchor), priorCovarianceAnchor, numParticles)';
            % 估计位置为扰动后的均值
            estimatedAnchors{sensor,1}{anchor}.x = anchorPositions(:,anchor);
            estimatedAnchors{sensor,1}{anchor}.posteriorExistence = posteriorParticlesAnchors{sensor}{anchor}.posteriorExistence;
            estimatedAnchors{sensor,1}{anchor}.generatedAt = 1;
        end
    end
end

end
