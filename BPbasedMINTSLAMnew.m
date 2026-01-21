% Florian Meyer, Erik Leitinger, 20/05/17.
%
% 基于信念传播的多路径SLAM算法核心函数
% 输入:
%   dataVA              - 虚拟锚点数据
%   clutteredMeasurements- 带误报的测量数据（距离+方差）
%   parameters          - 算法参数结构体
%   trueTrajectory      - 真实轨迹，用于误差计算（已知轨迹模式）
% 输出:
%   estimatedTrajectory          - 估计的移动体状态轨迹（位置+速度）
%   estimatedAnchors             - 估计的锚点位置和存在概率
%   posteriorParticlesAnchorsstorage - 存储部分时刻锚点粒子用于分析
%   numEstimatedAnchors          - 每时刻估计的锚点数量
%   historyParticles             - 历史粒子集合（用于后向平滑）
%   historyWeights               - 历史权重集合（用于后向平滑）

function [ estimatedTrajectory, estimatedAnchors, posteriorParticlesAnchorsstorage, numEstimatedAnchors, historyParticles, historyWeights ] =  BPbasedMINTSLAMnew( dataVA, clutteredMeasurements, parameters, trueTrajectory )

% 获取测量时间步数和传感器数量
[numSteps,numSensors] = size(clutteredMeasurements);
% 限制最大时间步数
numSteps = min(numSteps,parameters.maxSteps);
% 读取参数
numParticles = parameters.numParticles;
detectionProbability = parameters.detectionProbability;
priorMean = parameters.priorMean;
survivalProbability = parameters.survivalProbability;
undetectedAnchorsIntensity = parameters.undetectedAnchorsIntensity*ones(numSensors,1);
birthIntensity = parameters.birthIntensity;
clutterIntensity = parameters.clutterIntensity;
unreliabilityThreshold = parameters.unreliabilityThreshold;
execTimePerStep = zeros(numSteps,1);
known_track = parameters.known_track;

load scen_semroom_new; % 加载场景（如果需要）

% 预分配存储空间
estimatedTrajectory = zeros(4,numSteps); % 状态空间4维：x,y,vx,vy
numEstimatedAnchors = zeros(2,numSteps); % 记录两个传感器各自锚点数量
storing_idx = 30:30:numSteps;            % 每30步存储一次锚点粒子状态
% 确保最后一步总是被保存（用于地图先验）
if ~any(storing_idx == numSteps)
    storing_idx = [storing_idx, numSteps];
end
posteriorParticlesAnchorsstorage = cell(1,length(storing_idx));

% 初始化历史存储（用于后向平滑）
historyParticles = cell(numSteps, 1);
historyWeights = cell(numSteps, 1);

% 初始化移动体粒子
if(known_track)
  % 已知轨迹时，所有粒子初始化为真实轨迹状态，速度为0
  posteriorParticlesAgent = repmat([trueTrajectory(:,1);0;0],1,numParticles);
else
  % 未知轨迹时，均匀采样位置粒子，速度粒子随机采样
  posteriorParticlesAgent(1:2,:) = drawSamplesUniformlyCirc(priorMean(1:2), parameters.UniformRadius_pos ,parameters.numParticles);
  posteriorParticlesAgent(3:4,:) = repmat(priorMean(3:4),1,parameters.numParticles) + 2*parameters.UniformRadius_vel * rand( 2, parameters.numParticles ) - parameters.UniformRadius_vel;  
end
% 记录初始状态估计（粒子均值）
estimatedTrajectory(:,1) = mean(posteriorParticlesAgent,2);

% 保存第0步的初始状态（用于后向平滑）
initWeights = ones(numParticles, 1) / numParticles;
historyParticles{1} = posteriorParticlesAgent;
historyWeights{1} = initWeights;

% 初始化锚点状态（位置粒子和权重）
% 检查是否有先验地图
if isfield(parameters, 'priorMap') && ~isempty(parameters.priorMap)
    % 使用先验地图初始化（Round 2）
    [ estimatedAnchors, posteriorParticlesAnchors ] = ...
        initAnchorsWithPrior(parameters, dataVA, numSteps, numSensors, parameters.priorMap);
else
    % 标准初始化（Round 1）
    [ estimatedAnchors, posteriorParticlesAnchors ] = ...
        initAnchors(parameters, dataVA, numSteps, numSensors);
end
for sensor = 1:numSensors
  numEstimatedAnchors(sensor, 1) = size(estimatedAnchors{sensor,1},2);
end

%% 主循环，遍历每个时间步
for step = 2:numSteps
  tic % 计时开始

  % 预测移动体状态
  if(known_track)
    % 已知轨迹，粒子直接用真实轨迹
    predictedParticlesAgent = repmat([trueTrajectory(:,step);0;0],1,numParticles);
  else
    % 未知轨迹时，基于动力学模型预测粒子状态
    predictedParticlesAgent = performPrediction( posteriorParticlesAgent, parameters );

    % 如果存在先验轨迹，则融合先验信息
    if isfield(parameters, 'priorTrajectory') && ~isempty(parameters.priorTrajectory)
      priorWeight = 0.3; % 默认先验权重
      if isfield(parameters, 'priorWeight')
        priorWeight = parameters.priorWeight;
      end

      % 从先验轨迹中提取当前时刻的状态
      priorState = parameters.priorTrajectory(:, step);

      % ===== 关键改进：避免"数据乱伦"和"粒子坍缩" =====
      % 问题1：数据乱伦 - 先验轨迹来自Round 1的测量Z，Round 2又用Z更新
      % 问题2：粒子坍缩 - 所有粒子被拉向同一点，失去多样性
      %
      % 解决方案：添加额外噪声（1.5倍标准噪声）保持粒子活性

      % 噪声放大因子（防止过拟合）
      noiseFactor = 1.5;

      % 获取运动模型的转移矩阵
      [A_prior, W_prior] = getTransitionMatrices(parameters.scanTime);

      % 计算过程噪声标准差
      processNoiseStd = sqrt(parameters.drivingNoiseVariance);

      for p = 1:numParticles
        % 1. 计算融合均值
        meanPos = (1 - priorWeight) * predictedParticlesAgent(:, p) + priorWeight * priorState;

        % 2. 生成额外噪声（在加速度空间，保持物理一致性）
        accelNoise = noiseFactor * processNoiseStd * randn(2, 1);
        stateNoise = W_prior * accelNoise;

        % 3. 最终粒子 = 融合均值 + 额外噪声
        predictedParticlesAgent(:, p) = meanPos + stateNoise;
      end

      if step == 2
        fprintf('  [先验注入] 权重=%.2f, 噪声因子=%.2f (保持粒子多样性)\n', priorWeight, noiseFactor);
      end
    end
  end

  % 初始化存储每个粒子每个传感器权重的矩阵
  weightsSensors = nan(numParticles,numSensors);

  % 对每个传感器进行锚点估计和数据关联更新
  for sensor = 1:numSensors
    % 继承上一时刻估计的锚点状态
    estimatedAnchors{sensor,step} = estimatedAnchors{sensor,step-1};
    measurements = clutteredMeasurements{step,sensor}; % 当前时刻传感器测量
    numMeasurements = size(measurements,2);            % 测量个数
    
    % 预测未检测锚点强度（存活概率衰减+新锚点出生强度）
    undetectedAnchorsIntensity(sensor) = undetectedAnchorsIntensity(sensor) * survivalProbability + birthIntensity;
    
    % 预测“遗留”锚点粒子状态及权重
    [predictedParticlesAnchors, weightsAnchor] = predictAnchors( posteriorParticlesAnchors{sensor}, parameters );
    
    % 针对每个测量生成新锚点粒子（新特征）
    [newParticlesAnchors,newInputBP] = generateNewAnchors(measurements, undetectedAnchorsIntensity(sensor) , predictedParticlesAgent, parameters);
    
    % 预测由锚点到移动体的测量值及其不确定度
    [predictedMeasurements, predictedUncertainties, predictedRange] = predictMeasurements(predictedParticlesAgent, predictedParticlesAnchors, weightsAnchor);

    % 计算测量与锚点的数据关联概率
    [associationProbabilities, associationProbabilitiesNew, messagelhfRatios, messagesNew] = calculateAssociationProbabilitiesGA(measurements, predictedMeasurements, predictedUncertainties, weightsAnchor, newInputBP, parameters);
    
    % 对每个锚点计算粒子权重，结合检测概率和测量似然
    numAnchors = size(predictedParticlesAnchors,3);
    weights = zeros(numParticles,numAnchors);
    for anchor = 1:numAnchors
      % 初始化权重为未检测概率
      weights(:,anchor) = repmat((1-detectionProbability),numParticles,1);
      for measurement = 1:numMeasurements
        measurementVariance = measurements(2,measurement);
        % 计算似然因子，归一化处理（“BP风格”）
        factor = 1/sqrt(2*pi*measurementVariance)*detectionProbability/clutterIntensity;
        % 叠加每个测量对权重的贡献
        weights(:,anchor) = weights(:,anchor) + factor*messagelhfRatios(measurement,anchor)*exp(-1/(2*measurementVariance)*(measurements(1,measurement)-predictedRange(:,anchor)).^2);
      end
      
      % 计算该锚点预测存在概率
      predictedExistence = sum(weightsAnchor(:,anchor));
      
      % 计算锚点存在的后验概率
      aliveUpdate = sum(predictedExistence*1/numParticles*weights(:,anchor));
      deadUpdate = 1 - predictedExistence;
      posteriorParticlesAnchors{sensor}{anchor}.posteriorExistence = aliveUpdate/(aliveUpdate+deadUpdate);
      
      % 重采样粒子，依据权重调整粒子集合
      idx_resampling = resampleSystematic(weights(:,anchor)/sum(weights(:,anchor)),numParticles);
      posteriorParticlesAnchors{sensor}{anchor}.x = predictedParticlesAnchors(:,idx_resampling(1:numParticles),anchor);
      posteriorParticlesAnchors{sensor}{anchor}.w = posteriorParticlesAnchors{sensor}{anchor}.posteriorExistence/numParticles*ones(numParticles,1);
      
      % 计算锚点位置的均值估计和存在概率
      estimatedAnchors{sensor,step}{anchor}.x = mean(posteriorParticlesAnchors{sensor}{anchor}.x,2);
      estimatedAnchors{sensor,step}{anchor}.posteriorExistence = posteriorParticlesAnchors{sensor}{anchor}.posteriorExistence;
      
      % 计算归一化权重的对数，防止数值溢出
      weights(:,anchor) = predictedExistence*weights(:,anchor) + deadUpdate;
      weights(:,anchor) = log(weights(:,anchor));
      weights(:,anchor) = weights(:,anchor) - max(weights(:,anchor));      
    end
    
    % 更新锚点数量
    numEstimatedAnchors(sensor, step) = size(estimatedAnchors{sensor,step},2);
    
    % 汇总所有锚点权重，为移动体粒子加权
    weightsSensors(:,sensor) = sum(weights,2);
    weightsSensors(:,sensor) = weightsSensors(:,sensor) - max(weightsSensors(:,sensor));
  
    % 更新未检测锚点强度，乘以未检测概率
    undetectedAnchorsIntensity(sensor) = undetectedAnchorsIntensity(sensor) * (1-parameters.detectionProbability);
    
    % 更新新锚点的后验存在概率和粒子集
    for measurement = 1:numMeasurements
      posteriorParticlesAnchors{sensor}{numAnchors+measurement}.posteriorExistence = messagesNew(measurement)*newParticlesAnchors(measurement).constant/(messagesNew(measurement)*newParticlesAnchors(measurement).constant + 1);
      posteriorParticlesAnchors{sensor}{numAnchors+measurement}.x = newParticlesAnchors(measurement).x;
      posteriorParticlesAnchors{sensor}{numAnchors+measurement}.w = posteriorParticlesAnchors{sensor}{numAnchors+measurement}.posteriorExistence/numParticles;
      estimatedAnchors{sensor,step}{numAnchors+measurement}.x = mean(newParticlesAnchors(measurement).x,2);
      estimatedAnchors{sensor,step}{numAnchors+measurement}.posteriorExistence = posteriorParticlesAnchors{sensor}{numAnchors+measurement}.posteriorExistence;
      estimatedAnchors{sensor,step}{numAnchors+measurement}.generatedAt = step; % 记录生成时间
    end
    
    % 删除存在概率低于阈值的不可靠锚点，控制复杂度
    [estimatedAnchors{sensor,step}, posteriorParticlesAnchors{sensor}] = deleteUnreliableVA( estimatedAnchors{sensor,step}, posteriorParticlesAnchors{sensor}, unreliabilityThreshold );
    numEstimatedAnchors(sensor, step) = size(estimatedAnchors{sensor,step},2);
  end
  
  % 汇总所有传感器权重，归一化移动体粒子权重
  weightsSensors = sum(weightsSensors,2);
  weightsSensors = weightsSensors - max(weightsSensors);
  weightsSensors = exp(weightsSensors);
  weightsSensors = weightsSensors/sum(weightsSensors);

  % 保存当前时刻的粒子和权重（必须在重采样之前！）
  historyParticles{step} = predictedParticlesAgent;
  historyWeights{step} = weightsSensors;

  % 保存部分关键时间步的锚点粒子状态，用于分析
  if(any(storing_idx == step))
    posteriorParticlesAnchorsstorage{storing_idx == step} = posteriorParticlesAnchors;
  end
  
  % 更新移动体估计轨迹
  if(known_track)
    % 已知轨迹时，直接用预测粒子均值
    estimatedTrajectory(:,step) = mean(predictedParticlesAgent,2);
    posteriorParticlesAgent = predictedParticlesAgent;
  else
    % 未知轨迹时，基于权重重采样粒子，更新估计
    estimatedTrajectory(:,step) = predictedParticlesAgent*weightsSensors;
    posteriorParticlesAgent = predictedParticlesAgent(:,resampleSystematic(weightsSensors,numParticles));
  end
  
  % 计算估计误差（距离误差）并打印输出
  execTimePerStep(step) = toc;
  error_agent = calcDistance_(trueTrajectory(1:2,step),estimatedTrajectory(1:2,step));
  fprintf('Time instance: %d \n',step);
  fprintf('Number of Anchors Sensor 1: %d \n',numEstimatedAnchors(1, step));
  fprintf('Number of Anchors Sensor 2: %d \n',numEstimatedAnchors(2, step));
  fprintf('Position error agent: %d \n',error_agent);
  fprintf('Execution Time: %4.4f \n',execTimePerStep(step));
  fprintf('--------------------------------------------------- \n\n')
end

end
