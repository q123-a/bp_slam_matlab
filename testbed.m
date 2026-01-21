clear variables; clc;
close all;

% ---------------------------
% 1. 通用参数及数据加载
% ---------------------------
parameters.known_track = 0;  % 是否已知轨迹（0表示未知轨迹）

load('scenarioCleanM2_new.mat'); % 加载场景数据，包括虚拟锚点 dataVA 和真实轨迹 trueTrajectory

% 将所有锚点的可见性设置为全可见（1）
[numSensors, ~] = size(dataVA);
for sensor = 1:numSensors
  dataVA{sensor}.visibility = ones(size(dataVA{sensor}.visibility,1), length(trueTrajectory));
end

% ---------------------------
% 2. 算法参数配置
% ---------------------------
parameters.maxSteps = 900;          % 最大时间步数
trueTrajectory = trueTrajectory(:,1:parameters.maxSteps); % 取前maxSteps个时间步的轨迹
parameters.lengthStep = 0.03;       % 单步移动距离（米）
parameters.scanTime = 1;             % 采样时间间隔（秒）

% 最大速度和过程噪声方差计算
v_max = parameters.lengthStep / parameters.scanTime;
parameters.drivingNoiseVariance = (v_max / 3 / parameters.scanTime)^2;

% 测量噪声参数
parameters.measurementVariance = 0.1^2;      % 距离测量方差
parameters.measurementVarianceLHF = 0.15^2;  % 后验测量方差（用于LHF）

% 检测概率
parameters.detectionProbability = 0.95;

% 区域尺寸及杂波相关参数
parameters.regionOfInterestSize = 30;               % 区域边长（米）
parameters.meanNumberOfClutter = 1;                  % 平均误报数
parameters.clutterIntensity = parameters.meanNumberOfClutter / parameters.regionOfInterestSize; % 杂波强度

% 新锚点出生率
parameters.meanNumberOfBirth = 1e-4;
parameters.birthIntensity = parameters.meanNumberOfBirth / (2 * parameters.regionOfInterestSize)^2;

% 未检测锚点强度
parameters.meanNumberOfUndetectedAnchors = 6;
parameters.undetectedAnchorsIntensity = parameters.meanNumberOfUndetectedAnchors / (2 * parameters.regionOfInterestSize)^2;

% 粒子滤波相关参数
parameters.numParticles = 100000;   % 粒子数量
parameters.upSamplingFactor = 1;    % 粒子上采样因子

% SLAM相关阈值与先验
parameters.detectionThreshold = 0.5;
parameters.survivalProbability = 0.999;  % 锚点存活概率
parameters.unreliabilityThreshold = 1e-4; % 锚点存在概率阈值，低于则删除
parameters.priorKnownAnchors{1} = 1;      % 传感器1已知锚点索引
parameters.priorKnownAnchors{2} = 1;      % 传感器2已知锚点索引
parameters.priorCovarianceAnchor = 0.001^2 * eye(2); % 锚点位置先验协方差
parameters.anchorRegularNoiseVariance = 1e-4^2;      % 锚点过程噪声方差

% agent参数（均匀采样半径）
parameters.UniformRadius_pos = 0.5;  % 初始位置均匀采样半径
parameters.UniformRadius_vel = 0.05; % 初始速度均匀采样半径

% ---------------------------
% 3. 随机种子设置（保证结果可重复）
% ---------------------------
rng(1)

% ---------------------------
% 4. 移动体初始位置均值设定（真实轨迹起点）
% ---------------------------
parameters.priorMean = [trueTrajectory(1:2,1); 0; 0]; % 初始位置+速度

% ---------------------------
% 5. 生成理想测量数据（无杂波）
% ---------------------------
measurements = generateMeasurements(trueTrajectory, dataVA, parameters);

% ---------------------------
% 6. 加入误报和漏检，生成带杂波测量
% ---------------------------
clutteredMeasurements = generateClutteredMeasurements(measurements, parameters);

% ---------------------------
% 7. 调用核心BP-SLAM算法进行估计
% ---------------------------
[estimatedTrajectory, estimatedAnchors, posteriorParticlesAnchors, numEstimatedAnchors, historyParticles, historyWeights] = ...
    BPbasedMINTSLAMnew(dataVA, clutteredMeasurements, parameters, trueTrajectory);

% ---------------------------
% 7.5 执行后向粒子平滑（可选）
% ---------------------------
fprintf('\n========================================\n');
fprintf('开始执行后向粒子平滑...\n');
fprintf('========================================\n');

% 调用平滑函数（默认使用10条轨迹）
smoothedTrajectory = runBackwardSmoothing(historyParticles, historyWeights, parameters, 50);

fprintf('平滑完成！\n\n');

% ---------------------------
% 8. 绘制结果（轨迹、锚点估计等）
% ---------------------------
plotAll(trueTrajectory, estimatedTrajectory, estimatedAnchors, posteriorParticlesAnchors{end}, numEstimatedAnchors, dataVA, parameters, 0, parameters.maxSteps);

% ---------------------------
% 9. 计算并比较滤波与平滑的误差
% ---------------------------
fprintf('\n========================================\n');
fprintf('误差分析\n');
fprintf('========================================\n');

% 计算滤波误差
filterErrors = sqrt(sum((trueTrajectory(1:2,:) - estimatedTrajectory(1:2,:)).^2, 1));
filterRMSE = sqrt(mean(filterErrors.^2));

% 计算平滑误差
smoothErrors = sqrt(sum((trueTrajectory(1:2,:) - smoothedTrajectory(1:2,:)).^2, 1));
smoothRMSE = sqrt(mean(smoothErrors.^2));

% 打印结果
fprintf('滤波 RMSE: %.6f m\n', filterRMSE);
fprintf('平滑 RMSE: %.6f m\n', smoothRMSE);
improvement = (filterRMSE - smoothRMSE) / filterRMSE * 100;
if improvement > 0
    fprintf('改善: %.2f%% ✓\n', improvement);
else
    fprintf('恶化: %.2f%% ✗\n', -improvement);
end

% ---------------------------
% 10. 保存结果
% ---------------------------
fprintf('\n保存结果到 results_matlab.mat...\n');
save('results_matlab.mat', 'trueTrajectory', 'estimatedTrajectory', ...
     'smoothedTrajectory', 'filterErrors', 'smoothErrors', ...
     'filterRMSE', 'smoothRMSE', 'parameters');
fprintf('结果已保存！\n');
