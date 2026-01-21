% BP-SLAM 迭代优化测试脚本
% Iterative BP-SLAM with Smoothing
%
% 简化迭代方案:
% - 2轮迭代
% - 每轮: 前向滤波 + 后向平滑(30条轨迹)
% - 第2轮使用第1轮的平滑轨迹作为先验
%
% 预期性能: 改善步数 65% → 68-72%
% 计算时间: 约36分钟

clear; clc;

% 添加当前目录到路径（确保能找到新函数）
addpath(pwd);

fprintf('========================================\n');
fprintf('BP-SLAM 迭代优化测试\n');
fprintf('========================================\n');

% ---------------------------
% 1. 加载数据和参数设置
% ---------------------------
fprintf('\n[步骤 1/6] 加载数据和参数...\n');

% 加载场景数据（加载所有变量）
load('scenarioCleanM2_new.mat');

% 初始化 dataVA 的可见性（必须在裁剪 trueTrajectory 之前！）
[numSensors, ~] = size(dataVA);
for sensor = 1:numSensors
    dataVA{sensor}.visibility = ones(size(dataVA{sensor}.visibility, 1), length(trueTrajectory));
end

% 基本参数
parameters.maxSteps = 900;
trueTrajectory = trueTrajectory(:, 1:parameters.maxSteps);
parameters.lengthStep = 0.03;
parameters.scanTime = 1;

% 速度和噪声参数
v_max = parameters.lengthStep / parameters.scanTime;
parameters.drivingNoiseVariance = (v_max / 3 / parameters.scanTime)^2;

% 测量噪声
parameters.measurementVariance = 0.1^2;
parameters.measurementVarianceLHF = 0.15^2;

% 检测和杂波参数
parameters.detectionProbability = 0.95;
parameters.regionOfInterestSize = 30;
parameters.meanNumberOfClutter = 1;
parameters.clutterIntensity = parameters.meanNumberOfClutter / parameters.regionOfInterestSize;

% 锚点出生和存活参数
parameters.meanNumberOfBirth = 1e-4;
parameters.birthIntensity = parameters.meanNumberOfBirth / (2 * parameters.regionOfInterestSize)^2;
parameters.meanNumberOfUndetectedAnchors = 6;
parameters.undetectedAnchorsIntensity = parameters.meanNumberOfUndetectedAnchors / ...
    (2 * parameters.regionOfInterestSize)^2;

% 粒子滤波参数
parameters.numParticles = 100000;
parameters.upSamplingFactor = 1;

% SLAM参数
parameters.detectionThreshold = 0.5;
parameters.survivalProbability = 0.999;
parameters.unreliabilityThreshold = 1e-4;
parameters.priorKnownAnchors{1} = 1;  % 传感器1已知锚点索引
parameters.priorKnownAnchors{2} = 1;  % 传感器2已知锚点索引
parameters.priorCovarianceAnchor = 0.001^2 * eye(2);
parameters.anchorRegularNoiseVariance = 1e-4^2;

% Agent参数
parameters.UniformRadius_pos = 0.5;
parameters.UniformRadius_vel = 0.05;

% 随机种子
rng(1);

% 初始位置
parameters.priorMean = [trueTrajectory(1:2, 1); 0; 0];

% 轨迹模式（false = 未知轨迹，需要估计）
parameters.known_track = false;

fprintf('  参数加载完成\n');
fprintf('  时间步数: %d\n', parameters.maxSteps);
fprintf('  粒子数量: %d\n', parameters.numParticles);

% ---------------------------
% 2. 生成测量数据
% ---------------------------
fprintf('\n[步骤 2/6] 生成测量数据...\n');

% 生成理想测量数据（无杂波）
measurements = generateMeasurements(trueTrajectory, dataVA, parameters);

% 加入误报和漏检，生成带杂波测量
clutteredMeasurements = generateClutteredMeasurements(measurements, parameters);

fprintf('  测量数据生成完成\n');

% ---------------------------
% 3. 第1轮迭代
% ---------------------------
fprintf('\n========================================\n');
fprintf('[步骤 3/6] 第1轮迭代\n');
fprintf('========================================\n');

fprintf('\n>>> 第1轮: 前向滤波...\n');
tic;
[estimatedTrajectory1, estimatedAnchors1, posteriorParticlesAnchors1, ...
 numEstimatedAnchors1, historyParticles1, historyWeights1] = ...
    BPbasedMINTSLAMnew(dataVA, clutteredMeasurements, parameters, trueTrajectory);
time_filter1 = toc;

fprintf('>>> 第1轮: 前向滤波完成 (耗时: %.1f 分钟)\n', time_filter1/60);

% 计算第1轮滤波误差
filterErrors1 = sqrt(sum((trueTrajectory(1:2,:) - estimatedTrajectory1(1:2,:)).^2, 1));
filterRMSE1 = sqrt(mean(filterErrors1.^2));
fprintf('>>> 第1轮: 滤波 RMSE = %.6f m\n', filterRMSE1);

fprintf('\n>>> 第1轮: 后向平滑 (30条轨迹)...\n');
tic;
smoothedTrajectory1 = runBackwardSmoothing(historyParticles1, historyWeights1, parameters, 30);
time_smooth1 = toc;

fprintf('>>> 第1轮: 后向平滑完成 (耗时: %.1f 分钟)\n', time_smooth1/60);

% 计算第1轮平滑误差
smoothErrors1 = sqrt(sum((trueTrajectory(1:2,:) - smoothedTrajectory1(1:2,:)).^2, 1));
smoothRMSE1 = sqrt(mean(smoothErrors1.^2));
improvedSteps1 = sum(smoothErrors1 < filterErrors1);
improvedRatio1 = improvedSteps1 / parameters.maxSteps * 100;

fprintf('>>> 第1轮: 平滑 RMSE = %.6f m\n', smoothRMSE1);
fprintf('>>> 第1轮: 改善步数 = %d/%d (%.1f%%)\n', ...
    improvedSteps1, parameters.maxSteps, improvedRatio1);
fprintf('>>> 第1轮总耗时: %.1f 分钟\n', (time_filter1 + time_smooth1)/60);

% 保存 Round 1 的地图作为先验
fprintf('\n>>> 第1轮: 保存地图先验...\n');
priorMap = savePriorMap(estimatedAnchors1, posteriorParticlesAnchors1, numSensors);

% ---------------------------
% 4. 第2轮迭代 (使用轨迹+地图先验)
% ---------------------------
fprintf('\n========================================\n');
fprintf('[步骤 4/6] 第2轮迭代 (使用轨迹+地图先验)\n');
fprintf('========================================\n');

% 修改参数: 使用轨迹+地图先验
parameters.priorTrajectory = smoothedTrajectory1;  % 轨迹先验
parameters.priorWeight = 0.3;  % 轨迹先验权重 (30%先验 + 70%运动模型)
parameters.priorMap = priorMap;  % 地图先验 (关键新增!)

fprintf('\n>>> 第2轮: 前向滤波 (使用轨迹+地图先验)...\n');
tic;
[estimatedTrajectory2, estimatedAnchors2, posteriorParticlesAnchors2, ...
 numEstimatedAnchors2, historyParticles2, historyWeights2] = ...
    BPbasedMINTSLAMnew(dataVA, clutteredMeasurements, parameters, trueTrajectory);
time_filter2 = toc;

fprintf('>>> 第2轮: 前向滤波完成 (耗时: %.1f 分钟)\n', time_filter2/60);

% 计算第2轮滤波误差
filterErrors2 = sqrt(sum((trueTrajectory(1:2,:) - estimatedTrajectory2(1:2,:)).^2, 1));
filterRMSE2 = sqrt(mean(filterErrors2.^2));
fprintf('>>> 第2轮: 滤波 RMSE = %.6f m\n', filterRMSE2);

fprintf('\n>>> 第2轮: 后向平滑 (30条轨迹)...\n');
tic;
smoothedTrajectory2 = runBackwardSmoothing(historyParticles2, historyWeights2, parameters, 30);
time_smooth2 = toc;

fprintf('>>> 第2轮: 后向平滑完成 (耗时: %.1f 分钟)\n', time_smooth2/60);

% 计算第2轮平滑误差
smoothErrors2 = sqrt(sum((trueTrajectory(1:2,:) - smoothedTrajectory2(1:2,:)).^2, 1));
smoothRMSE2 = sqrt(mean(smoothErrors2.^2));
improvedSteps2 = sum(smoothErrors2 < filterErrors2);
improvedRatio2 = improvedSteps2 / parameters.maxSteps * 100;

fprintf('>>> 第2轮: 平滑 RMSE = %.6f m\n', smoothRMSE2);
fprintf('>>> 第2轮: 改善步数 = %d/%d (%.1f%%)\n', ...
    improvedSteps2, parameters.maxSteps, improvedRatio2);
fprintf('>>> 第2轮总耗时: %.1f 分钟\n', (time_filter2 + time_smooth2)/60);

% ---------------------------
% 5. 结果对比
% ---------------------------
fprintf('\n========================================\n');
fprintf('[步骤 5/6] 迭代结果对比\n');
fprintf('========================================\n');

fprintf('\n性能对比:\n');
fprintf('  第1轮滤波 RMSE: %.6f m\n', filterRMSE1);
fprintf('  第1轮平滑 RMSE: %.6f m (改善步数: %.1f%%)\n', smoothRMSE1, improvedRatio1);
fprintf('  第2轮滤波 RMSE: %.6f m\n', filterRMSE2);
fprintf('  第2轮平滑 RMSE: %.6f m (改善步数: %.1f%%)\n', smoothRMSE2, improvedRatio2);

fprintf('\n改善幅度:\n');
improvement1 = (filterRMSE1 - smoothRMSE1) / filterRMSE1 * 100;
improvement2 = (filterRMSE2 - smoothRMSE2) / filterRMSE2 * 100;
fprintf('  第1轮: %.2f%%\n', improvement1);
fprintf('  第2轮: %.2f%%\n', improvement2);

fprintf('\n总耗时:\n');
totalTime = time_filter1 + time_smooth1 + time_filter2 + time_smooth2;
fprintf('  总计: %.1f 分钟\n', totalTime/60);

% ---------------------------
% 6. 保存结果
% ---------------------------
fprintf('\n========================================\n');
fprintf('[步骤 6/6] 保存结果\n');
fprintf('========================================\n');

% 保存所有结果
save('results_iterative.mat', ...
    'trueTrajectory', ...
    'estimatedTrajectory1', 'smoothedTrajectory1', ...
    'estimatedTrajectory2', 'smoothedTrajectory2', ...
    'filterErrors1', 'smoothErrors1', ...
    'filterErrors2', 'smoothErrors2', ...
    'filterRMSE1', 'smoothRMSE1', ...
    'filterRMSE2', 'smoothRMSE2', ...
    'improvedRatio1', 'improvedRatio2', ...
    'parameters');

fprintf('  结果已保存到 results_iterative.mat\n');

fprintf('\n========================================\n');
fprintf('迭代优化完成!\n');
fprintf('========================================\n');
fprintf('最终结果: 平滑 RMSE = %.6f m, 改善步数 = %.1f%%\n', ...
    smoothRMSE2, improvedRatio2);
