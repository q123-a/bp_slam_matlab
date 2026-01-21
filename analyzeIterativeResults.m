% 迭代优化结果分析脚本
% Iterative Optimization Results Analysis Script
%
% 分析两轮迭代优化的性能对比
%
% 用法:
%   analyzeIterativeResults('results_iterative.mat')
%
% 输入:
%   filename - 结果文件名（.mat格式）
%
% 输出:
%   生成详细的性能对比分析和可视化图表
%
% Author: 2025-01

function analyzeIterativeResults(filename)

% 默认文件名
if nargin < 1
    filename = 'results_iterative.mat';
end

fprintf('\n========================================\n');
fprintf('迭代优化结果分析\n');
fprintf('========================================\n');
fprintf('加载文件: %s\n', filename);

% 加载结果
try
    data = load(filename);
catch
    error('无法加载文件 %s', filename);
end

% 提取数据
trueTrajectory = data.trueTrajectory;

% Round 1 数据
estimatedTrajectory1 = data.estimatedTrajectory1;
smoothedTrajectory1 = data.smoothedTrajectory1;
filterErrors1 = data.filterErrors1;
smoothErrors1 = data.smoothErrors1;
filterRMSE1 = data.filterRMSE1;
smoothRMSE1 = data.smoothRMSE1;

% Round 2 数据
estimatedTrajectory2 = data.estimatedTrajectory2;
smoothedTrajectory2 = data.smoothedTrajectory2;
filterErrors2 = data.filterErrors2;
smoothErrors2 = data.smoothErrors2;
filterRMSE2 = data.filterRMSE2;
smoothRMSE2 = data.smoothRMSE2;

numSteps = size(trueTrajectory, 2);

fprintf('\n========================================\n');
fprintf('Round 1 性能统计\n');
fprintf('========================================\n');
fprintf('  滤波 RMSE: %.6f m\n', filterRMSE1);
fprintf('  平滑 RMSE: %.6f m\n', smoothRMSE1);

% 计算 Round 1 改善
improvement1 = (filterRMSE1 - smoothRMSE1) / filterRMSE1 * 100;
fprintf('  改善比例: %.2f%%\n', improvement1);

% 计算 Round 1 改善的步数
improvedSteps1 = sum(smoothErrors1 < filterErrors1);
improvedRatio1 = improvedSteps1 / numSteps * 100;
fprintf('  改善步数: %d/%d (%.1f%%)\n', improvedSteps1, numSteps, improvedRatio1);

fprintf('\n========================================\n');
fprintf('Round 2 性能统计\n');
fprintf('========================================\n');
fprintf('  滤波 RMSE: %.6f m\n', filterRMSE2);
fprintf('  平滑 RMSE: %.6f m\n', smoothRMSE2);

% 计算 Round 2 改善
improvement2 = (filterRMSE2 - smoothRMSE2) / filterRMSE2 * 100;
fprintf('  改善比例: %.2f%%\n', improvement2);

% 计算 Round 2 改善的步数
improvedSteps2 = sum(smoothErrors2 < filterErrors2);
improvedRatio2 = improvedSteps2 / numSteps * 100;
fprintf('  改善步数: %d/%d (%.1f%%)\n', improvedSteps2, numSteps, improvedRatio2);

fprintf('\n========================================\n');
fprintf('Round 1 vs Round 2 对比\n');

fprintf('========================================\n');

% 滤波性能对比
filterImprovement = (filterRMSE1 - filterRMSE2) / filterRMSE1 * 100;
fprintf('  滤波 RMSE 改善: %.2f%% (R1: %.6f → R2: %.6f)\n', ...
    filterImprovement, filterRMSE1, filterRMSE2);

% 平滑性能对比
smoothImprovement = (smoothRMSE1 - smoothRMSE2) / smoothRMSE1 * 100;
fprintf('  平滑 RMSE 改善: %.2f%% (R1: %.6f → R2: %.6f)\n', ...
    smoothImprovement, smoothRMSE1, smoothRMSE2);

% 改善比例对比
fprintf('  改善比例提升: %.2f%% → %.2f%% (提升 %.2f%%)\n', ...
    improvedRatio1, improvedRatio2, improvedRatio2 - improvedRatio1);

fprintf('\n========================================\n');
fprintf('关键发现\n');
fprintf('========================================\n');

if filterRMSE2 < filterRMSE1
    fprintf('✓ Round 2 滤波性能优于 Round 1 (地图先验有效)\n');
else
    fprintf('✗ Round 2 滤波性能未改善\n');
end

if smoothRMSE2 < smoothRMSE1
    fprintf('✓ Round 2 平滑性能优于 Round 1\n');
else
    fprintf('✗ Round 2 平滑性能未改善\n');
end

if improvedRatio2 > improvedRatio1
    fprintf('✓ Round 2 改善步数比例更高 (%.1f%% vs %.1f%%)\n', ...
        improvedRatio2, improvedRatio1);
else
    fprintf('✗ Round 2 改善步数比例未提升\n');
end

fprintf('========================================\n');

% 生成可视化图表
generatePlots(trueTrajectory, ...
    estimatedTrajectory1, smoothedTrajectory1, filterErrors1, smoothErrors1, ...
    estimatedTrajectory2, smoothedTrajectory2, filterErrors2, smoothErrors2);

end


function generatePlots(trueTrajectory, ...
    estTraj1, smoothTraj1, filterErr1, smoothErr1, ...
    estTraj2, smoothTraj2, filterErr2, smoothErr2)
% 生成可视化图表

fprintf('\n生成可视化图表...\n');

figure('Position', [100, 100, 1800, 1000]);

% 子图1: Round 1 轨迹对比
subplot(2, 3, 1);
plot(trueTrajectory(1,:), trueTrajectory(2,:), 'k-', 'LineWidth', 2.5, 'DisplayName', 'Ground Truth');
hold on;
plot(estTraj1(1,:), estTraj1(2,:), 'b--', 'LineWidth', 1.5, 'DisplayName', 'R1 Filter');
plot(smoothTraj1(1,:), smoothTraj1(2,:), 'r-', 'LineWidth', 2, 'DisplayName', 'R1 Smoother');
xlabel('X [m]');
ylabel('Y [m]');
title('Round 1: Trajectory Comparison');
legend('Location', 'best');
grid on;
axis equal;

% 子图2: Round 2 轨迹对比
subplot(2, 3, 2);
plot(trueTrajectory(1,:), trueTrajectory(2,:), 'k-', 'LineWidth', 2.5, 'DisplayName', 'Ground Truth');
hold on;
plot(estTraj2(1,:), estTraj2(2,:), 'b--', 'LineWidth', 1.5, 'DisplayName', 'R2 Filter');
plot(smoothTraj2(1,:), smoothTraj2(2,:), 'r-', 'LineWidth', 2, 'DisplayName', 'R2 Smoother');
xlabel('X [m]');
ylabel('Y [m]');
title('Round 2: Trajectory Comparison (with Map Prior)');
legend('Location', 'best');
grid on;
axis equal;

% 子图3: 误差对比 - Round 1
subplot(2, 3, 3);
plot(filterErr1, 'b-', 'LineWidth', 1.5, 'DisplayName', 'R1 Filter');
hold on;
plot(smoothErr1, 'r-', 'LineWidth', 2, 'DisplayName', 'R1 Smoother');
xlabel('Time Step');
ylabel('Position Error [m]');
title('Round 1: Error vs Time');
legend('Location', 'best');
grid on;

% 子图4: 误差对比 - Round 2
subplot(2, 3, 4);
plot(filterErr2, 'b-', 'LineWidth', 1.5, 'DisplayName', 'R2 Filter');
hold on;
plot(smoothErr2, 'r-', 'LineWidth', 2, 'DisplayName', 'R2 Smoother');
xlabel('Time Step');
ylabel('Position Error [m]');
title('Round 2: Error vs Time');
legend('Location', 'best');
grid on;

% 子图5: 滤波性能对比 (R1 vs R2)
subplot(2, 3, 5);
plot(filterErr1, 'b--', 'LineWidth', 1.5, 'DisplayName', 'R1 Filter');
hold on;
plot(filterErr2, 'b-', 'LineWidth', 2, 'DisplayName', 'R2 Filter (with Prior)');
xlabel('Time Step');
ylabel('Position Error [m]');
title('Filter Performance: R1 vs R2');
legend('Location', 'best');
grid on;

% 子图6: 平滑性能对比 (R1 vs R2)
subplot(2, 3, 6);
plot(smoothErr1, 'r--', 'LineWidth', 1.5, 'DisplayName', 'R1 Smoother');
hold on;
plot(smoothErr2, 'r-', 'LineWidth', 2, 'DisplayName', 'R2 Smoother');
xlabel('Time Step');
ylabel('Position Error [m]');
title('Smoother Performance: R1 vs R2');
legend('Location', 'best');
grid on;

% 保存图表
saveas(gcf, 'iterative_results_analysis.png');
fprintf('✓ 分析图表已保存到 iterative_results_analysis.png\n');

end
