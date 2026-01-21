% 对比基线、Round 1 和 Round 2 的性能
% Compare Baseline, Round 1, and Round 2 Performance
%
% 用法:
%   compareWithBaseline('results_matlabfirst.mat', 'results_iterative.mat')
%
% 输入:
%   baselineFile  - 基线结果文件（没有平滑的原始滤波）
%   iterativeFile - 迭代优化结果文件（包含 R1 和 R2）
%
% Author: 2025-01

function compareWithBaseline(baselineFile, iterativeFile)

% 默认文件名
if nargin < 1
    baselineFile = 'results_matlabfirst.mat';
end
if nargin < 2
    iterativeFile = 'results_iterative.mat';
end

fprintf('\n========================================\n');
fprintf('基线 vs 平滑优化 性能对比\n');
fprintf('========================================\n');

% 加载基线结果
fprintf('加载基线文件: %s\n', baselineFile);
try
    baseline = load(baselineFile);
catch
    error('无法加载基线文件 %s', baselineFile);
end

% 加载迭代优化结果
fprintf('加载迭代文件: %s\n', iterativeFile);
try
    iterative = load(iterativeFile);
catch
    error('无法加载迭代文件 %s', iterativeFile);
end

% 提取基线数据
if isfield(baseline, 'estimatedTrajectory')
    baselineTrajectory = baseline.estimatedTrajectory;
    trueTrajectory = baseline.trueTrajectory;
elseif isfield(baseline, 'estimatedTrajectory1')
    % 如果基线文件也是迭代格式，使用 R1 的滤波结果
    baselineTrajectory = baseline.estimatedTrajectory1;
    trueTrajectory = baseline.trueTrajectory;
else
    error('基线文件格式不正确');
end

% 计算基线误差
numSteps = size(trueTrajectory, 2);
baselineErrors = sqrt(sum((baselineTrajectory(1:2,:) - trueTrajectory(1:2,:)).^2, 1));
baselineRMSE = sqrt(mean(baselineErrors.^2));

% 提取 Round 1 数据
filterRMSE1 = iterative.filterRMSE1;
smoothRMSE1 = iterative.smoothRMSE1;
filterErrors1 = iterative.filterErrors1;
smoothErrors1 = iterative.smoothErrors1;

% 提取 Round 2 数据
filterRMSE2 = iterative.filterRMSE2;
smoothRMSE2 = iterative.smoothRMSE2;
filterErrors2 = iterative.filterErrors2;
smoothErrors2 = iterative.smoothErrors2;

fprintf('\n========================================\n');
fprintf('性能统计对比\n');
fprintf('========================================\n');

% 打印 RMSE 对比
fprintf('\n1. RMSE 对比:\n');
fprintf('   基线 (无平滑):     %.6f m\n', baselineRMSE);
fprintf('   R1 滤波:           %.6f m\n', filterRMSE1);
fprintf('   R1 平滑:           %.6f m\n', smoothRMSE1);
fprintf('   R2 滤波 (地图先验): %.6f m\n', filterRMSE2);
fprintf('   R2 平滑:           %.6f m\n', smoothRMSE2);

% 计算相对基线的改善
fprintf('\n2. 相对基线的改善:\n');
r1FilterImprovement = (baselineRMSE - filterRMSE1) / baselineRMSE * 100;
r1SmoothImprovement = (baselineRMSE - smoothRMSE1) / baselineRMSE * 100;
r2FilterImprovement = (baselineRMSE - filterRMSE2) / baselineRMSE * 100;
r2SmoothImprovement = (baselineRMSE - smoothRMSE2) / baselineRMSE * 100;

fprintf('   R1 滤波:           %.2f%%\n', r1FilterImprovement);
fprintf('   R1 平滑:           %.2f%% ⭐\n', r1SmoothImprovement);
fprintf('   R2 滤波 (地图先验): %.2f%%\n', r2FilterImprovement);
fprintf('   R2 平滑:           %.2f%% ⭐⭐\n', r2SmoothImprovement);

% 计算增量改善
fprintf('\n3. 增量改善 (相对于前一步):\n');
r1SmoothVsFilter = (filterRMSE1 - smoothRMSE1) / filterRMSE1 * 100;
r2FilterVsR1Smooth = (smoothRMSE1 - filterRMSE2) / smoothRMSE1 * 100;
r2SmoothVsFilter = (filterRMSE2 - smoothRMSE2) / filterRMSE2 * 100;

fprintf('   R1 平滑 vs R1 滤波:  %.2f%%\n', r1SmoothVsFilter);
fprintf('   R2 滤波 vs R1 平滑:  %.2f%% (地图先验效果)\n', r2FilterVsR1Smooth);
fprintf('   R2 平滑 vs R2 滤波:  %.2f%%\n', r2SmoothVsFilter);

% 总体改善
fprintf('\n4. 总体改善 (基线 → 最终):\n');
totalImprovement = (baselineRMSE - smoothRMSE2) / baselineRMSE * 100;
fprintf('   基线 → R2 平滑:     %.2f%%\n', totalImprovement);
fprintf('   绝对改善:          %.6f m → %.6f m\n', baselineRMSE, smoothRMSE2);
fprintf('   误差降低:          %.6f m\n', baselineRMSE - smoothRMSE2);

fprintf('\n========================================\n');
fprintf('关键发现\n');
fprintf('========================================\n');

if r1SmoothImprovement > 10
    fprintf('✓ R1 平滑显著改善基线性能 (%.2f%%)\n', r1SmoothImprovement);
end

if r2FilterImprovement > r1FilterImprovement
    fprintf('✓ 地图先验有效：R2 滤波优于 R1 滤波\n');
end

if r2SmoothImprovement > r1SmoothImprovement
    fprintf('✓ R2 平滑达到最佳性能 (%.2f%% 改善)\n', r2SmoothImprovement);
end

fprintf('✓ 总体改善：%.2f%% (从 %.6f m 降至 %.6f m)\n', ...
    totalImprovement, baselineRMSE, smoothRMSE2);

fprintf('========================================\n');

% 生成可视化
generateComparisonPlot(baselineErrors, filterErrors1, smoothErrors1, ...
    filterErrors2, smoothErrors2, baselineRMSE, filterRMSE1, smoothRMSE1, ...
    filterRMSE2, smoothRMSE2);

end


function generateComparisonPlot(baselineErrors, filterErrors1, smoothErrors1, ...
    filterErrors2, smoothErrors2, baselineRMSE, filterRMSE1, smoothRMSE1, ...
    filterRMSE2, smoothRMSE2)
% 生成对比可视化图表

fprintf('\n生成可视化图表...\n');

figure('Position', [100, 100, 1600, 900]);

% 子图1: 误差随时间变化 - 所有方法对比
subplot(2, 3, 1);
plot(baselineErrors, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Baseline');
hold on;
plot(filterErrors1, 'b--', 'LineWidth', 1.2, 'DisplayName', 'R1 Filter');
plot(smoothErrors1, 'r-', 'LineWidth', 2, 'DisplayName', 'R1 Smooth');
plot(filterErrors2, 'g--', 'LineWidth', 1.2, 'DisplayName', 'R2 Filter');
plot(smoothErrors2, 'm-', 'LineWidth', 2, 'DisplayName', 'R2 Smooth');
xlabel('Time Step');
ylabel('Position Error [m]');
title('Error vs Time: All Methods');
legend('Location', 'best', 'FontSize', 8);
grid on;

% 子图2: RMSE 柱状图对比
subplot(2, 3, 2);
methods = {'Baseline', 'R1 Filter', 'R1 Smooth', 'R2 Filter', 'R2 Smooth'};
rmseValues = [baselineRMSE, filterRMSE1, smoothRMSE1, filterRMSE2, smoothRMSE2];
colors = [0 0 0; 0 0 1; 1 0 0; 0 0.5 0; 1 0 1];
bar(rmseValues, 'FaceColor', 'flat', 'CData', colors);
set(gca, 'XTickLabel', methods, 'XTickLabelRotation', 45);
ylabel('RMSE [m]');
title('RMSE Comparison');
grid on;

% 子图3: 相对基线的改善百分比
subplot(2, 3, 3);
improvements = [(baselineRMSE - filterRMSE1)/baselineRMSE*100, ...
                (baselineRMSE - smoothRMSE1)/baselineRMSE*100, ...
                (baselineRMSE - filterRMSE2)/baselineRMSE*100, ...
                (baselineRMSE - smoothRMSE2)/baselineRMSE*100];
bar(improvements);
set(gca, 'XTickLabel', {'R1 Filter', 'R1 Smooth', 'R2 Filter', 'R2 Smooth'}, ...
    'XTickLabelRotation', 45);
ylabel('Improvement [%]');
title('Improvement vs Baseline');
grid on;

% 子图4: 基线 vs R1 平滑
subplot(2, 3, 4);
plot(baselineErrors, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Baseline');
hold on;
plot(smoothErrors1, 'r-', 'LineWidth', 2, 'DisplayName', 'R1 Smooth');
xlabel('Time Step');
ylabel('Position Error [m]');
title('Baseline vs R1 Smooth');
legend('Location', 'best');
grid on;

% 子图6: 最终对比 - 基线 vs R2 平滑
subplot(2, 3, 6);
plot(baselineErrors, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Baseline');
hold on;
plot(smoothErrors2, 'm-', 'LineWidth', 2.5, 'DisplayName', 'R2 Smooth (Final)');
xlabel('Time Step');
ylabel('Position Error [m]');
title('Final Result: Baseline vs R2 Smooth');
legend('Location', 'best');
grid on;

% 保存图表
saveas(gcf, 'baseline_comparison.png');
fprintf('✓ 对比图表已保存到 baseline_comparison.png\n');

end
