% 平滑性能分析脚本
% Smoothing Performance Analysis Script
%
% 分析后向粒子平滑的性能，生成详细的可视化结果
%
% 用法:
%   analyzeSmoothing('results_matlab.mat')
%
% 输入:
%   filename - 结果文件名（.mat格式）
%
% 输出:
%   生成6个子图的分析图表，保存为 smoothing_analysis.png
%
% Author: 2025

function analyzeSmoothing(filename)

% 默认文件名
if nargin < 1
    filename = 'results_matlab.mat';
end

fprintf('\n========================================\n');
fprintf('平滑性能分析\n');
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
estimatedTrajectory = data.estimatedTrajectory;
smoothedTrajectory = data.smoothedTrajectory;
filterErrors = data.filterErrors;
smoothErrors = data.smoothErrors;
filterRMSE = data.filterRMSE;
smoothRMSE = data.smoothRMSE;

numSteps = size(trueTrajectory, 2);

fprintf('\n数据统计:\n');
fprintf('  时间步数: %d\n', numSteps);
fprintf('  滤波 RMSE: %.6f m\n', filterRMSE);
fprintf('  平滑 RMSE: %.6f m\n', smoothRMSE);

% 计算改善百分比
improvement = (filterRMSE - smoothRMSE) / filterRMSE * 100;
if improvement > 0
    fprintf('  改善: %.2f%% ✓\n', improvement);
else
    fprintf('  恶化: %.2f%% ✗\n', -improvement);
end

% 计算其他统计指标
maxFilterError = max(filterErrors);
maxSmoothError = max(smoothErrors);
meanFilterError = mean(filterErrors);
meanSmoothError = mean(smoothErrors);
stdFilterError = std(filterErrors);
stdSmoothError = std(smoothErrors);

fprintf('\n详细统计:\n');
fprintf('  最大误差: 滤波=%.6f m, 平滑=%.6f m, 改善=%.2f%%\n', ...
    maxFilterError, maxSmoothError, (maxFilterError-maxSmoothError)/maxFilterError*100);
fprintf('  平均误差: 滤波=%.6f m, 平滑=%.6f m, 改善=%.2f%%\n', ...
    meanFilterError, meanSmoothError, (meanFilterError-meanSmoothError)/meanFilterError*100);
fprintf('  标准差: 滤波=%.6f m, 平滑=%.6f m\n', stdFilterError, stdSmoothError);

% 计算改善的时间步比例
improvedSteps = sum(smoothErrors < filterErrors);
improvedRatio = improvedSteps / numSteps * 100;
fprintf('  改善步数: %d/%d (%.1f%%)\n', improvedSteps, numSteps, improvedRatio);

% 创建可视化图表
fprintf('\n生成可视化图表...\n');
figure('Position', [100, 100, 1400, 900]);

% 子图1: 轨迹对比
subplot(2, 3, 1);
plot(trueTrajectory(1,:), trueTrajectory(2,:), 'k-', 'LineWidth', 2, 'DisplayName', '真实轨迹');
hold on;
plot(estimatedTrajectory(1,:), estimatedTrajectory(2,:), 'b--', 'LineWidth', 1.5, 'DisplayName', '滤波轨迹');
plot(smoothedTrajectory(1,:), smoothedTrajectory(2,:), 'r:', 'LineWidth', 1.5, 'DisplayName', '平滑轨迹');
xlabel('X (m)');
ylabel('Y (m)');
title('轨迹对比');
legend('Location', 'best');
grid on;
axis equal;

% 子图2: 误差随时间变化
subplot(2, 3, 2);
plot(1:numSteps, filterErrors, 'b-', 'LineWidth', 1.5, 'DisplayName', '滤波误差');
hold on;
plot(1:numSteps, smoothErrors, 'r-', 'LineWidth', 1.5, 'DisplayName', '平滑误差');
xlabel('时间步');
ylabel('位置误差 (m)');
title('误差随时间变化');
legend('Location', 'best');
grid on;

% 子图3: 误差改善量
subplot(2, 3, 3);
errorDiff = filterErrors - smoothErrors;
plot(1:numSteps, errorDiff, 'g-', 'LineWidth', 1.5);
hold on;
plot([1, numSteps], [0, 0], 'k--', 'LineWidth', 1);
xlabel('时间步');
ylabel('误差改善 (m)');
title('误差改善量 (正值表示改善)');
grid on;

% 子图4: 误差直方图对比
subplot(2, 3, 4);
histogram(filterErrors, 30, 'FaceColor', 'b', 'FaceAlpha', 0.5, 'DisplayName', '滤波');
hold on;
histogram(smoothErrors, 30, 'FaceColor', 'r', 'FaceAlpha', 0.5, 'DisplayName', '平滑');
xlabel('位置误差 (m)');
ylabel('频数');
title('误差分布直方图');
legend('Location', 'best');
grid on;

% 子图5: 累积分布函数 (CDF)
subplot(2, 3, 5);
[fFilter, xFilter] = ecdf(filterErrors);
[fSmooth, xSmooth] = ecdf(smoothErrors);
plot(xFilter, fFilter, 'b-', 'LineWidth', 2, 'DisplayName', '滤波');
hold on;
plot(xSmooth, fSmooth, 'r-', 'LineWidth', 2, 'DisplayName', '平滑');
xlabel('位置误差 (m)');
ylabel('累积概率');
title('误差累积分布函数 (CDF)');
legend('Location', 'best');
grid on;

% 子图6: 统计摘要
subplot(2, 3, 6);
axis off;
textStr = sprintf(['统计摘要\n\n' ...
    '滤波 RMSE: %.6f m\n' ...
    '平滑 RMSE: %.6f m\n' ...
    '改善: %.2f%%\n\n' ...
    '最大误差改善: %.2f%%\n' ...
    '平均误差改善: %.2f%%\n\n' ...
    '改善步数: %d/%d (%.1f%%)'], ...
    filterRMSE, smoothRMSE, improvement, ...
    (maxFilterError-maxSmoothError)/maxFilterError*100, ...
    (meanFilterError-meanSmoothError)/meanFilterError*100, ...
    improvedSteps, numSteps, improvedRatio);
text(0.1, 0.5, textStr, 'FontSize', 12, 'VerticalAlignment', 'middle');

% 保存图表
saveas(gcf, 'smoothing_analysis.png');
fprintf('图表已保存为 smoothing_analysis.png\n');

fprintf('\n分析完成！\n');

end
