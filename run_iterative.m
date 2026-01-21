% 运行迭代优化测试的启动脚本
% Run Iterative Optimization Test - Launcher Script

% 清理工作空间
clear all; close all; clc;

% 获取当前脚本所在目录
scriptDir = fileparts(mfilename('fullpath'));

% 切换到脚本目录
cd(scriptDir);

% 添加必要的路径
addpath(scriptDir);

% 显示当前目录
fprintf('当前工作目录: %s\n', pwd);

% 检查必要文件是否存在
if ~exist('testbed_iterative.m', 'file')
    error('找不到 testbed_iterative.m 文件！');
end

if ~exist('scenarioCleanM2_new.mat', 'file')
    error('找不到 scenarioCleanM2_new.mat 数据文件！');
end

if ~exist('BPbasedMINTSLAMnew.m', 'file')
    error('找不到 BPbasedMINTSLAMnew.m 函数文件！');
end

if ~exist('runBackwardSmoothing.m', 'file')
    error('找不到 runBackwardSmoothing.m 函数文件！');
end

fprintf('所有必要文件检查完毕，开始运行...\n\n');

% 运行迭代优化脚本
run('testbed_iterative.m');
