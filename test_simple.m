% 简单测试脚本 - 验证环境是否正常
fprintf('测试开始...\n');
fprintf('当前目录: %s\n', pwd);

% 检查文件
if exist('testbed_iterative.m', 'file')
    fprintf('✓ 找到 testbed_iterative.m\n');
else
    fprintf('✗ 未找到 testbed_iterative.m\n');
end

if exist('BPbasedMINTSLAMnew.m', 'file')
    fprintf('✓ 找到 BPbasedMINTSLAMnew.m\n');
else
    fprintf('✗ 未找到 BPbasedMINTSLAMnew.m\n');
end

if exist('scenarioCleanM2_new.mat', 'file')
    fprintf('✓ 找到 scenarioCleanM2_new.mat\n');
else
    fprintf('✗ 未找到 scenarioCleanM2_new.mat\n');
end

fprintf('\n如果所有文件都找到了，请运行:\n');
fprintf('  testbed_iterative\n');
