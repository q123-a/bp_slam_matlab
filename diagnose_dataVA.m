% 诊断 dataVA 结构
% Diagnose dataVA structure

fprintf('加载数据...\n');
load('scenarioCleanM2_new.mat', 'dataVA', 'trueTrajectory');

fprintf('\n=== dataVA 基本信息 ===\n');
fprintf('类型: %s\n', class(dataVA));
fprintf('大小: %s\n', mat2str(size(dataVA)));
fprintf('是否为 cell: %d\n', iscell(dataVA));
fprintf('是否为 struct: %d\n', isstruct(dataVA));

fprintf('\n=== 尝试不同的访问方式 ===\n');

% 方式1: 直接索引
try
    fprintf('\n方式1: dataVA{1}\n');
    elem1 = dataVA{1};
    fprintf('  成功! 类型: %s, 大小: %s\n', class(elem1), mat2str(size(elem1)));
    if isstruct(elem1)
        fprintf('  字段: %s\n', strjoin(fieldnames(elem1), ', '));
    end
catch ME
    fprintf('  失败: %s\n', ME.message);
end

% 方式2: 双重索引
try
    fprintf('\n方式2: dataVA{1,1}\n');
    elem1 = dataVA{1,1};
    fprintf('  成功! 类型: %s, 大小: %s\n', class(elem1), mat2str(size(elem1)));
    if isstruct(elem1)
        fprintf('  字段: %s\n', strjoin(fieldnames(elem1), ', '));
    end
catch ME
    fprintf('  失败: %s\n', ME.message);
end

% 方式3: 作为结构体数组
try
    fprintf('\n方式3: dataVA(1)\n');
    elem1 = dataVA(1);
    fprintf('  成功! 类型: %s, 大小: %s\n', class(elem1), mat2str(size(elem1)));
    if isstruct(elem1)
        fprintf('  字段: %s\n', strjoin(fieldnames(elem1), ', '));
    end
catch ME
    fprintf('  失败: %s\n', ME.message);
end

% 找到正确的访问方式后，尝试访问 positions
fprintf('\n=== 尝试访问 positions 字段 ===\n');

% 尝试所有可能的组合
attempts = {
    'dataVA{1}.positions'
    'dataVA{1,1}.positions'
    'dataVA(1).positions'
    'dataVA{1}{1}.positions'
};

for i = 1:length(attempts)
    try
        fprintf('\n尝试 %d: %s\n', i, attempts{i});
        result = eval(attempts{i});
        fprintf('  成功! 大小: %s\n', mat2str(size(result)));
        fprintf('  前几个值:\n');
        disp(result(:, 1:min(3, size(result, 2))));
    catch ME
        fprintf('  失败: %s\n', ME.message);
    end
end

fprintf('\n诊断完成!\n');
