% 调试 dataVA 结构问题
clear; clc;

fprintf('=== 调试 dataVA 结构 ===\n\n');

% 加载数据
fprintf('1. 加载数据...\n');
load('scenarioCleanM2_new.mat', 'dataVA', 'trueTrajectory');

% 检查 dataVA 基本信息
fprintf('\n2. dataVA 基本信息:\n');
fprintf('   类型: %s\n', class(dataVA));
fprintf('   大小: [%d, %d]\n', size(dataVA, 1), size(dataVA, 2));
fprintf('   是否为 cell: %d\n', iscell(dataVA));

% 检查每个传感器
fprintf('\n3. 检查每个传感器:\n');
[numSensors, ~] = size(dataVA);
fprintf('   传感器数量: %d\n', numSensors);

for sensor = 1:numSensors
    fprintf('\n   传感器 %d:\n', sensor);

    % 尝试访问
    try
        sensorData = dataVA{sensor};
        fprintf('     访问成功!\n');
        fprintf('     类型: %s\n', class(sensorData));

        % 检查是否有 positions 字段
        if isstruct(sensorData)
            fprintf('     字段: %s\n', strjoin(fieldnames(sensorData), ', '));

            % 检查 positions
            if isfield(sensorData, 'positions')
                posSize = size(sensorData.positions);
                fprintf('     positions 大小: [%d, %d]\n', posSize(1), posSize(2));
                fprintf('     positions 前3列:\n');
                disp(sensorData.positions(:, 1:min(3, posSize(2))));
            end
        end
    catch ME
        fprintf('     访问失败: %s\n', ME.message);
    end
end

% 测试 priorKnownAnchors 访问
fprintf('\n4. 测试 priorKnownAnchors 访问:\n');
parameters.priorKnownAnchors = {[1], [1]};

for sensor = 1:numSensors
    fprintf('\n   传感器 %d:\n', sensor);
    fprintf('     priorKnownAnchors{%d} = %s\n', sensor, mat2str(parameters.priorKnownAnchors{sensor}));

    try
        anchorPositions = dataVA{sensor}.positions(:, parameters.priorKnownAnchors{sensor});
        fprintf('     访问成功! 大小: [%d, %d]\n', size(anchorPositions, 1), size(anchorPositions, 2));
        fprintf('     值:\n');
        disp(anchorPositions);
    catch ME
        fprintf('     访问失败: %s\n', ME.message);

        % 尝试其他方式
        fprintf('     尝试直接访问第1列...\n');
        try
            anchorPositions = dataVA{sensor}.positions(:, 1);
            fprintf('     成功! 值:\n');
            disp(anchorPositions);
        catch ME2
            fprintf('     仍然失败: %s\n', ME2.message);
        end
    end
end

fprintf('\n=== 调试完成 ===\n');
