% Florian Meyer, 19/04/17
%
% 根据目标轨迹和虚拟锚点数据生成带噪声的测量值
%
% 输入：
%   targetTrajectory - 目标（移动体）的轨迹，状态矩阵（至少包含二维位置，大小2×numSteps）
%   dataVA           - 虚拟锚点数据结构体数组，长度为传感器数量，每个包含锚点位置和可见性
%   parameters       - 参数结构体，包含测量方差等信息
%
% 输出：
%   measurementsCell - 生成的测量数据cell数组，大小为numSteps×numSensors，
%                      每个元素是2×numVisibleAnchors矩阵（距离和方差）

function [ measurementsCell ] = generateMeasurements(targetTrajectory, dataVA, parameters)

measurementVarianceRange = parameters.measurementVariance; % 距离测量方差

[~, numSteps] = size(targetTrajectory);    % 时间步数
numSensors = length(dataVA);                % 传感器数量

% 初始化测量存储cell
measurementsCell = cell(numSteps,numSensors);

% 遍历每个传感器
for sensor = 1:numSensors
  positions = dataVA{sensor}.positions;    % 当前传感器所有锚点位置（2×numAnchors）
  visibility = dataVA{sensor}.visibility;  % 锚点在每个时间步的可见性矩阵（numAnchors×numSteps）
  
  % 遍历每个时间步
  for step = 1:numSteps
    k = 0;                                % 计数当前时刻可见锚点数
    [~, numAnchors] = size(positions);   % 锚点数量
    measurements = zeros(2,numAnchors);  % 预分配测距矩阵（距离+方差）
    
    % 遍历所有锚点
    for anchor = 1:numAnchors
      if(visibility(anchor,step))         % 该锚点在当前时刻可见时
        k = k + 1;
        % 赋予测距方差（固定）
        measurements(2,k) = measurementVarianceRange;
        % 计算实际距离（欧氏距离） + 加入高斯噪声
        measurements(1,k) = sqrt( (positions(1,anchor) - targetTrajectory(1,step))^2 + ...
                                 (positions(2,anchor) - targetTrajectory(2,step))^2 ) ...
                             + sqrt(measurements(2,k)) * randn;
        % 更新测量方差为另一个参数（可认为是测量噪声的后验方差）
        measurements(2,k) = parameters.measurementVarianceLHF;
      end
    end
    
    % 只保留当前时刻可见锚点的测量数据
    measurementsCell{step,sensor} = measurements(:,1:k);
  end
end

end
