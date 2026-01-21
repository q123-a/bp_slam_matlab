function [ A, W ] = getTransitionMatrices( scanTime )
% getTransitionMatrices - 生成状态转移矩阵和过程噪声矩阵
%
% 输入：
%   scanTime - 两次状态更新之间的时间间隔（采样时间）
%
% 输出：
%   A - 状态转移矩阵 (4x4)，用于预测下一状态
%   W - 过程噪声输入矩阵 (4x2)，用于将过程噪声映射到状态空间
%
% 状态变量假设为4维：[位置_x; 位置_y; 速度_x; 速度_y]

% 初始化为单位矩阵（4×4）
A = diag(ones(4,1));

% 位置受速度影响，位置更新方程中包含速度乘以时间间隔
A(1,3) = scanTime; % x位置随x速度变化
A(2,4) = scanTime; % y位置随y速度变化

% 过程噪声输入矩阵W，将二维加速度噪声映射到4维状态空间
W = zeros(4,2);
W(1,1) = 0.5 * scanTime^2; % 位置x受加速度x影响，积分关系0.5*t^2
W(2,2) = 0.5 * scanTime^2; % 位置y受加速度y影响
W(3,1) = scanTime;         % 速度x受加速度x影响，积分关系t
W(4,2) = scanTime;         % 速度y受加速度y影响

end
