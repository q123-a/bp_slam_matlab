function pos = drawSamplesUniformlyCirc(posCenter, radius, N)
% drawSamplesUniformlyCirc - 在二维圆形区域内均匀采样点
%
% 输入参数：
%   posCenter - 采样圆的中心坐标，2×1向量 [x_center; y_center]
%   radius    - 采样圆的半径
%   N         - 需要采样的点数
%
% 输出参数：
%   pos       - 采样点坐标，2×N矩阵，每列是一个采样点的 [x; y]

pos = zeros(2,N);              % 预分配采样点矩阵
phi = 2*pi*rand(1,N);          % 采样角度，均匀分布在[0, 2π)
r = sqrt(rand(1,N));           % 采样半径，经过开方确保在圆内均匀分布

% 根据极坐标转换为笛卡尔坐标，加上圆心坐标得到最终位置
pos(1,:) = (radius * r).*cos(phi) + posCenter(1);
pos(2,:) = (radius * r).*sin(phi) + posCenter(2);

end
