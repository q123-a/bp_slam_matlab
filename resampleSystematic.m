function [indx] = resampleSystematic(w, N)
% resampleSystematic - 基于系统重采样算法，从权重分布中采样粒子索引
%
% 输入：
%   w - 粒子权重向量（已归一化，和为1）
%   N - 需要采样的粒子数量
%
% 输出：
%   indx - 重采样后粒子的索引向量（长度N）
%
% 说明：
%   系统重采样通过在[0,1)区间均匀采样N个点并与累积权重比较，
%   实现低方差粒子重采样，是粒子滤波中常用的重采样策略。

indx = zeros(N,1);          % 初始化索引向量
Q = cumsum(w);              % 计算权重的累积分布函数（CDF）

% 在[0,1)区间生成N个均匀间隔采样点，起点带随机偏移
T = linspace(0, 1 - 1/N, N) + rand(1)/N;
T(N+1) = 1;                % 边界条件，方便索引比较

i = 1; % T指针
j = 1; % Q指针

while i <= N
    if T(i) < Q(j)
        indx(i) = j;  % 采样该粒子索引
        i = i + 1;    % 移动T指针
    else
        j = j + 1;    % 权重CDF指针前进
    end
end

end
