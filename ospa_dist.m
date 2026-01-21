function [dist, varargout] = ospa_dist(X, Y, c, p)
% ospa_dist - 计算两个有限点集的OSPA距离
%
% 输入：
%   X, Y - 点集，2维矩阵，列为各点坐标
%   c    - 截断参数，控制最大距离的惩罚
%   p    - 指标阶数，用于计算距离的p阶范数
%
% 输出：
%   dist       - 标量，X和Y之间的OSPA距离
%   varargout{1} - (可选)位置误差部分
%   varargout{2} - (可选)基数误差部分
%
% 说明：
%   OSPA距离统一考虑了位置误差和基数误差（元素个数差异），
%   适用于多目标估计性能评估。
%
% 参考文献：
%   Schuhmacher et al., IEEE Trans. Signal Processing, 2008.

% 检查输出参数个数
if nargout ~= 1 && nargout ~= 3
   error('Incorrect number of outputs'); 
end

% 两个集合均为空，距离为0
if isempty(X) && isempty(Y)
    dist = 0;
    if nargout == 3
        varargout{1} = 0; % 位置误差
        varargout{2} = 0; % 基数误差
    end
    return;
end

% 其中一个集合为空，距离为截断参数c（最大惩罚）
if isempty(X) || isempty(Y)
    dist = c;
    if nargout == 3
        varargout{1} = 0; % 位置误差为0
        varargout{2} = c; % 基数误差为c
    end
    return;
end

% 计算两个集合的点数
n = size(X, 2);
m = size(Y, 2);

% 计算两集合所有点对的欧氏距离矩阵D（大小 n×m）
XX = repmat(X, [1 m]);                % 扩展X以匹配Y点数
YY = reshape(repmat(Y, [n 1]), [size(Y,1) n*m]); % 重复Y以匹配X点数
D = reshape(sqrt(sum((XX - YY).^2)), [n m]); % 欧氏距离
D = min(c, D).^p;                     % 截断并做p次方

% 求解最优匹配（最小权匹配），调用匈牙利算法
[assignment, cost] = Hungarian(D);

% 计算最终OSPA距离
dist = ( 1/max(m,n) * ( c^p * abs(m-n) + cost ) )^(1/p);

% 如果需要，输出位置误差和基数误差
if nargout == 3
    varargout{1} = (1/max(m,n) * cost)^(1/p);          % 位置误差
    varargout{2} = (1/max(m,n) * c^p * abs(m-n))^(1/p); % 基数误差
end

end
