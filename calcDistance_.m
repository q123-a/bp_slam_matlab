function d = calcDistance_(p1, p2, p, mode, c)
% calcDistance_ - 计算点集之间的p阶距离，支持多种模式
%
% 语法：
%   d = calcDistance_(p1, p2)                  % 默认欧氏距离（p=2）
%   d = calcDistance_(p1, p2, p)               % 指定范数阶数p
%   d = calcDistance_(p1, p2, p, mode)         % 指定模式mode
%   d = calcDistance_(p1, p2, p, mode, c)      % mode=2时指定截断距离c
%
% 输入：
%   p1, p2 - dxN矩阵，列为N个d维点。p1和p2列数相等，或p1为1列，p2为多列
%   p      - 范数阶数（默认为2，即欧几里得距离）
%   mode   - 计算模式（可选）：
%            1 - 常规p阶距离（默认）
%            2 - 截断距离，结果距离最大为c
%            3 - 计算欧式距离矩阵，输出N×M矩阵
%   c      - 截断距离阈值，仅mode=2时有效
%
% 输出：
%   d - 计算得到的距离向量或矩阵
%
% 作者：Paul Meissner, SPSC Lab, 2010/11/13

% 如果没传入p，则默认使用2（欧氏距离）
if(nargin == 2 || isempty(p))   
  p = 2;
end

% 如果任一输入为空，返回NaN
if( isempty(p1) || isempty(p2))
  d = nan;
  return;
end

% 如果mode没给，默认1（常规距离）
if(nargin < 4)       
  mode = 1;
elseif(nargin == 4 && mode == 2)
  % 如果mode=2但没给截断值c，报错
  error('Cutoff mode selected without specifying cutoff value!')
end
  
% 获取点的数量（列数）
N = size(p1, 2);
M = size(p2, 2);

% 如果p1和p2点数不匹配且不是单点对多点模式，且不是模式3，报错
if( (N~=M) && (N>1) && mode ~= 3  )  
  error('Size of input vectors incorrect!')
end

% 如果p1只有1列而p2有多列，则复制p1使两者列数一致
if( (N == 1) && (M>N) )      
  p1 = p1*ones(1,M);
end

% 模式3，计算欧式距离矩阵（N×M）
if(mode == 3)  
  d = zeros(N, M);
  for n = 1:N
    for m = n+1:M
       % 计算p阶范数距离
       d(n,m) =  norm(p1(:,n)-p2(:,m), p);
    end
  end
  % 补全对称矩阵
  d = triu(d)+triu(d)';
  % 如果给了截断值c，执行截断
  if( nargin == 5)
    d = min(c, d);
  end
  return
end

% 常规距离计算，按列计算范数
d = sum(abs(p1-p2).^p, 1).^(1/p);

% 如果模式为2，执行截断距离
if( mode == 2)   
  d = min(c, d);
end

% 下面代码是注释掉的逐点计算版本，效率较低，已弃用
% %Calculate distance
% d = zeros(1,M);
% for i = 1:M
%   d(i) = norm(p1(:,i)-p2(:,i), p);
%   if( mode == 2)
%     d(i) = min(c, d(i));
%   end
% end

end
