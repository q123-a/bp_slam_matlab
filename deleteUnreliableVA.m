function [estimatedAnchors, posteriorParticlesAnchors] = deleteUnreliableVA( estimatedAnchors, posteriorParticlesAnchors, unreliabilityThreshold )
% deleteUnreliableVA - 删除存在概率低于阈值的不可靠锚点
%
% 输入：
%   estimatedAnchors           - 当前时刻所有锚点的估计结构体数组
%   posteriorParticlesAnchors  - 锚点对应的粒子后验结构体数组
%   unreliabilityThreshold    - 存在概率阈值，低于该值锚点被删除
%
% 输出：
%   estimatedAnchors           - 删除不可靠锚点后的锚点估计数组
%   posteriorParticlesAnchors  - 删除不可靠锚点后的锚点粒子后验数组

numAnchors = size(posteriorParticlesAnchors,2);  % 当前锚点总数

unreliableAnchors = []; % 记录不可靠锚点的索引
for anchor = 1:numAnchors
  priorExistence = posteriorParticlesAnchors{anchor}.posteriorExistence; % 该锚点存在概率
  if(priorExistence < unreliabilityThreshold)
    % 如果存在概率低于阈值，标记为不可靠
    unreliableAnchors = [unreliableAnchors anchor];
  end
end

% 如果存在不可靠锚点，则从数组中删除它们
if(size(unreliableAnchors,2))
  posteriorParticlesAnchors(unreliableAnchors) = [];
  estimatedAnchors(unreliableAnchors) = [];
end

end
