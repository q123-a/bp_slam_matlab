function [assocProbExisting,assocProbNew,messagelhfRatios,messagelhfRatiosNew] = dataAssociationBP( legacy, new, checkConvergence, threshold, numIterations )
% dataAssociationBP - 使用信念传播（BP）算法计算数据关联概率
%
% 输入：
%   legacy          - 现有锚点的输入消息矩阵，大小 (M+1)×N
%                     第一行为锚点未检测概率，后M行为测量与锚点的似然消息
%   new             - 新锚点的输入消息向量 (M×1)，对应每个测量
%   checkConvergence- 迭代次数间隔，用于判断收敛
%   threshold       - 收敛阈值，消息变化低于该值认为收敛
%   numIterations   - 最大迭代次数
%
% 输出：
%   assocProbExisting   - 现有锚点与测量的关联概率矩阵 (M+1)×N
%   assocProbNew        - 新锚点与测量的关联概率向量 (M×1)
%   messagelhfRatios    - 现有锚点对应的消息比率矩阵 M×N
%   messagelhfRatiosNew - 新锚点对应的消息比率向量 (M×1)

[m,n] = size(legacy); 
m = m - 1;  % 测量数 M （去掉未检测行）
assocProbNew = ones(m,1);            % 新锚点关联概率初始化
assocProbExisting = ones(m+1,n);     % 现有锚点关联概率初始化
messagelhfRatios = ones(m,n);        % 现有锚点消息比率初始化
messagelhfRatiosNew = ones(m,1);     % 新锚点消息比率初始化

% 如果无测量或无锚点，直接返回默认值
if(n==0 || m==0)
  return;
end

% 如果新锚点输入消息为空，初始化为1
if(isempty(new))
  new = 1;
end

% 全1向量辅助计算
om = ones(1,m);
on = ones(1,n);

% 初始化消息矩阵muba (M×N)，测量到锚点的消息
muba = ones(m,n);

% BP迭代循环
for iteration = 1:numIterations
  mubaOld = muba; % 保存上次消息，便于收敛判断
  
  % 计算每个锚点的消息乘积（测量消息 * 先验似然）
  prodfact = muba .* legacy(2:end,:);
  sumprod = legacy(1,:) + sum(prodfact,1); % 所有测量和未检测概率之和
  
  % 归一化因子，防止除零
  normalization = (sumprod(om,:) - prodfact);
  normalization(normalization == 0) = eps;
  
  % 计算锚点到测量的消息更新
  muab = legacy(2:end,:) ./ normalization;
  
  % 计算测量总和消息（包括新锚点消息）
  summuab = new + sum(muab,2);
  normalization = summuab(:,on) - muab;
  normalization(normalization == 0) = eps;
  
  % 更新测量到锚点的消息
  muba = 1 ./ normalization;
  
  % 每隔checkConvergence步判断是否收敛
  if(mod(iteration,checkConvergence) == 0)
    % 计算消息变化的最大对数比，判断是否小于阈值
    distance = max(max(abs(log(muba./mubaOld))));
    if(distance < threshold)
      break
    end
  end
end

% 计算关联概率，结合输入消息和更新后的消息
assocProbExisting(1,:) = legacy(1,:);            % 未检测概率保持不变
assocProbExisting(2:end,:) = legacy(2:end,:).*muba; % 关联测量概率更新

% 对每个锚点列归一化概率和为1
for target=1:n
  assocProbExisting(:,target) = assocProbExisting(:,target)/sum(assocProbExisting(:,target));
end

% 消息比率输出
messagelhfRatios = muba;
assocProbNew = new ./ summuab; % 新锚点关联概率计算

% 新锚点消息比率计算（归一化）
messagelhfRatiosNew = [ones(m,1), muab]; % 连接未检测与检测消息
messagelhfRatiosNew = messagelhfRatiosNew ./ repmat(sum(messagelhfRatiosNew,2), [1,n+1]);
messagelhfRatiosNew = messagelhfRatiosNew(:,1); % 取未检测消息部分作为输出

end
