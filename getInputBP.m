function [ messageOut ] = getInputBP( existence, messageTargetIsPresent)
% getInputBP - 结合锚点存在概率，计算输入信念传播消息
%
% 输入：
%   existence            - 锚点存在的概率（标量，范围[0,1]）
%   messageTargetIsPresent- 当锚点存在时测量的消息向量（长度为测量数+1）
%
% 输出：
%   messageOut           - 结合存在概率后的消息向量，用于信念传播

[lenMessage, ~] = size(messageTargetIsPresent);

% 当锚点不存在时的消息，初始化为0向量，只有第一项（未检测）为1
messageTargetIsAbsent = zeros(lenMessage,1);
messageTargetIsAbsent(1) = 1;

% 根据存在概率加权合成消息
% messageOut = existence * messageTargetIsPresent + (1 - existence) * messageTargetIsAbsent
% 体现了贝叶斯混合：锚点存在与否的加权消息
messageOut = existence * messageTargetIsPresent + (1 - existence) * messageTargetIsAbsent;

end
