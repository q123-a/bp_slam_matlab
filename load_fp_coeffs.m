% Coeffs for wall segments in VALOUR
%
% 本文件定义了室内墙壁段材质的权重系数和连接类型，
% 用于多径信号传播建模（例如虚拟锚点产生、信号阻挡等）。
%
% 注意：本文件应在加载每个楼层平面图时载入。
%
% 作者：Paul Meissner，SPSC实验室，格拉茨工业大学
% 日期：2011年1月25日

% 材质权重定义说明：
% 所有权重大于0的材质会阻挡信号传播，
% 权重小于等于0的材质不会阻挡信号，但可能导致反射（产生虚拟锚点VA）。

fp_coeffs.trans = -1;           % 透射和反射材质（产生虚拟锚点VA，但不阻挡信号）
fp_coeffs.unspec_VA = 1;        % 未指定的反射材质（用于虚拟锚点VA）
fp_coeffs.door_var = 2;         % 门材质（如果未来要具体使用）
fp_coeffs.concrete_wall = 3;    % 混凝土墙
fp_coeffs.glass = 4;            % 玻璃材质（用于所有玻璃表面，后续可细分）
fp_coeffs.metal = 5;            % 金属材质
fp_coeffs.dummy = 0;            % 内部使用的占位符，不产生反射
fp_coeffs.absorb = 17;          % 吸收材质，阻挡信号但不产生虚拟锚点VA

% 墙段连接类型定义：
% 用于描述墙壁段之间的几何关系，影响多径反射路径建模。

seg_con.unspec = -1;    % 未指定连接类型
seg_con.corner = 1;     % 墙角（可能产生双重反射）
seg_con.edge = 2;       % 墙边缘（无反射，仅散射点）
seg_con.cont = 3;       % 墙面连续（0度连接），仅材质变化

