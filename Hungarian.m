function [Matching,Cost] = Hungarian(Perf)
% Hungarian - 使用匈牙利算法求解最小权重完美匹配问题
%
% 输入:
%   Perf - M×N的权重矩阵，表示边权重。Inf表示该边不存在。
%
% 输出:
%   Matching - M×N的匹配矩阵，1表示匹配，0表示不匹配。
%   Cost     - 最小匹配总代价（权重和）
%
% 说明:
%   使用标准匈牙利算法（Hungarian Algorithm）求解二分图最小权匹配问题。
%
% 作者: Alex Melin, 2006年6月30日

% 初始化匹配矩阵为零
Matching = zeros(size(Perf));

% 去除无连接顶点，减少问题规模以加速计算
num_y = sum(~isinf(Perf),1); % 每列非无穷元素数（连接数）
num_x = sum(~isinf(Perf),2); % 每行非无穷元素数（连接数）

x_con = find(num_x ~= 0); % 有连接的行索引
y_con = find(num_y ~= 0); % 有连接的列索引

% 构造缩减的性能矩阵（只保留有连接的部分）
P_size = max(length(x_con), length(y_con)); % 新矩阵大小为行列数最大值
P_cond = zeros(P_size);
P_cond(1:length(x_con), 1:length(y_con)) = Perf(x_con, y_con);

% 如果矩阵为空，直接返回
if isempty(P_cond)
    Cost = 0;
    return
end

% 确保存在完美匹配，若无则扩展矩阵并添加虚拟顶点和边
Edge = P_cond;
Edge(P_cond ~= Inf) = 0; % 构造边矩阵，0表示存在边
cnum = min_line_cover(Edge); % 计算最小覆盖数，缺失匹配数量

Pmax = max(max(P_cond(P_cond ~= Inf))); % 非Inf元素最大权重
P_size = length(P_cond) + cnum; % 扩展矩阵大小
P_cond = ones(P_size) * Pmax; % 用最大值填充新矩阵
P_cond(1:length(x_con), 1:length(y_con)) = Perf(x_con, y_con);

% 主循环，根据当前步骤控制算法流程
exit_flag = 1;
stepnum = 1;
while exit_flag
    switch stepnum
        case 1
            [P_cond, stepnum] = step1(P_cond); % 第1步，行减法
        case 2
            [r_cov, c_cov, M, stepnum] = step2(P_cond); % 第2步，初始匹配
        case 3
            [c_cov, stepnum] = step3(M, P_size); % 第3步，列覆盖检查
        case 4
            [M, r_cov, c_cov, Z_r, Z_c, stepnum] = step4(P_cond, r_cov, c_cov, M); % 第4步，质子标记
        case 5
            [M, r_cov, c_cov, stepnum] = step5(M, Z_r, Z_c, r_cov, c_cov); % 第5步，更新匹配路径
        case 6
            [P_cond, stepnum] = step6(P_cond, r_cov, c_cov); % 第6步，调整矩阵
        case 7
            exit_flag = 0; % 结束循环
    end
end

% 将缩减矩阵的匹配映射回原矩阵大小
Matching(x_con, y_con) = M(1:length(x_con), 1:length(y_con));

% 计算匹配总成本
Cost = sum(sum(Perf(Matching == 1)));

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Step 1: 每行减去该行最小元素，保证每行至少有一个0
function [P_cond, stepnum] = step1(P_cond)
    P_size = length(P_cond);
    for ii = 1:P_size
        rmin = min(P_cond(ii,:)); % 当前行最小值
        P_cond(ii,:) = P_cond(ii,:) - rmin; % 行减法
    end
    stepnum = 2;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Step 2: 标记零元素，选取不同行列冲突的0作为初始匹配(星标零)
function [r_cov, c_cov, M, stepnum] = step2(P_cond)
    P_size = length(P_cond);
    r_cov = zeros(P_size,1); % 行覆盖向量
    c_cov = zeros(P_size,1); % 列覆盖向量
    M = zeros(P_size);       % 匹配标记矩阵：1星标0，2质子标记
    
    for ii = 1:P_size
        for jj = 1:P_size
            if P_cond(ii,jj) == 0 && r_cov(ii) == 0 && c_cov(jj) == 0
                M(ii,jj) = 1; % 标记星标零
                r_cov(ii) = 1; % 行覆盖
                c_cov(jj) = 1; % 列覆盖
            end
        end
    end
    
    % 重置覆盖向量，为下一步准备
    r_cov = zeros(P_size,1);
    c_cov = zeros(P_size,1);
    stepnum = 3;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Step 3: 覆盖所有含星标零的列，若覆盖列数等于矩阵维度，完成匹配
function [c_cov, stepnum] = step3(M, P_size)
    c_cov = sum(M,1); % 各列是否含星标零
    if sum(c_cov) == P_size
        stepnum = 7; % 匹配完成
    else
        stepnum = 4; % 否则进入下一步
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Step 4: 查找未覆盖的零，标记质子零，寻找增广路径
function [M, r_cov, c_cov, Z_r, Z_c, stepnum] = step4(P_cond, r_cov, c_cov, M)
    P_size = length(P_cond);
    zflag = 1;
    while zflag
        row = 0; col = 0; exit_flag = 1;
        ii = 1; jj = 1;
        % 寻找第一个未被覆盖的0
        while exit_flag
            if P_cond(ii,jj) == 0 && r_cov(ii) == 0 && c_cov(jj) == 0
                row = ii;
                col = jj;
                exit_flag = 0;
            end
            jj = jj + 1;
            if jj > P_size
                jj = 1; ii = ii + 1;
            end
            if ii > P_size
                exit_flag = 0;
            end
        end
        
        if row == 0
            % 无未覆盖0，转第6步
            stepnum = 6;
            zflag = 0;
            Z_r = 0; Z_c = 0;
        else
            % 质子标记该0
            M(row,col) = 2;
            % 若该行有星标零，则覆盖该行并展开覆盖列
            if sum(find(M(row,:) == 1)) ~= 0
                r_cov(row) = 1;
                zcol = find(M(row,:) == 1);
                c_cov(zcol) = 0;
            else
                % 无星标零，准备构建增广路径，转第5步
                stepnum = 5;
                zflag = 0;
                Z_r = row;
                Z_c = col;
            end
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Step 5: 构建增广路径，更新星标和质子标记，调整匹配
function [M, r_cov, c_cov, stepnum] = step5(M, Z_r, Z_c, r_cov, c_cov)
    zflag = 1;
    ii = 1;
    while zflag
        rindex = find(M(:, Z_c(ii)) == 1); % 星标零所在行
        if rindex > 0
            ii = ii + 1;
            Z_r(ii,1) = rindex;
            Z_c(ii,1) = Z_c(ii-1);
        else
            zflag = 0;
        end
        
        if zflag == 1
            cindex = find(M(Z_r(ii), :) == 2); % 质子零所在列
            ii = ii + 1;
            Z_r(ii,1) = Z_r(ii-1);
            Z_c(ii,1) = cindex;
        end
    end
    
    % 交替翻转路径上的星标和质子标记
    for ii = 1:length(Z_r)
        if M(Z_r(ii), Z_c(ii)) == 1
            M(Z_r(ii), Z_c(ii)) = 0;
        else
            M(Z_r(ii), Z_c(ii)) = 1;
        end
    end
    
    % 清空覆盖和质子标记
    r_cov = zeros(size(r_cov));
    c_cov = zeros(size(c_cov));
    M(M == 2) = 0; % 去除所有质子标记
    
    stepnum = 3;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Step 6: 调整矩阵值，通过加减最小未覆盖值生成更多零元素
function [P_cond, stepnum] = step6(P_cond, r_cov, c_cov)
    a = find(r_cov == 0); % 未覆盖行索引
    b = find(c_cov == 0); % 未覆盖列索引
    minval = min(min(P_cond(a,b))); % 最小未覆盖元素
    
    % 覆盖行加上最小值
    P_cond(find(r_cov == 1), :) = P_cond(find(r_cov == 1), :) + minval;
    % 未覆盖列减去最小值
    P_cond(:, find(c_cov == 0)) = P_cond(:, find(c_cov == 0)) - minval;
    
    stepnum = 4;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 计算最小线覆盖数，辅助判断完美匹配存在性
function cnum = min_line_cover(Edge)
    [r_cov, c_cov, M, stepnum] = step2(Edge);
    [c_cov, stepnum] = step3(M, length(Edge));
    [M, r_cov, c_cov, Z_r, Z_c, stepnum] = step4(Edge, r_cov, c_cov, M);
    % 计算缺失的匹配数 = 矩阵大小 - 覆盖行数 - 覆盖列数
    cnum = length(Edge) - sum(r_cov) - sum(c_cov);
end
