%% 验证机器人3分支构型各项指标求解函数
clc;
clear;

%% 模块库初始化
RP_data = Module_Lib();

%% 原始决策变量
n = 16;
module    =    [1 2 3 4 10    1 2 3 4 10     1 2  3  4  10 3    zeros(1,24)];
install   =    ones(1,40);
align     =    zeros(1,40);
sequence  =    [0 1 2 3 4 ...
    0 6 7 8 9 ...
    0 11 12 13 14 15 ...
    zeros(1,24)];

%% 真实决策变量
module_real   =   module(1:n);
install_real  =   install(1:n);
align_real    =   align(1:n);
sequence_real =   sequence(1:n);

%% 真实决策变量展开
[module_out, install_out, align_out, sequence_out, num_modules_physical, is_valid, err] = expand_module_units(module_real, install_real, align_real, sequence_real, RP_data);

%% 机器人初始化
LP = LP_generate(module_out, install_out, align_out, sequence_out, RP_data);
SV = SV_generate(LP);
q = 0*pi*ones(1,LP.num_q);
SV = Trans_aa_pos_ori(LP,SV,q);
PlotSV(LP,SV);

%% 目标点设置，此处为偷懒采用正运动学设置
Goal = Goal_init(SV);
Goal.change = [1 1 1];

%% 求优化目标1：机器人末端最大可操作度
[SV_all, flag_all, q_all, w_all] =  SQP_all(LP, SV, Goal);

%% 求优化目标2：机器人末端最大定位误差
sig_worst_all2 = calc_sig_worst_all(SV_all, LP);

%% 求优化目标3：机器人原始模块数num1
num1 = numel(module_real);

%% 求优化目标4：机器人展开后模块数num2
num2 = num_modules_physical;

