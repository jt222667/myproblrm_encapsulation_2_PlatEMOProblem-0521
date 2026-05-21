%% 优化目标1：计算机器人所有末端最大可操作度之和w_all

function [SV_all, flag_all, q_all, w_all] =  SQP_all(LP, SV, Goal)

flag_all = 0;
q_all = zeros(LP.num_q,1);
w_all  = 0;

for k = 1:SV.m
    [SV, flag, q_k, ~, w_k] =  SQP_main(LP, SV, Goal, k);
    q_all(SV.Path{k}) = q_k;
    w_all = w_all + w_k;
    if flag
        flag_all = 1;
        SV_all = SV;
        return;
    end

end

SV_all = Trans_aa_pos_ori(LP,SV,q_all);

end
