function [K] = lqr_control(A,B,Q,R)
%LQR_CONTROL 此处显示有关此函数的摘要
%   此处显示详细说明
% Q = [ 0.2   0   0   0   0   0;
%       0   1   0   0   0   0;
%       0   0   1   0   0   0;
%       0   0   0   0.1   0   0;
%       0   0   0   0   1   0;
%       0   0   0   0   0   1]*50;
% R = [1   0;
%      0   1]*1;
M = [ A   -B/R*B';
     -Q   -A'];

[evec, eval] = eig(M);
eval = sum(eval);
evec_stable = evec(:,find(real(eval)<0));
% P = evec_stable(,:)
V = evec_stable(1:6,:);
U = evec_stable(7:12,:);
P = U/V;

K = R \ B' * P;

end

