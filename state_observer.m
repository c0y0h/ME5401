function [L,At,Bt,Ct] = state_observer(K,A,B,C,D)
%STATE_OBSERVER Summary of this function goes here
%   Detailed explanation goes here
ss_lqr = ss(A-B*K, B, C, D);
p=pole(ss_lqr)*5;
L=(place(A',C',p))';
At=[A-B*K           B*K;
    zeros(size(A))  A-L*C];
Bt=[B;zeros(size(B))];
Ct=[C   zeros(size(C))];
end

