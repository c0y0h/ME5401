function [L,At,Bt,Ct] = state_observer(K,A,B,C,p)
%STATE_OBSERVER Summary of this function goes here
%   Detailed explanation goes here

[L1,L]=pole_placement(A',C',p);
L=L';

At=[A-B*K           B*K;
    zeros(size(A))  A-L*C];
Bt=[B;zeros(size(B))];
Ct=[C   zeros(size(C))];
end

