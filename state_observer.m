function [L,At,Bt,Ct] = state_observer(K,A,B,C,p)
%STATE_OBSERVER Summary of this function goes here
%   Detailed explanation goes here
% ss_lqr = ss(A-B*K, B, C, D);
% p=pole(ss_lqr)*2.5;

% damp = 0.8;
% wn=2;
% lamda1 = -(damp*wn + wn*sqrt(1-damp*damp)*1i);
% lamda2 = -(damp*wn - wn*sqrt(1-damp*damp)*1i);
% lamda3 = -4*damp*wn;
% lamda4 = -4.3*damp*wn;
% lamda5 = -4.6*damp*wn;
% lamda6 = -5.0*damp*wn;
% p = [lamda1 lamda2 lamda3 lamda4 lamda5 lamda6]*2.7;


L=(place(A',C',p))';
At=[A-B*K           B*K;
    zeros(size(A))  A-L*C];
Bt=[B;zeros(size(B))];
Ct=[C   zeros(size(C))];
end

