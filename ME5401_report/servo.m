function [K,L] = servo(A,B,C)
%SERVO 
Aba=[ A  zeros(6,3);
     -C  zeros(3,3)];
Bba=[B; zeros(3,2)];
Bwba=[B; zeros(3,2)];
Brba=[zeros(6,3); eye(3)];
Cba=[C zeros(3,3)];

Qc=[Bba Aba*Bba Aba^2*Bba Aba^3*Bba Aba^4*Bba Aba^5*Bba Aba^6*Bba Aba^7*Bba Aba^8*Bba];
rank(Qc)
rank([A B;C zeros(3,2)])

%dlqr
Q=eye(9)*20;
% Q=[2  0  0  0  0  0  0  0  0;
%    0  2  0  0  0  0  0  0  0;
%    0  0  3  0  0  0  0  0  0;
%    0  0  0  4  0  0  0  0  0;
%    0  0  0  0  5  0  0  0  0;
%    0  0  0  0  0  4  0  0  0;
%    0  0  0  0  0  0  3  0  0;
%    0  0  0  0  0  0  0  2  0;
%    0  0  0  0  0  0  0  0  2]*20;
R=[1 0;
   0 1]*5;
K=lqr(Aba,Bba,Q,R,0);
% K=lqr_control(Aba,Bba,Q,R);

M = [ Aba   -Bba/R*Bba';
     -Q   -Aba'];

[evec, eval] = eig(M);
eval = sum(eval);
evec_stable = evec(:,find(real(eval)<0));
% P = evec_stable(,:)
V = evec_stable(1:9,:);
U = evec_stable(10:18,:);
P = U/V;

K = R \ Bba' * P;


K1=K(:,1:6);
K2=K(:,7:9);

%observer
% lamda1 = -10+10*1i;
% lamda2 = -10-10*1i;
% lamda3 = -50;
% lamda4 = -75;
% lamda5 = -100;
% lamda6 = -125;
% p = [lamda1 lamda2 lamda3 lamda4 lamda5 lamda6]*2.7;

damp = 0.8;
wn=2;
lamda1 = -(damp*wn + wn*sqrt(1-damp*damp)*1i);
lamda2 = -(damp*wn - wn*sqrt(1-damp*damp)*1i);
lamda3 = -4*damp*wn;
lamda4 = -4.3*damp*wn;
lamda5 = -4.6*damp*wn;
lamda6 = -5.0*damp*wn;
p = [lamda1 lamda2 lamda3 lamda4 lamda5 lamda6]*2.7;
[L,At,Bt,Ct]=state_observer(K1,A,B,C,p);

end

