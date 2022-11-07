function [K] = dlqr_control(A,B,Q,R)
%DLQR_CONTROL Summary of this function goes here
%   Detailed explanation goes here
%   discretize state space equation
m=size(A,1);
n=size(B,2);
dt=0.01;
Aba=(eye(m)-A*dt/2)\(eye(m)+A*dt/2);
Bba=B*dt;
P=Q;
P_new=Q+Aba'*P*Aba-Aba'*P*Bba/(R+Bba'*P*Bba)*Bba'*P*Aba;
it_count=1;
while max(abs(P_new-P),[],'all') > 1e-5
    P=P_new;
    P_new=Q+Aba'*P*Aba-Aba'*P*Bba/(R+Bba'*P*Bba)*Bba'*P*Aba;
    it_count=it_count+1;
end
K=inv(R+Bba'*P_new*Bba)*Bba'*P_new*Aba;
end

