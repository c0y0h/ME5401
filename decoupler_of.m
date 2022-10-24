function [Kd,Ks,H] = decoupler_of(A,B,C)
%DECOUPLER_OF 此处显示有关此函数的摘要
%   此处显示详细说明
D=[0,0;0,0];
system=ss(A,B,C,D);
G=tf(system);
syms s;

N=[25*s^4 + 412.3*s^3 + 510.9*s^2 - 7497*s + 1.086e04,...
   11.2*s^4 + 184.7*s^3 - 222.7*s^2 - 5622*s + 1.535e04;
   40*s^4 + 790.8*s^3 + 2122*s^2 - 1.108e04*s + 6114,...
   60.2*s^4 + 1190*s^3 + 3101*s^2 - 1.707e04*s + 9306];
ds=s^6 + 31.49*s^5 + 282.2*s^4 + 300.9*s^3 - 31738*s^2 + 2563*s - 379.8;

Kd=inv(N)*det(N);
GKd = det(N) / ds * eye(2);
[num,den]=numden(GKd);
% tmp = gcd(GKd(1,1),GKd(2,2));
root = vpa(solve(num(1,1)==0));
sr1 = root(5);
sr2 = root(6);
sr3 = root(7);
sr4 = root(8);
tmp = (s-sr1)*(s-sr2)*(s-sr3)*(s-sr4);
Ks=[1/tmp,0; 0,1/tmp];
K=Kd*Ks;

H=inv(eye(2)+GKd*Ks)*GKd*Ks;

[num,den]=numden(K);

damp = 0.707;
wn = 1.13;
lamda1 = -damp*wn + wn*sqrt(1-damp*damp)*1i;
lamda2 = -damp*wn - wn*sqrt(1-damp*damp)*1i;
lamda3 = -1.6;
lamda4 = -2.4;
lamda5 = -3.2;
lamda6 = -4.0;
p1 = [lamda1 lamda2 lamda3 lamda4 lamda5 lamda6];
p2 = [lamda3 lamda4 lamda5 lamda6];

[A1dot,B1dot,C1dot,D1dot]=tf2ss(num(1,1),den(1,1));
K41 = place(A1dot,B1dot,p1);

[A2dot,B2dot,C2dot,D2dot]=tf2ss(num(2,2),den(2,2));
K42 = place(A1dot,B1dot,p1);

t = 0 : 0.1 : 10;
Aprime1 = A1dot-B1dot*K41;
ss1 = ss(Aprime, B1dot, C, D);
u1 = [ones(size(t,2),1),zeros(size(t,2),1)];
x = zeros(6,1);
[y, tout, x] = lsim(ss, u1, t, x0);
plot(t,x);

end

