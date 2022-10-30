function [Kd,Ks,H_dot] = decoupler_of(A,B,C)
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
% ans =
%  
% 0.63770632251741116766026078601204
%  2.1465029345390733678026116764884
%  -7.505820676274245729945688353487
% -15.045830441247355084586951550874
% ans =
%  
% 1.9974109902372673171954183043233 - 0.67110307475296686797281010640108i
% 1.9974109902372673171954183043233 + 0.67110307475296686797281010640108i
%                                      -7.5806507760781643208976361689259
%                                      -12.906171204396370313493200439721
GKd = det(N) / ds * eye(2);
[num,den]=numden(GKd);
% tmp = gcd(GKd(1,1),GKd(2,2));
root = vpa(solve(num(1,1)==0));
sr1 = root(5);
sr2 = root(6);
sr3 = root(7);
sr4 = root(8);

tmp=1;
% tmp = (s-sr1)*(s-sr2)*(s-sr3)*(s-sr4);
% tmp = (s-sr1)*(s-sr2)*(s-sr3)*(s-sr4)*(s+1)^4;
Ks_dot=[1/tmp,0; 0,1/tmp];
K=Kd*Ks_dot;

H=GKd*Ks_dot/(eye(2)+GKd*Ks_dot);
h11=H(1,1);
h22=H(2,2);

[num11,den11]=numden(h11);
num11=double(coeffs(num11));
den11=double(coeffs(den11));
[num22,den22]=numden(h22);
num22=double(coeffs(num22));
den22=double(coeffs(den22));

[A11,B11,C11,D11] = tf2ss(num11,den11);
[A22,B22,C22,D22] = tf2ss(num22,den22);
% sys1 = tf2ss(num11,den11);
% sys2 = tf2ss(num22,den22);

% Q = eye(8);
% R = 1;
% 
% % K11=lqr_control(A11,B11,Q,R);
% % K22=lqr_control(A22,B22,Q,R);
% 
% K11=lqr(A11,B11,Q,R,0);
% K22=lqr(A11,B11,Q,R,0);

damp = 0.9;
wn=10;
% lamda1 = -(damp*wn + wn*sqrt(1-damp*damp)*1i);
% lamda2 = -(damp*wn - wn*sqrt(1-damp*damp)*1i);
lamda1 = -10+10*1i;
lamda2 = -10-10*1i;
lamda3 = -50;
lamda4 = -100;
lamda5 = -150;
lamda6 = -200;
lamda7 = -250;
lamda8 = -300;
p11 = [lamda1 lamda2 lamda3 lamda4 lamda5 lamda6 lamda7 lamda8];
p22 = [lamda1 lamda2 lamda3 lamda4 lamda5 lamda6 lamda7 lamda8];

K11=place(A11,B11,p11);
K22=place(A22,B22,p22);

% H_dot11=ss2tf(A11-B11*K11,B11,C11,D11);
% H_dot22=ss2tf(A22-B22*K22,B22,C22,D22); 

sys1=ss(A11-B11*K11,B11,C11,D11);
sys2=ss(A22-B22*K22,B22,C22,D22);
H_dot11=tf(sys1);
H_dot22=tf(sys2);

figure(1);
step(H_dot11);
% figure(2);
% step(H_dot22);

Ks11=H_dot11/(1-H_dot11)/GKd(1,1);
Ks22=H_dot22/(1-H_dot22)/GKd(2,2);

Ks=[Ks11,0;0,Ks22];
H_dot=[H_dot11,0;0,H_dot22];

end

