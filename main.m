clc,clear;
%   load parameter
[A, B, C, x0] = load_parameter(3, 1, 1, 5);
%%
%   pole placement feedback
%       Assume that you can measure all the six state variables, design a state feedback controller using
%   the pole place method, simulate the designed system and show all the six state responses to non-
%   zero initial state with zero external inputs. Discuss effects of the positions of the poles on system
%   performance, and also monitor control signal size. In this step, both the disturbance and set point
%   can be assumed to be zero.

[K11,K1] = pole_placement(A, B);

% %%
% t = 0 : 0.1 : 30;
% Aprime = A-B*K1;
% D = zeros(3, 2);
% ss = ss(Aprime, B, C, D);
% u1 = [ones(size(t,2),1),zeros(size(t,2),1)];
% x = zeros(6,1);
% [y, tout, x] = lsim(ss, u1, t, x0);
% plot(t,x);

%%
%   LQR control
%       Assume that you can measure all the six state variables, design a state feedback controller using
%   the LQR method, simulate the designed system and show all the state responses to non-zero
%   initial state with zero external inputs. Discuss effects of weightings Q and R on system
%   performance, and also monitor control signal size. In this step, both the disturbance and set point
%   can be assumed to be zero.
K2 = lqr_control(A, B);
% %%
% t = 0 : 0.1 : 10;
% Aprime = A-B*K2;
% D = zeros(3, 2);
% ss = ss(Aprime, B, C, D);
% u1 = [ones(size(t,2),1),zeros(size(t,2),1)];
% x = zeros(6,1);
% [y, tout, x] = lsim(ss, u1, t, x0);
% plot(t,x);
%%
% Assume you can only measure the three outputs. 
% Design a state observer, simulate the resultant observer-based LQR control system, 
% monitor the state estimation error, investigate effects of observer poles 
% on state estimation error and closed-loop control performance. In this step, both
% the disturbance and set point can be assumed to be zero. (10 points)

%%
%  Design a decoupling controller with closed-loop stability and simulate the 
%  step set point response of the resultant control system to verify
%  decoupling performance with stability. In this step, the disturbance can be 
%  assumed to be zero. Is the decoupled system internally stable? 
%  Please provide both the step (transient) response with zero initial states 
%  and the initial response with respect to x0 of the decoupled system 
%  to support your conclusion.
C2=[1 0 0 0 0 0; 0 0 1 0 0 0];

%   1. state feedback
[K4,F4]=decoupler_sf(A,B,C2);
syms s;
H = C2/(s*eye(6)-A+B*K4)*B*F4;

[numerator,denominator]=numden(H);
n11=double(coeffs(numerator(1,1)));
n22=double(coeffs(numerator(2,2)));
d11=double(coeffs(denominator(1,1)));
d22=double(coeffs(denominator(2,2)));
G11=tf(n11,d11);
figure(1);
step(G11);
G22=tf(n22,d22);
figure(2);
step(G22);

%   check, not internally stable
D=zeros(2,2);
Bf=B*F4;
Af=A-B*K4;
decouple_model=ss(Af,Bf,C2,D);
p=pole(decouple_model)


% %   generalize method
% damp = 0.707;
% wn = 1.13;
% lamda1 = -damp*wn + wn*sqrt(1-damp*damp)*1i;
% lamda2 = -damp*wn - wn*sqrt(1-damp*damp)*1i;
% lamda3 = -1.6;
% lamda4 = -2.4;
% lamda5 = -3.2;
% lamda6 = -4.0;
% p = [lamda1 lamda2 lamda3 lamda4 lamda5 lamda6];
% 
% Gkl=decoupling_pole(A,B,C2,D,p);
% 
% [numerator,denominator]=numden(Gkl);
% n11=double(coeffs(numerator(1,1)));
% n12=double(coeffs(numerator(1,2)));
% n21=double(coeffs(numerator(2,1)));
% n22=double(coeffs(numerator(2,2)));
% d11=double(coeffs(denominator(1,1)));
% d12=double(coeffs(denominator(1,2)));
% d21=double(coeffs(denominator(2,1)));
% d22=double(coeffs(denominator(2,2)));
% G11=tf(n11,d11);
% figure(1);
% step(G11);
% G22=tf(n22,d22);
% figure(2);
% step(G22);
%%
%   2. output feedback
[Kd,Ks,H]=decoupler_of(A,B,C2);


[K41,K411] = pole_placement(A-B*Kd, B);

[numerator,denominator]=numden(H);
n11=double(coeffs(numerator(1,1)));
n12=double(coeffs(numerator(1,2)));
n21=double(coeffs(numerator(2,1)));
n22=double(coeffs(numerator(2,2)));
d11=double(coeffs(denominator(1,1)));
d12=double(coeffs(denominator(1,2)));
d21=double(coeffs(denominator(2,1)));
d22=double(coeffs(denominator(2,2)));

G11=tf(n11,d11);
step(G11);


%%



















