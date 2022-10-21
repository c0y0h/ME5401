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
% A=[0 1;-1 -2];
% B=[1 0;0 1];
% C2=[1 0.5; 1 1];
[K4,F4]=decoupler_sf(A,B,C2);
%%
D=zeros(2,2);
Bf=B*F4;
Af=A-B*K4;
decouple_model=ss(Af,Bf,C2,D);
p=pole(decouple_model);

% syms s;
% H=C2*inv(s*eye(2)-Af)*Bf;
%%

t = 0 : 0.1 : 30;
u1 = [ones(size(t,2),1),zeros(size(t,2),1)];
x = zeros(6,1);
[y, tout, x] = lsim(decouple_model, u1, t, x0);
plot(t,x);


















