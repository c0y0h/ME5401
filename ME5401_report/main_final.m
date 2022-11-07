clc,clear;
%   load parameter
[A, B, C, x0] = load_parameter(3, 1, 1, 5);

%   design specifications check

D = zeros(3, 2);
t = 0 : 0.1 : 10;

u1 = [ones(size(t,2),1),zeros(size(t,2),1)];
u2 = [zeros(size(t,2),1),ones(size(t,2),1)];
u0 = [zeros(size(t,2),1),zeros(size(t,2),1)];
u3 = [ones(size(t,2),1),ones(size(t,2),1)];

%% Q1
%   pole placement feedback
%       Assume that you can measure all the six state variables, design a state feedback controller using
%   the pole place method, simulate the designed system and show all the six state responses to non-
%   zero initial state with zero external inputs. Discuss effects of the positions of the poles on system
%   performance, and also monitor control signal size. In this step, both the disturbance and set point
%   can be assumed to be zero.

damp = 0.8;
wn = 3.125;
lamda1 = -(damp*wn + wn*sqrt(1-damp*damp)*1i);
lamda2 = -(damp*wn - wn*sqrt(1-damp*damp)*1i);
lamda3 = -4*damp*wn;
lamda4 = -4.3*damp*wn;
lamda5 = -4.6*damp*wn;
lamda6 = -5.0*damp*wn;
p = [lamda1 lamda2 lamda3 lamda4 lamda5 lamda6];

[K11,K1] = pole_placement(A,B,p);

% check for K11
ss_q1 = ss(A-B*K11, B, C, D);
global_check(ss_q1,u1,u2,t,zeros(6,1));

% check for K1
ss_q1 = ss(A-B*K1, B, C, D);
global_check(ss_q1,u1,u2,t,zeros(6,1));

% task 1 check
task12_check(ss_q1,t,u0,x0,K11);

%% Q2
%   LQR control
%       Assume that you can measure all the six state variables, design a state feedback controller using
%   the LQR method, simulate the designed system and show all the state responses to non-zero
%   initial state with zero external inputs. Discuss effects of weightings Q and R on system
%   performance, and also monitor control signal size. In this step, both the disturbance and set point
%   can be assumed to be zero.

Q = [ 0.2   0   0   0   0   0;
      0   1   0   0   0   0;
      0   0   1   0   0   0;
      0   0   0   0.1   0   0;
      0   0   0   0   1   0;
      0   0   0   0   0   1]*50;
R = [1   0;
     0   1]*1;

%   eigenvalue eigenvector method in class
K2 = lqr_control(A,B,Q,R);

%   discrete lqr
K22=dlqr_control(A,B,Q,R);

% check for K22
ss_q2 = ss(A-B*K22, B, C, D);
global_check(ss_q2,u1,u2,t,zeros(6,1));

% check for K2
ss_q2 = ss(A-B*K2, B, C, D);
global_check(ss_q2,u1,u2,t,zeros(6,1));

% task 2 check
task12_check(ss_q2,t,u0,x0,K2);

%% Q3
% Assume you can only measure the three outputs. 
% Design a state observer, simulate the resultant observer-based LQR control system, 
% monitor the state estimation error, investigate effects of observer poles 
% on state estimation error and closed-loop control performance. In this step, both
% the disturbance and set point can be assumed to be zero. (10 points)
Q = [ 0.2   0   0   0   0   0;
      0   1.1   0   0   0   0;
      0   0   1.1   0   0   0;
      0   0   0   0.2   0   0;
      0   0   0   0   1.1   0;
      0   0   0   0   0   1.1]*20;
R = [1   0;
     0   1]*5;

%   the lqr feedback
K3=lqr_control(A,B,Q,R);

damp = 0.8;
wn=2;
lamda1 = -(damp*wn + wn*sqrt(1-damp*damp)*1i);
lamda2 = -(damp*wn - wn*sqrt(1-damp*damp)*1i);
lamda3 = -4*damp*wn;
lamda4 = -4.3*damp*wn;
lamda5 = -4.6*damp*wn;
lamda6 = -5.0*damp*wn;
p1 = [lamda1 lamda2 lamda3 lamda4 lamda5 lamda6]*2.7;
p2 = [lamda1 lamda2 lamda3 lamda4 lamda5 lamda6]*2;
p3 = [lamda1 lamda2 lamda3 lamda4 lamda5 lamda6]*4;

[L,At,Bt,Ct]=state_observer(K3,A,B,C,p1);
ss_q3=ss(At,Bt,Ct,D);

[L,At,Bt,Ct]=state_observer(K3,A,B,C,p2);
ss_q32=ss(At,Bt,Ct,D);

[L,At,Bt,Ct]=state_observer(K3,A,B,C,p3);
ss_q33=ss(At,Bt,Ct,D);

xe=x0;

% global check
global_check(ss_q3,u1,u2,t,[zeros(6,1);xe]);

% task 3 check
task3_check(ss_q3,ss_q32,ss_q33,u0,t,[x0;xe]);

%% Q4
%  Design a decoupling controller with closed-loop stability and simulate the 
%  step set point response of the resultant control system to verify
%  decoupling performance with stability. In this step, the disturbance can be 
%  assumed to be zero. Is the decoupled system internally stable? 
%  Please provide both the step (transient) response with zero initial states 
%  and the initial response with respect to x0 of the decoupled system 
%  to support your conclusion.
C2=[1 0 0 0 0 0; 0 0 1 0 0 0];
D2=[0 0;0 0];

%   state feedback
[K4,F4]=decoupler_sf(A,B,C2);

ss_q4 = ss(A-B*K4, B*F4, C2, D2);

% global check
global_check(ss_q4,u1,u2,t,zeros(6,1));

% task 4 check
task4_check(ss_q4,t,u0,u1,u2,u3,x0);

%% Q5
% Assume that you only have three cheap sensors to measure the output. Design a controller such 
% that the plant (vehicle) can operate around the set point as close as possible at steady state even 
% when step disturbances are present at the plant input. Plot out both the control and output signals.
% In your simulation, you may assume the step disturbance for the two inputs,
% takes effect from time td = 10s afterwards.

ysp=-0.1*C/A*B*[-0.5+(3-1)/20; 0.1+(1-1)/(3+5+10)];
% ysp=[1;3;5];
[K5,L5]=servo(A,B,C);
K51=K5(:,1:6);
K52=K5(:,7:9);

%%
% run simulink
% plot
figure;
curve=plot(yout.time(300:end,1),yout.signals(1).values(300:end,1),yout.time(300:end,1),yout.signals(1).values(300:end,2),yout.time(300:end,1),yout.signals(1).values(300:end,3));
grid on; 
legend('Cart position','Handle angle','Bike angle');
xlabel('Time (sec)'), ylabel('Output Amplitude'); 
title('Output Signal');

figure;
curve=plot(yout.time(300:end,1),yout.signals(1).values(300:end,1)-ysp(1),yout.time(300:end,1),yout.signals(1).values(300:end,2)-ysp(2),yout.time(300:end,1),yout.signals(1).values(300:end,3)-ysp(3));
grid on; 
legend('Cart position error','Handle angle error','Bike angle error');
xlabel('Time (sec)'), ylabel('Error'); 
title('Tracking Error');

figure;
curve=plot(controlSignal.time(300:end,1),controlSignal.signals(1).values(300:end,1),controlSignal.time(300:end,1),controlSignal.signals(1).values(300:end,2));
grid on; 
legend('Control channel 1','Control channel 1');
xlabel('Time (sec)'), ylabel('Amplitude'); 
title('Control cost');

