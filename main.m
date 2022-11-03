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

[K11,K1] = pole_placement(A, B);


% check for K11
% C2=[1 0 0 0 0 0; 0 1 0 0 0 0];
% D2=[0 0; 0 0];
% F=inv(C2/(B*K11-A)*B);
% ss_q2 = ss(A-B*K11, B*F, C2, D2);
ss_q1 = ss(A-B*K11, B, C, D);

%   u=[1 0]
[y,t,x]=lsim(ss_q1,u1,t,zeros(6,1));
% [y,t,x]=lsim(ss_q2,u0,t,x0);
q1_info = stepinfo(y,t);

figure(3);
subplot(3,1,1);
plot(t,y(:,1));
title({'Output curve with u=[1 0]', ...
    ['overshoot: ', num2str(q1_info(1).Overshoot),'%'], ...
    ['2% setting time: ', num2str(q1_info(1).SettlingTime),'s' ]});
xlabel('Time (sec)');
ylabel('Cart Position (m)');

subplot(3,1,2);
plot(t,y(:,2));
title({'Output curve with u=[1 0]', ...
    ['overshoot: ', num2str(q1_info(2).Overshoot),'%'], ...
    ['2% setting time: ', num2str(q1_info(2).SettlingTime),'s' ]});
xlabel('Time (sec)');
ylabel('Handle Angle (rad)');

subplot(3,1,3);
plot(t,y(:,3));
title({'Output curve with u=[1 0]', ...
    ['overshoot: ', num2str(q1_info(3).Overshoot),'%'], ...
    ['2% setting time: ', num2str(q1_info(3).SettlingTime),'s' ]});
xlabel('Time (sec)');
ylabel('Bike Angle (rad)');

%   u=[0 1]
[y,t,x]=lsim(ss_q1,u2,t,zeros(6,1));
% [y,t,x]=lsim(ss_q2,u0,t,x0);
q1_info = stepinfo(y,t);

figure(4);
subplot(3,1,1);
plot(t,y(:,1));
title({'Output curve with u=[0 1]', ...
    ['overshoot: ', num2str(q1_info(1).Overshoot),'%'], ...
    ['2% setting time: ', num2str(q1_info(1).SettlingTime),'s' ]});
xlabel('Time (sec)');
ylabel('Cart Position (m)');

subplot(3,1,2);
plot(t,y(:,2));
title({'Output curve with u=[0 1]', ...
    ['overshoot: ', num2str(q1_info(2).Overshoot),'%'], ...
    ['2% setting time: ', num2str(q1_info(2).SettlingTime),'s' ]});
xlabel('Time (sec)');
ylabel('Handle Angle (rad)');

subplot(3,1,3);
plot(t,y(:,3));
title({'Output curve with u=[0 1]', ...
    ['overshoot: ', num2str(q1_info(3).Overshoot),'%'], ...
    ['2% setting time: ', num2str(q1_info(3).SettlingTime),'s' ]});
xlabel('Time (sec)');
ylabel('Bike Angle (rad)');

%%
% check for K1
%   u=[1 0]
ss_q1 = ss(A-B*K1, B, C, D);
[y,t,x]=lsim(ss_q1,u1,t,zeros(6,1));
q1_info = stepinfo(y,t);

figure(1);
subplot(3,1,1);
plot(t,y(:,1));
title({'Output curve with u=[1 0]', ...
    ['overshoot: ', num2str(q1_info(1).Overshoot),'%'], ...
    ['2% setting time: ', num2str(q1_info(1).SettlingTime),'s' ]});
xlabel('Time (sec)');
ylabel('Cart Position (m)');

subplot(3,1,2);
plot(t,y(:,2));
title({'Output curve with u=[1 0]', ...
    ['overshoot: ', num2str(q1_info(2).Overshoot),'%'], ...
    ['2% setting time: ', num2str(q1_info(2).SettlingTime),'s' ]});
xlabel('Time (sec)');
ylabel('Handle Angle (rad)');

subplot(3,1,3);
plot(t,y(:,3));
title({'Output curve with u=[1 0]', ...
    ['overshoot: ', num2str(q1_info(3).Overshoot),'%'], ...
    ['2% setting time: ', num2str(q1_info(3).SettlingTime),'s' ]});
xlabel('Time (sec)');
ylabel('Bike Angle (rad)');

%   u=[0 1]
[y,t,x]=lsim(ss_q1,u2,t,zeros(6,1));
q1_info = stepinfo(y,t);

figure(2);
subplot(3,1,1);
plot(t,y(:,1));
title({'Output curve with u=[0 1]', ...
    ['overshoot: ', num2str(q1_info(1).Overshoot),'%'], ...
    ['2% setting time: ', num2str(q1_info(1).SettlingTime),'s' ]});
xlabel('Time (sec)');
ylabel('Cart Position (m)');

subplot(3,1,2);
plot(t,y(:,2));
title({'Output curve with u=[0 1]', ...
    ['overshoot: ', num2str(q1_info(2).Overshoot),'%'], ...
    ['2% setting time: ', num2str(q1_info(2).SettlingTime),'s' ]});
xlabel('Time (sec)');
ylabel('Handle Angle (rad)');

subplot(3,1,3);
plot(t,y(:,3));
title({'Output curve with u=[0 1]', ...
    ['overshoot: ', num2str(q1_info(3).Overshoot),'%'], ...
    ['2% setting time: ', num2str(q1_info(3).SettlingTime),'s' ]});
xlabel('Time (sec)');
ylabel('Bike Angle (rad)');

%%
%   task 1 check
% 6 state responses to x0 with zero inputs
[y,t,x]=lsim(ss_q1,u0,t,x0);
q1_info = stepinfo(x,t);

figure(5);
for i = 1:6
    subplot(3,2,i);
    plot(t,x(:,i));
    title(['Response for x',num2str(i)]);
    xlabel('Time (sec)');
    ylabel(['x',num2str(i)]);
%     hold on;
%     legend_str{i} = ['overshoot: ', num2str(q1_info(i).Overshoot),'%, ', ...
%                     '2% setting time: ', num2str(q1_info(i).SettlingTime),'s'];
end
% legend(legend_str);
sgtitle('Six state responses with zero inputs');

% monitor the control signal size
u=-x*K11';
figure(6);
for i = 1:2
    subplot(2,1,i);
    plot(t,u(:,i));
    title(['Control signal u',num2str(i)]);
    xlabel('Time (sec)');
    ylabel(['u',num2str(i)]);
end
% legend(legend_str);
sgtitle('Control signal size');

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
K2 = lqr_control(A,B,Q,R);

%   discrete lqr
K22=dlqr_control(A,B,Q,R);
%%

%   check
ss_q2 = ss(A-B*K2, B, C, D);
% ss_q2 = ss(A-B*K22, B, C, D);

%   u=[1 0]
[y, t, x] = lsim(ss_q2, u1, t, zeros(6,1));
q2_info = stepinfo(y,t);

figure(7);
subplot(3,1,1);
plot(t,y(:,1));
title({'Output curve with u=[1 0]', ...
    ['overshoot: ', num2str(q2_info(1).Overshoot),'%'], ...
    ['2% setting time: ', num2str(q2_info(1).SettlingTime),'s' ]});
xlabel('Time (sec)');
ylabel('Cart Position (m)');

subplot(3,1,2);
plot(t,y(:,2));
title({'Output curve with u=[1 0]', ...
    ['overshoot: ', num2str(q2_info(2).Overshoot),'%'], ...
    ['2% setting time: ', num2str(q2_info(2).SettlingTime),'s' ]});
xlabel('Time (sec)');
ylabel('Handle Angle (rad)');

subplot(3,1,3);
plot(t,y(:,3));
title({'Output curve with u=[1 0]', ...
    ['overshoot: ', num2str(q2_info(3).Overshoot),'%'], ...
    ['2% setting time: ', num2str(q2_info(3).SettlingTime),'s' ]});
xlabel('Time (sec)');
ylabel('Bike Angle (rad)');

%   u=[0 1]
[y, t, x] = lsim(ss_q2, u2, t, zeros(6,1));
q2_info = stepinfo(y,t);

figure(8);
subplot(3,1,1);
plot(t,y(:,1));
title({'Output curve with u=[0 1]', ...
    ['overshoot: ', num2str(q2_info(1).Overshoot),'%'], ...
    ['2% setting time: ', num2str(q2_info(1).SettlingTime),'s' ]});
xlabel('Time (sec)');
ylabel('Cart Position (m)');

subplot(3,1,2);
plot(t,y(:,2));
title({'Output curve with u=[0 1]', ...
    ['overshoot: ', num2str(q2_info(2).Overshoot),'%'], ...
    ['2% setting time: ', num2str(q2_info(2).SettlingTime),'s' ]});
xlabel('Time (sec)');
ylabel('Handle Angle (rad)');

subplot(3,1,3);
plot(t,y(:,3));
title({'Output curve with u=[0 1]', ...
    ['overshoot: ', num2str(q2_info(3).Overshoot),'%'], ...
    ['2% setting time: ', num2str(q2_info(3).SettlingTime),'s' ]});
xlabel('Time (sec)');
ylabel('Bike Angle (rad)');

%   task 2 check
% 6 state responses to x0 with zero inputs
[y,t,x]=lsim(ss_q2,u0,t,x0);
q2_info = stepinfo(x,t);

figure(9);
for i = 1:6
    subplot(3,2,i);
    plot(t,x(:,i));
    title(['Response for x',num2str(i)]);
    xlabel('Time (sec)');
    ylabel(['x',num2str(i)]);
end
sgtitle('Six state responses with zero inputs');

% monitor control signal size
u=-x*K2';
figure(10);
for i = 1:2
    subplot(2,1,i);
    plot(t,u(:,i));
    title(['Control signal u',num2str(i)]);
    xlabel('Time (sec)');
    ylabel(['u',num2str(i)]);
end
% legend(legend_str);
sgtitle('Control signal size');

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

%   check basic requirements
ss_q3=ss(At,Bt,Ct,D);

[L,At,Bt,Ct]=state_observer(K3,A,B,C,p2);
ss_q32=ss(At,Bt,Ct,D);

[L,At,Bt,Ct]=state_observer(K3,A,B,C,p3);
ss_q33=ss(At,Bt,Ct,D);

xe=x0;

%   u=[1 0]
[y, t, x] = lsim(ss_q3,u1,t,[zeros(6,1);xe]);
q3_info = stepinfo(y,t);

figure(11);
subplot(3,1,1);
plot(t,y(:,1));
title({'Output curve with u=[1 0]', ...
    ['overshoot: ', num2str(q3_info(1).Overshoot),'%'], ...
    ['2% setting time: ', num2str(q3_info(1).SettlingTime),'s' ]});
xlabel('Time (sec)');
ylabel('Cart Position (m)');

subplot(3,1,2);
plot(t,y(:,2));
title({'Output curve with u=[1 0]', ...
    ['overshoot: ', num2str(q3_info(2).Overshoot),'%'], ...
    ['2% setting time: ', num2str(q3_info(2).SettlingTime),'s' ]});
xlabel('Time (sec)');
ylabel('Handle Angle (rad)');

subplot(3,1,3);
plot(t,y(:,3));
title({'Output curve with u=[1 0]', ...
    ['overshoot: ', num2str(q3_info(3).Overshoot),'%'], ...
    ['2% setting time: ', num2str(q3_info(3).SettlingTime),'s' ]});
xlabel('Time (sec)');
ylabel('Bike Angle (rad)');

%   u=[0 1]
[y, t, x] = lsim(ss_q3,u2,t,[zeros(6,1);xe]);
q3_info = stepinfo(y,t);

figure(12);
subplot(3,1,1);
plot(t,y(:,1));
title({'Output curve with u=[0 1]', ...
    ['overshoot: ', num2str(q3_info(1).Overshoot),'%'], ...
    ['2% setting time: ', num2str(q3_info(1).SettlingTime),'s' ]});
xlabel('Time (sec)');
ylabel('Cart Position (m)');

subplot(3,1,2);
plot(t,y(:,2));
title({'Output curve with u=[0 1]', ...
    ['overshoot: ', num2str(q3_info(2).Overshoot),'%'], ...
    ['2% setting time: ', num2str(q3_info(2).SettlingTime),'s' ]});
xlabel('Time (sec)');
ylabel('Handle Angle (rad)');

subplot(3,1,3);
plot(t,y(:,3));
title({'Output curve with u=[0 1]', ...
    ['overshoot: ', num2str(q3_info(3).Overshoot),'%'], ...
    ['2% setting time: ', num2str(q3_info(3).SettlingTime),'s' ]});
xlabel('Time (sec)');
ylabel('Bike Angle (rad)');

%%
%   task 3 check
[y,t,x]=lsim(ss_q3,u0,t,[x0;xe]);
[ydot1,t,xdot1]=lsim(ss_q32,u0,t,[x0;xe]);
[ydot2,t,xdot2]=lsim(ss_q33,u0,t,[x0;xe]);
figure(9);
for i = 1:3
    subplot(3,1,i);
    plot(t,y(:,i),'-r',t,ydot1(:,i),'-g',t,ydot2(:,i),'b');
    legend('3 times faster','2 times faster','4 times faster');
    title(['Response for y',num2str(i)]);
    xlabel('Time (sec)');
    ylabel(['y',num2str(i)]);
end
sgtitle('System performance with different observer poles');
%%

% monitor the state estimation error
% t1=0:1E-6:0.2;
t1=0:0.05:1.2;
u00 = [zeros(size(t1,2),1),zeros(size(t1,2),1)];
[y,t1,x]=lsim(ss_q33,u00,t1,[x0;xe]);
q3_info = stepinfo(x,t1);

e=x(:,7:end);
x=x(:,1:6);
x_est=x-e;

% figure(13);
% x1=x(:,1);x2=x(:,2);x3=x(:,3);x4=x(:,4);x5=x(:,5);x6=x(:,6);
% x1_est=x_est(:,1);x2_est=x_est(:,2);x3_est=x_est(:,3);x4_est=x_est(:,4);x5_est=x_est(:,5);x6_est=x_est(:,6);
% plot(t1,x1,'-r',t1,x1_est,':r',...
%      t1,x2,'-b',t1,x2_est,':b',...
%      t1,x3,'-g',t1,x3_est,':g',...
%      t1,x4,'-y',t1,x4_est,':y',...
%      t1,x5,'-c',t1,x5_est,':c',...
%      t1,x6,'-k',t1,x6_est,':k');
% legend('x1','x1_{est}','x2','x2_{est}','x3','x3_{est}','x4','x4_{est}','x5','x5_{est}','x6','x6_{est}');
% xlabel('Time (sec)');

figure(13);
for i = 1:6
    subplot(3,2,i);
    plot(t1,x(:,i));
    hold on;
    plot(t1,x_est(:,i));
    legend(['x',num2str(i)],['x_{est}',num2str(i)]);
    title(['Comparison between x',num2str(i),' and x_{est}',num2str(i)]);
    xlabel('Time (sec)');
    ylabel(['x',num2str(i)]);
end
% legend(legend_str);
sgtitle('Comparison between x and x_{est}');

figure(14);
e1=e(:,1);e2=e(:,2);e3=e(:,3);e4=e(:,4);e5=e(:,5);e6=e(:,6);
plot(t1,e1,'-r', t1,e2,'-b', t1,e3,'-g', t1,e4,'-y', t1,e5,'-c', t1,e6,'-k');
legend('e1','e2','e3','e4','e5','e6');
xlabel('Time (sec)');
ylabel('State estimation error');
title('State estimation error with time');


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

%   1. state feedback
[K4,F4]=decoupler_sf(A,B,C2);

%   check
ss_q4 = ss(A-B*K4, B*F4, C2, D2);

%   u=[1 0]
[y, t, x] = lsim(ss_q4, u1, t, zeros(6,1));
q4_info = stepinfo(y,t);

figure(10);
for i = 1:2
    plot(t,y(:,i));
    if i==1
        legend_str{i} = ['y1: overshoot: ', num2str(q4_info(i).Overshoot),'%, ', ...
                    '2% setting time: ', num2str(q4_info(i).SettlingTime),'s'];
    else
        legend_str{i} = 'y2';
    end
    hold on;
end
legend(legend_str);
xlabel('Time (sec)');
ylabel('y');
sgtitle('Output response with input u=[1 0]');

%   u=[1 0]
[y, t, x] = lsim(ss_q4, u2, t, zeros(6,1));
q4_info = stepinfo(y,t);

figure(15);
for i = 1:2
    plot(t,y(:,i));
    if i==2
        legend_str{i} = ['y2: overshoot: ', num2str(q4_info(i).Overshoot),'%, ', ...
                    '2% setting time: ', num2str(q4_info(i).SettlingTime),'s'];
    else
        legend_str{i} = 'y1';
    end
    hold on;
end
legend(legend_str);
xlabel('Time (sec)');
ylabel('y');
sgtitle('Output response with input u=[0 1]');

%%   task 4 check
%   check if external stable (zeros initial states / x0)
% x0
% u1=[1 0]#
[y, t, x] = lsim(ss_q4, u1, t, x0);

figure(16);
for i = 1:2
    plot(t,y(:,i));
    legend_str{i} = ['y',num2str(i)];
    hold on;
end
legend(legend_str);
xlabel('Time (sec)');
ylabel('y');
sgtitle('Output response with input u=[1 0]');

% u2=[0 1]
[y, t, x] = lsim(ss_q4, u2, t, x0);
q4_info = stepinfo(y,t);

figure(17);
for i = 1:2
    plot(t,y(:,i));
    legend_str{i} = ['y',num2str(i)];
    hold on;
end
legend(legend_str);
xlabel('Time (sec)');
ylabel('y');
sgtitle('Output response with input u=[0 1]');

% u3=[1 1]
[y, t, x] = lsim(ss_q4, u3, t, x0);
q4_info = stepinfo(y,t);

figure(18);
for i = 1:2
    plot(t,y(:,i));
    legend_str{i} = ['y',num2str(i)];
    hold on;
end
legend(legend_str);
xlabel('Time (sec)');
ylabel('y');
sgtitle('Output response with input u=[1 1]');


% u3=[0 0]
[y, t, x] = lsim(ss_q4, u0, t, x0);
q4_info = stepinfo(y,t);
figure(19);
for i = 1:2
    plot(t,y(:,i));
    legend_str{i} = ['y',num2str(i)];
    hold on;
end
legend(legend_str);
xlabel('Time (sec)');
ylabel('y');
sgtitle('Output response with input u=[0 0]');

%   check if internally stable
p=pole(ss_q4);
for i=1:size(p)
    if real(p(i))>0
        disp(['Find pole with negative part:',num2str(p(i))]);
    end
end

% also plot x to check internal instability
[y,t,x]=lsim(ss_q4, u1, t, x0);
figure(20);
for i=1:6
    subplot(3,2,i);
    plot(t,x(:,i));
    title(['State x',num2str(i)]);
    xlabel('Time (sec)');
    ylabel(['x',num2str(i)]);
end
sgtitle('Response of x with input u=[1 0] and initial state x0')
%%
% 这里状�?�应该是看不到的，做完第三题再回头看
% ss_q4 = ss(A-B*K4, B*F4, C2, D2);
% �? state observer 去看

damp = 0.8;
wn=2;
lamda1 = -(damp*wn + wn*sqrt(1-damp*damp)*1i);
lamda2 = -(damp*wn - wn*sqrt(1-damp*damp)*1i);
lamda3 = -4*damp*wn;
lamda4 = -4.3*damp*wn;
lamda5 = -4.6*damp*wn;
lamda6 = -5.0*damp*wn;
p1 = [lamda1 lamda2 lamda3 lamda4 lamda5 lamda6]*2.7;
[L4,At4,Bt4,Ct4]=state_observer(K4,A,B,C2,p1);
ss_q4=ss(At4,Bt4*F4,Ct4,D2);

xe=x0;

t1=0:0.05:1.2;
u00 = [zeros(size(t1,2),1),zeros(size(t1,2),1)];
[y,t1,x]=lsim(ss_q4,u00,t1,[x0;xe]);

e=x(:,7:end);
x=x(:,1:6);
x_est=x-e;

figure(21);
for i = 1:6
    subplot(3,2,i);
    plot(t1,x(:,i));
    hold on;
    plot(t1,x_est(:,i));
    legend(['x',num2str(i)],['x_{est}',num2str(i)]);
    title(['Comparison between x',num2str(i),' and x_{est}',num2str(i)]);
    xlabel('Time (sec)');
    ylabel(['x',num2str(i)]);
end
% legend(legend_str);
sgtitle('Comparison between x and x_{est}');

[y,t,x]=lsim(ss_q4, u1, t, [x0;xe]);
e=x(:,7:end);
x=x(:,1:6);
x_est=x-e;

figure(22);
for i = 1:6
    subplot(3,2,i);
    plot(t,x(:,i));
    hold on;
    plot(t,x_est(:,i));
    legend(['x',num2str(i)],['x_{est}',num2str(i)]);
    title(['Comparison between x',num2str(i),' and x_{est}',num2str(i)]);
    xlabel('Time (sec)');
    ylabel(['x',num2str(i)]);
end
% legend(legend_str);
sgtitle('Response of x with input u=[1 0] and initial state x0');



%%
% 不知道输出反馈能否使内部稳定
% %   2. output feedback
% [sys1,sys2,Kd]=decoupler_of(A,B,C2);


%% Q5
% Assume that you only have three cheap sensors to measure the output. Design a controller such 
% that the plant (vehicle) can operate around the set point as close as possible at steady state even 
% when step disturbances are present at the plant input. Plot out both the control and output signals.
% In your simulation, you may assume the step disturbance for the two inputs, w= [?1, 1]T
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
%%
figure;
curve=plot(controlSignal.time(300:end,1),controlSignal.signals(1).values(300:end,1),controlSignal.time(300:end,1),controlSignal.signals(1).values(300:end,2));
grid on; 
legend('Control channel 1','Control channel 1');
xlabel('Time (sec)'), ylabel('Amplitude'); 
title('Control cost');















