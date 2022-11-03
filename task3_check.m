function [] = task3_check(ss1,ss2,ss3,u0,t,x0)
%TASK3_CHECK Summary of this function goes here
%   Detailed explanation goes here

% pole influence
[y,t,x]=lsim(ss1,u0,t,x0);
[ydot1,t,xdot1]=lsim(ss2,u0,t,x0);
[ydot2,t,xdot2]=lsim(ss3,u0,t,x0);
figure;
for i = 1:3
    subplot(3,1,i);
    plot(t,y(:,i),'-r',t,ydot1(:,i),'-g',t,ydot2(:,i),'b');
    legend('3 times faster','2 times faster','4 times faster');
    title(['Response for y',num2str(i)]);
    xlabel('Time (sec)');
    ylabel(['y',num2str(i)]);
end
sgtitle('System performance with different observer poles');

% monitor the state estimation error
t1=0:0.05:1.2;
u00 = [zeros(size(t1,2),1),zeros(size(t1,2),1)];

[y,t1,x]=lsim(ss1,u00,t1,x0);

e=x(:,7:end);
x=x(:,1:6);
x_est=x-e;

figure;
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
sgtitle('Comparison between x and x_{est}');

figure;
e1=e(:,1);e2=e(:,2);e3=e(:,3);e4=e(:,4);e5=e(:,5);e6=e(:,6);
plot(t1,e1,'-r', t1,e2,'-b', t1,e3,'-g', t1,e4,'-y', t1,e5,'-c', t1,e6,'-k');
legend('e1','e2','e3','e4','e5','e6');
xlabel('Time (sec)');
ylabel('State estimation error');
title('State estimation error with time');


end
