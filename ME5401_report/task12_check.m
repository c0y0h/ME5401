function [] = task12_check(ss,t,u0,x0,K11)
%TASK1_CHECK Summary of this function goes here
%   Detailed explanation goes here

% 6 state responses to x0 with zero inputs
[y,t,x]=lsim(ss,u0,t,x0);

figure;
for i = 1:6
    subplot(3,2,i);
    plot(t,x(:,i));
    title(['Response for x',num2str(i)]);
    xlabel('Time (sec)');
    ylabel(['x',num2str(i)]);
end
sgtitle('Six state responses with zero inputs');

% monitor the control signal size
u=-x*K11';
figure;
for i = 1:2
    subplot(2,1,i);
    plot(t,u(:,i));
    title(['Control signal u',num2str(i)]);
    xlabel('Time (sec)');
    ylabel(['u',num2str(i)]);
end
% legend(legend_str);
sgtitle('Control signal size');

end

