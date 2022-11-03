function [] = task4_check(ss,t,u0,u1,u2,u3,x0)
%TASK4_CHECK Summary of this function goes here
%   Detailed explanation goes here

%   check if external stable (x0)

% u1=[1 0]
[y, t, x] = lsim(ss, u1, t, x0);

figure;
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
[y, t, x] = lsim(ss, u2, t, x0);

figure;
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
[y, t, x] = lsim(ss, u3, t, x0);

figure;
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
[y, t, x] = lsim(ss, u0, t, x0);

figure;
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
p=pole(ss);
for i=1:size(p)
    if real(p(i))>0
        disp(['Find pole with negative part:',num2str(p(i))]);
    end
end

end

