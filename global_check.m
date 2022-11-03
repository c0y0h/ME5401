function [] = global_check(ss,u1,u2,t,x0)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%   u=[1 0]
[y,t,x]=lsim(ss,u1,t,x0);
q1_info = stepinfo(y,t);

if size(y,2)==2
    figure;
    for i = 1:2
        plot(t,y(:,i));
        if i==1
            legend_str{i} = ['y1: overshoot: ', num2str(q1_info(i).Overshoot),'%, ', ...
                        '2% setting time: ', num2str(q1_info(i).SettlingTime),'s'];
        else
            legend_str{i} = 'y2';
        end
        hold on;
    end
    legend(legend_str);
    xlabel('Time (sec)');
    ylabel('y');
    sgtitle('Output response with input u=[1 0]');
    
else

    figure;
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
end


%   u=[0 1]
[y,t,x]=lsim(ss,u2,t,x0);
q1_info = stepinfo(y,t);

if size(y,2)==2
    
    figure;
    for i = 1:2
        plot(t,y(:,i));
        if i==2
            legend_str{i} = ['y2: overshoot: ', num2str(q1_info(i).Overshoot),'%, ', ...
                        '2% setting time: ', num2str(q1_info(i).SettlingTime),'s'];
        else
            legend_str{i} = 'y1';
        end
        hold on;
    end
    legend(legend_str);
    xlabel('Time (sec)');
    ylabel('y');
    sgtitle('Output response with input u=[0 1]');

else

    figure;
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
end

end

