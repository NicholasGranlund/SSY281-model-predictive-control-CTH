% PLOTMINTIME1
% By: Nicholas Granlund


% Plot
figure('Position',[250 250 700 600])
custom_color1 = [0/255 64/255 115/255];
custom_color2 = [117/255 151/255 68/255];
xtic = 0:h:h*(length(x)-1);

subplot(3,1,1)
hold on
plot(xtic,x(1,:),'LineWidth',2,'color',custom_color1)
plot(xtic,x(2,:),'LineWidth',2,'color',custom_color1*2)
plot(xtic,x(3,:),'LineWidth',2)
plot(xtic,x(4,:),'LineWidth',2,'color',custom_color2)
title('System states')
xlabel('Time [seconds]')
ylabel('States [rad, rad/sec]')
xline(N*h,'--','LineWidth',2)
legend('x_1','x_2','x_3','x_4','Minimum-time')
xticks(unique(round(get(gca, 'xTick'),1)));
grid on


% Measurements
subplot(3,1,2)
stairs(xtic, y,'LineWidth',2,'color',custom_color1)
title('System measurements')
xlabel('Time [seconds]')
ylabel('Torsional torque [Nm]')
xline(N*h,'--','LineWidth',2)
legend('Torsional torque T','Minimum-time')
xticks(unique(round(get(gca, 'xTick'),1)));
grid on


% Inouts
subplot(3,1,3)
stairs(xtic,u,'Linewidth',2,'color',custom_color1*2)
xline(N*h,'--','LineWidth',2)
yline(-200,':','LineWidth',2)
yline(200,':','LineWidth',2)
legend('u','Minimum-time','Lower bound','Upper bound')
title('System input')
xlabel('Time [seconds]')
ylabel('Input voltage [V]')
ylim([-250 250])
xticks(unique(round(get(gca, 'xTick'),1)));
grid on


% End script