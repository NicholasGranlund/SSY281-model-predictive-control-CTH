% PLOT_DISTURBED_SYSTEM
% Script by: Nicholas Granlund

% Plot state 1
figure()
subplot(4,1,1)
title('Ball angle \theta_1')
hold on
plot(x(1,1:end-1),'linewidth',2)
plot(ysp(1,1:end-1),'--k')
plot(dinit,x(1,dinit),'xr','MarkerSize',12)
grid on
legend('\theta_1','setpoint','disturbance instantiation')
xlabel('k = [0 1000]')
ylabel('[rad]')

% Plot state 2
subplot(4,1,2)
title('Ball speed {\partial \theta_1} / {\partial t}')
hold on
plot(x(2,1:end-1),'linewidth',2)
plot(ysp(2,1:end-1),'--k')
plot(dinit,x(2,dinit),'xr','MarkerSize',12)
grid on
legend('{\partial \theta_1} / {\partial t}','setpoint','disturbance instantiation')
xlabel('k = [0 1000]')
ylabel('[rad/s]')

% Plot state 3
subplot(4,1,3)
title('Wheel angle \theta_2')
hold on
plot(x(3,1:end-1),'linewidth',2)
plot(ysp(3,1:end-1),'--k')
plot(dinit,x(3,dinit),'xr','MarkerSize',12)
grid on
legend('\theta_2','setpoint','disturbance instantiation')
xlabel('k = [0 1000]')
ylabel('[rad]')

% Plot control signal
subplot(4,1,4)
title('Control u')
hold on
plot(u(1:end-1),'linewidth',2)
plot(dinit,u(1,dinit),'xr','MarkerSize',12)
grid on
legend('input u','disturbance instantiation')
xlabel('k = [0 1000]')
ylabel('[no unit]')











