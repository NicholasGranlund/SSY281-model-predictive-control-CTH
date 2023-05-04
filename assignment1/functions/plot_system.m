% Script by: Nicholas Granlund

figure()
subplot(4,1,1)
hold on; grid on
title('x_1 ball angle')
plot(0:100,x1(1,1:101),'linewidth',2)
plot(0:100,x2(1,1:101),'linewidth',2)
plot(0:100,x3(1,1:101),'linewidth',2)
plot(0:100,x4(1,1:101),'linewidth',2)
legend('R=1,N=40','R=1,N=80','R=0.1,N=40','R=0.1,N=80')
xlabel('timestep k')
ylabel('[rad]')
ylim([-0.03 0.1])

subplot(4,1,2)
hold on; grid on
title('x_2 ball angular velocity')
plot(0:100,x1(2,1:101),'linewidth',2)
plot(0:100,x2(2,1:101),'linewidth',2)
plot(0:100,x3(2,1:101),'linewidth',2)
plot(0:100,x4(2,1:101),'linewidth',2)
legend('R=1,N=40','R=1,N=80','R=0.1,N=40','R=0.1,N=80')
xlabel('timestep k')
ylabel('[rad/s]')
ylim([-1.5 0.2])


subplot(4,1,3)
hold on; grid on
title('x_3 wheel angle')
plot(0:200,x1(3,1:201),'linewidth',2)
plot(0:200,x2(3,1:201),'linewidth',2)
plot(0:200,x3(3,1:201),'linewidth',2)
plot(0:200,x4(3,1:201),'linewidth',2)
legend('R=1,N=40','R=1,N=80','R=0.1,N=40','R=0.1,N=80')
xlabel('timestep k')
ylabel('[rad]')
ylim([-0.8 0])


subplot(4,1,4)
hold on; grid on
title('u input')
plot(0:30,u1(1:31),'linewidth',2)
plot(0:30,u2(1:31),'linewidth',2)
plot(0:30,u3(1:31),'linewidth',2)
plot(0:30,u4(1:31),'linewidth',2)
legend('R=1,N=40','R=1,N=80','R=0.1,N=40','R=0.1,N=80')
xlabel('timestep k')
ylabel('[no unit]')
ylim([-10 1])

