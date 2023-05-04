%% Assignment 5
% By: Nicholas Granlund
close all
clear all
clc
%addpath('./functions')


%% Question 1
% (a)
% Define state-space
close all
clc

A = [1.2  1
      0   1];
B = [0;1];

% Create model
model = LTISystem('A', A, 'B', B);

% Input and state constraints
model.x.min = [-15; -15];
model.x.max = [15; 15];
model.u.min = -1;
model.u.max = 1;

% Create polyhedrons for these constraint
X = Polyhedron('lb',model.x.min,'ub',model.x.max);
U = Polyhedron('lb',model.u.min,'ub',model.u.max);

% RHC parameters
N = 4;
Q=eye(2); 
R=100;

% Set model penalties
model.x.penalty = QuadFunction(Q);
model.u.penalty = QuadFunction(R);

% Xf=0 as terminal set
Xtarget = Polyhedron([0 0]);
model.x.with('terminalSet');
model.x.terminalSet = Xtarget;

% Pf = the penalty function
% solve discrete algebraic riccati equation
[H,~,~] = idare(A,B,Q,R);
H

% Set Pf as x'*H*x 
Pf = QuadFunction(H);
model.x.with('terminalPenalty');
model.x.terminalPenalty = Pf;


% Plot 
figure()
hold on
custom_color = [0/255 64/255 115/255];
for i=1:N
    % Calculate backwards reachable set for N=4
    XN = X.intersect(model.reachableSet('X',Xtarget, 'U',U, 'N',i, 'direction','backward'));
    plot(XN,'color',custom_color,'alpha',0.5)
    hold on
end

% Plot points
plot(0,0,'o','MarkerFaceColor','auto','MarkerEdgeColor','k','MarkerSize',8)
xlabel('x_1')
ylabel('x_2')
legend('X_N, N=1','X_N, N=2','X_N, N=3','X_N, N=4','(0,0)')
title('X_N')
axis equal










%% Question 1
% (b)

close all
clc

% Initial condition
x0 = [7; -4];

% Simulation time
tf=40;

% Prediction horizons
% N=6 is the feasible limit for x0=[7; -4]
N = [10, 15, 20];

figure('Position',[400 250 1000 500])
for i=1:length(N)

    % Create controller with prediction horizon N
    controller = MPCController(model, N(i));

    % Create closed loop of controller and model.
    loop = ClosedLoop(controller, model);

    % Simulate closed loop with x0 from k=0 to k=tf
    simulation = loop.simulate(x0,tf);

    % Predict system
    [~, feasible, openloop] = controller.evaluate(x0);

    % Ensure feasibilty
    if ~feasible
        fprintf('RHC not feasible with pred horizon N=%.0f \n',N(i))
        break
    end
    
    % Plot simulated system
    subplot(2,length(N),i)
    hold on, grid on;
    plot(0:tf, simulation.X(1,:), 'Linewidth',2,'color',custom_color);
    plot(0:tf, simulation.X(2,:), 'Linewidth',2,'color',custom_color*2);
    stairs(0:tf-1, simulation.U,'LineWidth',2)

    % Plot predicted system at k=0
    plot(0:N(i), openloop.X','.', 'Linewidth',2, 'Color','black');
    title(sprintf('System, N=%.0f\ncost=%.2f',N(i),openloop.cost))
    xlim([0 tf])
    xlabel('time instant k'); ylabel('x_1, x_2, u')
    legend('x_1','x_2','u','system prediction')

    % Plot states in x1x2 plane
    subplot(2,length(N),i+length(N))
    hold on; grid on
    plot(X.intersect(model.reachableSet('X',XN, 'U',U, 'N',N(i), 'direction','backward')),'color',custom_color,'alpha',0.7)
    plot(simulation.X(1,:),simulation.X(2,:),'color','k','LineWidth',2)
    plot(x0(1),x0(2),'o','MarkerFaceColor',custom_color,'MarkerEdgeColor','k','MarkerSize',8)
    plot(0,0,'o','MarkerFaceColor','auto','MarkerEdgeColor','k','MarkerSize',8)
    legend('Set of feasible initial points','system trajectory','Initial states','origin/setpoint (0,0)')
    xlabel('x_1')
    ylabel('x_2')

end














%% Question 1
% (c)

% THIS SECTION RUNS ON A SEPARATE COMPUTER
% DONT WORK ON MAC & MATLAB R2022a...

close all
clear all
clc

% Model
A =  [1.2 1; 0 1];
B = [0; 1];
model = LTISystem('A', A, 'B', B);

% Constraints
model.x.min = [-15; -15];
model.x.max = [15; 15]; 
model.u.min = -1;
model.u.max = 1;

% Penalties
Q = eye(2);
R =100;
model.x.penalty = QuadFunction(Q);
model.u.penalty = QuadFunction(R);

% Initial set
P = Polyhedron('lb', model.x.min, 'ub', model.x.max);
model.x.with('initialSet');
model.x.initialSet = P;

% Terminal penalty
[H,~,~] = idare(A,B,Q,R);
Pf = QuadFunction(H);
model.x.with('terminalPenalty');
model.x.terminalPenalty = Pf;

% Terminal set
Xtarget = Polyhedron('A',[eye(2); -1*eye(2)],'B',[0.01 0.01 0.01 0.01]);
model.x.with('terminalSet');
model.x.terminalSet = Xtarget;

% Synthesize controller
N=20;
ctrl = MPCController(model,N);

% Create explicit controller
explicit_ctrl = ctrl.toExplicit();

% Plot 1
figure()
explicit_ctrl.partition.plot()
hold on
P.plot('wire', true, 'linestyle', '-.', 'linewidth', 2)
xlabel('x_1'); ylabel('x_2')

% Plot 2
figure()
custom_color = [0/255 64/255 115/255];
plot(explicit_ctrl.partition.Set,'color',custom_color,'alpha',0.8)
hold on
P.plot('wire', true, 'linestyle', '-.', 'linewidth', 2)













%% Question 1
% (d)
close all
clc

% Find invariant set
Cinf=model.invariantSet();

% XN = Cinf
XN = Cinf;

% Xf = reach(XN) intersected with Cinf
X = Polyhedron('lb',model.x.min,'ub',model.x.max);
U = Polyhedron('lb',model.u.min,'ub',model.u.max);
Xtarget = model.reachableSet('X',Cinf,'U',U,'N',1,'direction','forward').intersect(Cinf);

% Plot
figure()
hold on
custom_color = [0/255 64/255 115/255];
plot(X,'color',custom_color,'alpha',0.2)
plot(Cinf,'color',custom_color,'alpha',0.6)
plot(Xtarget,'color',custom_color)
plot(0,0,'o','MarkerFaceColor','auto','MarkerEdgeColor','k','MarkerSize',8)
legend('X','C_{\infty}','X_f = reach(C_{\infty}) \cap C_{\infty}','Origin/setpoint (0,0)')
xlabel('x_1')
ylabel('x_2')
axis equal















%% Question 2
% (a)
close all
clear all
clc
format short

% Parameters
Ls      = 1.0;         % Shaft length
ds      = 0.02;        % Shaft diameter
JM      = 0.5;         % Motor inertia
betaM   = 0.1;         % Motor viscous friction coeff
R       = 20;          % Resistance of armature
kT      = 10;          % Motor constant
roh     = 20;          % Gear ratio
ktheta  = 1280.2;      % Torsion rigidity
JL      = 50*JM;       % Nominal load inertia
betaL   = 25;          % Load viscous friction coeff


% Continuous state-space model
Ac   =   [0             1              0              0;
   -ktheta/JL       -betaL/JL     ktheta/(roh*JL)     0;
          0             0              0              1;
    ktheta/(roh*JM)     0      -ktheta/(roh*roh*JM)   -(betaM  +((kT*kT)/R)) / JM];

Bc = [0;  0;  0;  kT/(R*JM)];

Cc = [ktheta   0    -ktheta/roh   0];

Dc = [];

sys = ss(Ac,Bc,Cc,Dc);

% Sampling interval
h = 0.1;        % seconds

% Discretize
sysd = c2d(sys,h);

% Extract model matrices
A = sysd.A
B = sysd.B
C = sysd.C
















%% Question 2
% (b)
close all
clc

% initial state
x0 = [0 2.5 0 75]';

% Sizes
n = size(A,2);
m = size(B,2);

% Create system model
model = LTISystem('A', A, 'B', B, 'C', C); 

% Input constraints
model.u.min = -200;
model.u.max = 200;
U = Polyhedron('lb',model.u.min,'ub',model.u.max);

% Terminal set
% (Any set combination where x2 and x4 = 0)
Xtarget = Polyhedron('Ae', [0 1 0 0; 0 0 0 1], 'be', [0;0]);
model.x.with('terminalSet');
model.x.terminalSet = Xtarget;

% Determine the minimum time
XN = Polyhedron();
N = 0;
XNN = [Xtarget];
while ~XN.contains(x0)
    % increment while not feasible
    N = N+1;
    XN = model.reachableSet('X', Xtarget, 'U', U, 'direction','backward','N',N);
    XNN = [XNN; XN];
end

% Print results
clc
fprintf('\nThe shortest possible horizon is obtainable with N=%.0f',N)
fprintf('\nMinimum Time = %.2f [s]\n\n',N*h)


% Initialize vectors for simulation
x = zeros(4,N);
y = zeros(1,N);
u = zeros(1,N);
x(:,1) = x0;

% Simulate the system
for i=1:N

    % Initial state for the time instant k
    XK = Polyhedron('Ae',eye(n),'Be',x(:,i));

    % Target set for the time instant k
    XK_target = XNN((N+1)-i);

    % Set terminal set as target
    model.x.terminalSet = XK_target;

    % Synthesize controller for this
    mpc = MPCController(model, 1);

    % Calculate optimal control
    u(:,i) = mpc.evaluate(x(:,i));

    % Drive system with optimal control
    x(:,i+1) = A*x(:,i) + B*u(:,i);
    y(:,i+1) = C*x(:,i+1);

    % repeat

end

% Pad data
u(end+1) = u(end);

% Plot
plotmintime1;


















%% Question 2
% (c)

close all
clc

% initial state
x0 = [0 2.5 0 75]';

% Sizes
n = size(A,2);
m = size(B,2);

% Create system model
model = LTISystem('A', A, 'B', B, 'C', C);     % unstable A

% Input constraints
model.u.min = -200;
model.u.max = 200;
U = Polyhedron('lb',model.u.min,'ub',model.u.max);
model.u.with('setConstraint');
model.u.setConstraint = U;


% State constraint
% Measurements y=Cx are constrained
X = Polyhedron('A',[C;-C],'B',[150;150]);
model.x.with('setConstraint');
model.x.setConstraint = X;

% Terminal set
% (Any set combination where x2 and x4 = 0)
Xtarget = Polyhedron('Ae', [0 1 0 0; 0 0 0 1], 'be', [0;0]);
model.x.with('terminalSet');
model.x.terminalSet = Xtarget;


% Determine the shortest horizon
XN = Xtarget;
N = 0;
XNN = [Xtarget];
while ~XN.contains(x0)
    % increment while not feasible
    N = N+1;
    XN = model.reachableSet('X', XN, 'U', U, 'direction','backward','N',1);
   
    % Intersect with constrained set
    XN = XN.intersect(X);
    XNN = [XNN; XN];
end


% Print results
clc
fprintf('\nThe shortest possible horizon is obtainable with N=%.0f',N)
fprintf('\nMinimum Time = %.2f [s]\n\n',N*h)

% Initialize vectors
x = zeros(4,N);
y = zeros(1,N);
u = zeros(1,N);
x(:,1) = x0;

% Simulate the system
for i=1:N

    % Initial state for the time instant k
    XK = Polyhedron('Ae',eye(n),'Be',x(:,i));

    % Target set for the time instant k
    XK_target = XNN((N+1)-i);

    % Set terminal set as target
    model.x.terminalSet = XK_target;

    % Synthesize controller for this
    mpc = MPCController(model, 1);

    % Calculate optimal control
    u(:,i) = mpc.evaluate(x(:,i));

    % Drive system with optimal control
    x(:,i+1) = A*x(:,i) + B*u(:,i);
    y(:,i+1) = C*x(:,i+1);

    % repeat

end

% Pad data
u(end+1) = u(end);

% Plot
plotmintime2;















%% Question 2
% (d)

close all
clc

% initial state
x0 = [0 2.5 0 75]';

% Sizes
n = size(A,2);
m = size(B,2);

% Create system model
model = LTISystem('A', A, 'B', B, 'C', C); 

% Input constraints
model.u.min = -200;
model.u.max = 200;
U = Polyhedron('lb',model.u.min,'ub',model.u.max);
model.u.with('setConstraint');
model.u.setConstraint = U;

% State constraint
% Measurements y=Cx are constrained
X = Polyhedron('A',[C;-C],'B',[150;150]);
model.x.with('setConstraint');
model.x.setConstraint = X;

% Target set
Xtarget = Polyhedron('lb',[-10; -0.01; -10; -0.01],'ub',[10; 0.01; 10; 0.01]);

% Largest control invariant set in terminal set Xf
Xf = model.invariantSet().intersect(Xtarget);
model.x.with('terminalSet');
model.x.terminalSet = Xf;

% Determine the shortest horizon
% Drive system towards, just not Xf, but the control invariant part of Xf!
XN = Xf;
N = 0;
XNN = [Xf];

% Please be patient, this takes approx 30 sec...
while ~XN.contains(x0)
    % increment while not feasible
    N = N+1;

    XN = model.reachableSet('X', XN, 'U', U, 'direction','backward','N',1);
   
    % Intersect with constrained set
    XN = XN.intersect(X);

    XNN = [XNN; XN];
end

% Print results
clc
fprintf('\nThe shortest possible horizon is obtainable with N=%.0f',N)
fprintf('\nMinimum Time = %.2f [s]\n\n',N*h)


% Initialize vectors
x = zeros(4,N);
y = zeros(1,N);
u = zeros(1,N);

% Insert initial state
x(:,1) = x0;

% Simulate the system
for i=1:N

    % Initial state for the time instant k
    XK = Polyhedron('Ae',eye(n),'Be',x(:,i));

    % Target set for the time instant k
    XK_target = XNN((N+1)-i);

    % Set terminal set as target
    model.x.terminalSet = XK_target;

    % Synthesize controller for this
    mpc = MPCController(model, 1);

    % Calculate optimal control
    u(:,i) = mpc.evaluate(x(:,i));

    % Drive system with optimal control towards target set
    x(:,i+1) = A*x(:,i) + B*u(:,i);
    y(:,i+1) = C*x(:,i+1);

    % repeat

end


% Continue simulation after reaching Xf
tf = 89;
for i=1:tf

    % Turn off controller (u=0). 
    % Should remain inside control invariant set if we
    % have controlled the system correctly...
    x(:,end+1) = A*x(:,end) + B*u(end);
    y(:,end+1) = C*x(:,end);
    u(end+1) = 0;

    if i==9 || i==tf
        % Plot
        plotmintime3;
    end

end


