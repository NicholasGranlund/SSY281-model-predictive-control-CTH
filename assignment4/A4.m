%% ASSIGNMENT 4
% By: Nicholas Granlund

close all
clear all
clc



%% -----------------------------------------------
% 1(a)
close all


% H-representation
A = [ 0  1
     -1  0
     -1 -1
      1  1];

b = [0 0 1 1]';

PHrep = Polyhedron(A,b);


% V-representation
V1 = [0 0]; V2 = [1 0]; V3 = [0 -1]; 
R = [1 -1];
PVrep = Polyhedron('V',[V1; V2; V3],'R',R);


% Plot
custom_color = [0/255 64/255 115/255];
figure()
subplot(1,2,1)
plot(PHrep,'color',custom_color)
title('H-representation')
subplot(1,2,2)
plot(PVrep,'color',custom_color)
title('V-representation')



%% -----------------------------------------------
% 1(b)
close all
clear all


% Polyhedron P
A1 = [0  1
      1  0
      0 -1
     -1  0];

b1 = [2 2 2 2]';

P = Polyhedron('A',A1,'b',b1);


% Polyhedron Q
A2 = [-1 -1
       1  1
       1 -1 
      -1  1];
b2 = [1 1 1 1]';

Q = Polyhedron('A',A2,'b',b2);


% Different polyhedrons
poly1 = P;
poly2 = Q;
poly3 = P+Q;
poly4 = P-Q;
poly5 = (P-Q)+Q;
poly6 = (P+Q)-Q;
poly7 = (Q-P)+P;
poly8 = (Q+P)-P;


% Plot the polyhedrons
figure()
custom_color = [0/255 64/255 115/255];
subplot(4,2,1)
plot(poly1,'color',custom_color)
title('P')
axis([-3 3 -3 3])

subplot(4,2,2)
plot(poly2,'color',custom_color)
title('Q')
axis([-3 3 -3 3])

subplot(4,2,3)
plot(poly3,'color',custom_color)
title('P+Q')
axis([-3 3 -3 3])

subplot(4,2,4)
plot(poly4,'color',custom_color)
title('P-Q')
axis([-3 3 -3 3])

subplot(4,2,5)
plot(poly5,'color',custom_color)
title('(P-Q)+Q')
axis([-3 3 -3 3])

subplot(4,2,6)
plot(poly6,'color',custom_color)
title('(P+Q)-Q')
axis([-3 3 -3 3])

subplot(4,2,7)
plot(poly7,'color',custom_color)
title('(Q-P)+P')
axis([-3 3 -3 3])

subplot(4,2,8)
plot(poly5,'color',custom_color)
title('(Q+P)-P')
axis([-3 3 -3 3])





%% -----------------------------------------------
% FORWARD AND BACKWARD REACHABILITY
% 2(a)
close all
clear all
clc

%mpt_demo2

A = [0.8   0.4
    -0.4   0.8];

Ain = [1  0
       0  1
      -1  0
       0 -1
       1  1
       1 -1
      -1  1
      -1 -1];

bin = [1 1 1 1 1.5 1.5 1.5 1.5]';


% Define set S
S = Polyhedron('A',Ain,'b',bin);

% Define set reach(S)
S_Reach = Polyhedron('A',Ain*inv(A),'b',bin);

% Plot
custom_color = [0/255 64/255 115/255];
plot(S,'color',custom_color,'alpha',0.6)
hold on
plot(S_Reach,'color',custom_color)
legend('S','reach(S)')




%% -----------------------------------------------
% 2(b)
close all
clc


B = [0 1]';

Au = [1 -1]';
bu = [1 1]';

% polyhedron S is defined in previous question

% Define set U
U = Polyhedron('lb',-1,'ub',1);

% Reachable set
AA = [Ain*inv(A)   -Ain*inv(A)*B
      zeros(2,2)         Au];

bb = [bin; bu];


S_Reach = Polyhedron('A',AA,'b',bb);

% Projection
%S_Reach_proj = A*S + B*U;
S_Reach_proj = projection(S_Reach,[1:2]);



% Plot
figure()
custom_color = [0/255 64/255 115/255];
subplot(1,2,1)
plot(S_Reach,'color',custom_color,'alpha',0.5)
hold on
plot(S,'color',custom_color)
title('S and S_{tot} in 3D')
legend('S_{tot}','S')
xlabel('x_1','FontSize',12)
ylabel('x_2','FontSize',12)
zlabel('u','FontSize',12)

subplot(1,2,2)
plot(S_Reach_proj,'color',custom_color,'alpha',0.5)
hold on
plot(S,'color',custom_color)
title('S and reach(S). A projection on x_1-x_2 plane')
legend('reach(S)','S')
xlabel('x_1','FontSize',12)
ylabel('x_2','FontSize',12)


%% -----------------------------------------------
% 2(b)
close all
clc


% polyhedron S is defined in previous question

% Define set U
U = Polyhedron('lb',-1,'ub',1);

% Reachable set
AA = [Ain*A      Ain*B
      zeros(2,2)   Au];

bb = [bin; bu];


S_pre = Polyhedron('A',AA,'b',bb);


% Projection
S_pre_proj = projection(S_pre,[1:2]);

% Plot
figure()
custom_color = [0/255 64/255 115/255];
subplot(1,2,1)
plot(S_pre,'color',custom_color,'alpha',0.5)
hold on
plot(S,'color',custom_color)
title('S and S_{tot} in 3D')
legend('S_{tot}','S')
xlabel('x_1','FontSize',12)
ylabel('x_2','FontSize',12)
zlabel('u','FontSize',12)

subplot(1,2,2)
plot(S_pre_proj,'color',custom_color,'alpha',0.5)
hold on
plot(S,'color',custom_color)
title('S and Pre(S). Projection on x_1-x_2 plane')
legend('pre(S)','S')
xlabel('x_1','FontSize',12)
ylabel('x_2','FontSize',12)



%% -----------------------------------------------
% PERSISTENT FEASIBILITY
% 3(a)
close all
clear all
clc

A=[0.9   0.4
  -0.4   0.9];

B=[0;1];

x0 = [2;0];

% create model in MPT3 interface
model = LTISystem('A',A,'B',B);


% constraints on inputs and states
model.u.min = -0.1;
model.u.max = 0.1;
model.x.min = [-3;-3];
model.x.max = [ 3; 3];

% Quadratic state penalty
Q = eye(2);
model.x.penalty = QuadFunction(Q);

% Quadratic input penalty
R = 1;
model.u.penalty = QuadFunction(R);

% Add terminal penalty and terminal set
Xf = Polyhedron([0 0]);
model.x.with('terminalSet');
model.x.terminalSet = Xf;


% Iterate through N
for N=25:50

    % Synthesize MPC with prediction horizon N
    mpc1 = MPCController(model,N)

    % Test feasability of MPC
    [~, feasible, openloop] = mpc1.evaluate(x0);


    if feasible
        fprintf('The shortest horizon is N=%.0f',N)
        break
    end
end





% Plot controller (states and input)
figure('Position',[150 250 500 500])
custom_color = [0/255 64/255 115/255];
subplot(2,1,1)
hold on
plot([0:N],openloop.X(1,:),'linewidth',2,'Color',custom_color)
plot([0:N],openloop.X(2,:),'linewidth',2,'Color',custom_color*2)
plot([0:N-1],openloop.U,'linewidth',2)
legend('x_1','x_2','u')
xlabel('k'); ylabel('x,u')
title('System states and input','FontSize',16)
grid on

% Plot 2D state trajectory
subplot(2,1,2)
plot(openloop.X(1,:),openloop.X(2,:),'linewidth',2,'color',custom_color)
hold on
plot(0,0,'o','MarkerFaceColor',custom_color,'MarkerEdgeColor','k','MarkerSize',8)
plot(2,0,'o','MarkerFaceColor','auto','MarkerEdgeColor','k','MarkerSize',8)
legend('state trajectory','terminal set','initial set')
xlabel('x_1'); ylabel('x_2')
title('System trajectory','FontSize',16)
xlim([-1.7 2.1]); ylim([-2 1.5])
grid on





% BONUS PLOT 1

% constraint sets represented as polyhedra
X = Polyhedron('lb',model.x.min,'ub',model.x.max);
U = Polyhedron('lb',model.u.min,'ub',model.u.max);

% Printing parameters
figure('Position',[800 250 1000 500])
subplot(1,2,1)
hold on
label_font_size = 14;
tick_font_size  = 10;
line_width      = 0.8;
axeswidth       = 0.2;

for i=1:N

    % compute forward reachable set
    XN_forward = model.reachableSet('X', Polyhedron([2 0]), 'U', U, 'N',i ,'direction', 'forward');

    % intersect with the state constraints
    XN_forward = XN_forward.intersect(X).minHRep();

    %  plot the feasible set
    plot(XN_forward,'color',custom_color,'linewidth',2,'alpha',0.4)
    % axis([model.x.min(1),model.x.max(1),model.x.min(2),model.x.max(2)])
    plot(openloop.X(1,1:i+1),openloop.X(2,1:i+1),'.-','linewidth',3,'color','k')
    xlim([-3 2.5]); ylim([-3 3])
    plot(0,0,'o','MarkerFaceColor','white','MarkerEdgeColor','k','MarkerSize',8)
    plot(2,0,'o','MarkerFaceColor','white','MarkerEdgeColor','k','MarkerSize',8)
    title('Forward reachable set, N=',i,'FontSize',16)
    xlabel('x_1'); ylabel('x_2')
    pause(0.05)

end



% BONUS PLOT 2

% Printing parameters
%figure('Position',[800 250 750 500])
subplot(1,2,2)
hold on
label_font_size = 14;
tick_font_size  = 10;
line_width      = 0.8;
axeswidth       = 0.2;

for i=1:N

    % compute backwards reachable set
    XN_backward = model.reachableSet('X', Polyhedron([0 0]), 'U', U, 'N',i ,'direction', 'backward');
   
    % intersect with the state constraints
    XN_backward = XN_backward.intersect(X).minHRep();

    %  plot the feasible set
    plot(XN_backward,'color',custom_color,'linewidth',2,'alpha',0.4)

    plot(openloop.X(1,end:end-i),openloop.X(2,end:end-1),'.-','linewidth',3,'color','k')
    xlim([-3 2.5]); ylim([-3 3])
    plot(0,0,'o','MarkerFaceColor','white','MarkerEdgeColor','k','MarkerSize',8)
    plot(2,0,'o','MarkerFaceColor','white','MarkerEdgeColor','k','MarkerSize',8)
    title('Backward reachable set, N=',i,'FontSize',16)
    xlabel('x_1'); ylabel('x_2')
    pause(0.05)

end





%% -----------------------------------------------
% 3(b)
clc
close all



% Set prediction horizon
N = 2;

% Change terminal set
Cinf = model.invariantSet();
Xf = Cinf;
model.x.with('terminalSet');
model.x.terminalSet = Xf;

% Synthesize new MPC
mpc2 = MPCController(model,N)

% Test feasability of MPC
[~, feasible, ~] = mpc2.evaluate(x0);


if feasible
    fprintf('Controller is still feasible!\n')
else
    fprintf('Controller infeasible...\n')
end

% Plot
figure()
custom_color = [0/255 64/255 115/255];
plot(Cinf,'color',custom_color,'alpha',0.5)
hold on
plot(0,0,'o','MarkerFaceColor',custom_color,'MarkerEdgeColor','k','MarkerSize',8)
plot(2,0,'o','MarkerFaceColor','auto','MarkerEdgeColor','k','MarkerSize',8)
plot(openloop.X(1,:),openloop.X(2,:),'linewidth',2,'color',custom_color)
legend('Xf=C_{\infty}','terminal set','initial set','trajectory')




%% -----------------------------------------------
% 3(c)
close all
clc

% Redefine model
model = LTISystem('A', A, 'B', B);
model.x.min = [-3;-3];
model.x.max = [3 3];
model.u.min = -0.1;
model.u.max = 0.1;

% Terminal state constraint set (controller 1)
Xf_cont1 = Polyhedron([0 0]);         % just the origin

% Terminal state constraint set (controller 2)
Xf_cont2 = model.invariantSet();     % Entire control-invariant set

% Set of feasible initial states
Xfeasible_cont1 = model.reachableSet('X',Xf_cont1,'U',U,'N',26,'direction','backward');
Xfeasible_cont2 = model.reachableSet('X',Xf_cont2,'U',U,'N',2,'direction','backward');

% Plot the sets
figure()
hold on
plot(Xfeasible_cont2,'color',custom_color,'alpha',0.5)
plot(Xfeasible_cont1,'color',custom_color)
plot(2,0,'o','MarkerFaceColor','auto','MarkerEdgeColor','k','MarkerSize',8)
legend('X_{N2}    for X_f = C_{\infty}, N=2','X_{N1}    for X_f = 0,    N=26','initial state [2 0]')
xlabel('x_1'); ylabel('x_2')
title('Set of feasible initial states')

clc
Xfeasible_cont1
Xfeasible_cont2








