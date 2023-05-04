%% ASSIGNMENT 6
% BY: Nicholas Granlund


%% 1(a)
close all
clear all
clc

n = 4;

A = [1.0041    0.0100    0.0000    0.0000
     0.8281    1.0041    0.0000   -0.0093
     0.0002    0.0000    1.0000    0.0098
     0.0491    0.0002    0.0000    0.9629];

B = [0.0007
     0.1398
     0.0028
     0.5605];

S = eye(n);

eigenvalues = eig(A)
Q = - ((A'*S*A) - S)


% S is symetric and eig positive
% Q is not positive definite, thus not stable
% Eigenvalues of A is not all inside unit dics, unstable





%% 1(b)
close all
clc
format short

K = [114.3879   12.7189   -1.2779   -1.5952];
S = eye(n);
A_cl = A-(B*K);
eigenvalues_cl = eig(A_cl)


Q_cl = - ((A_cl' * S * A_cl) - S)


% S is symetric and eig positive
% Q is not positive semidefinite
% Eigenvalues are inside unit disc, system stable




%% 1(c)
close all
clc
format short

Q = eye(n);

S = dlyap(A_cl',Q)
eigenvalues__S = eig(S)
S_symmetric = S == S'


% S is symetric and eig positive
% Q is positive definite
% Eigenvalues are inside unit disc, system stable








%% 2(a)
close all
clear all
clc


% No effect. 



%% 2(b)
close all
clc

A = [1.0041    0.0100    0.0000    0.0000
     0.8281    1.0041    0.0000   -0.0093
     0.0002    0.0000    1.0000    0.0098
     0.0491    0.0002    0.0000    0.9629];

B = [0.0007
     0.1398
     0.0028
     0.5605];

Q = eye(4);
R=1;
Pf=Q;


[P_N, N, K] = find_minimum_horiz_DP(A,B,Q,R,Pf);
K



%% 2(c)
close all
clc

S_lyap = P_N
Q_lyap = -(((A-(B*K))'*S_lyap * (A-(B*K))) - S_lyap)




% Q is not positive definite, not Lyapunov!



%% 2(d)
close all
clc

% Lets set Pf as riccati solution!
Q = eye(4);
R = 1;
[Pf,~,K_ric] = idare(A',B,Q,R);


S_lyap = Pf
Q_lyap = -(((A-(B*K_ric'))'*S_lyap * (A-(B*K_ric'))) - S_lyap)



% Q is not positive definite, not Lyapunov!








%% 3(a)







%% 4(a)

close all
clear all
clc

% system
A = [2 0.1;
     0 1.5];
B = [0;1];

% initial condition
x0 = [-0.1; 1.3];

% parameters
Q = eye(2);
R = 100;
N = 2;

% Define model
model = LTISystem('A', A, 'B',B);
% model.x.min = [-0.5; -0.5];
% model.x.max = [0.5; 0.5];
model.u.min = -1;
model.u.max =  1;
U = Polyhedron('lb', model.u.min, 'ub', model.u.max);

model.x.penalty = QuadFunction(Q);
model.u.penalty = QuadFunction(R);

% Terminal constraint
Xf_terminalconstraint = Polyhedron('A',[eye(2);-eye(2)],'b',[0.5;0.5;0.5;0.5]); 


plot(Xf_terminalconstraint)

% control invariant part of the terminal constraint
Cinf = model.invariantSet();
plot(Cinf)

Xf = Cinf.intersect(Xf_terminalconstraint);      
model.x.with('terminalSet');
model.x.terminalSet = Xf;


plot(Xf)


XN = model.reachableSet('X', Xf, 'U', U, 'direction', 'backward','N',N);
plot(XN)



%% Pf = I
close all
clc
custom_color = [0/255 64/255 115/255];

% Set terminal penalty
Pf = eye(2);
model.x.with('terminalPenalty');
model.x.terminalPenalty = QuadFunction(Pf);

% Synthesize controller
tf = 20;
mpc  = MPCController(model, N);
loop = ClosedLoop(mpc, model);
data  = loop.simulate(x0, tf);

figure()
subplot(1,2,1)
plot(Xf, 'alpha', 0.2,'color',custom_color)
hold on
plot(XN, 'alpha', 0.6,'color',custom_color)
plot(data.X(1,:),data.X(2,:), 'Linewidth', 3,'color','k')
scatter(x0(1), x0(2), 'o','markerfacecolor','white','MarkerEdgeColor','k')
scatter(data.X(1,end), data.X(2,end),'o','markerfacecolor','white','MarkerEdgeColor','k')
legend('X_f','X_0','State trajectory')
title('P_f = I_4')





% Pf = riccati
custom_color = [0/255 64/255 115/255];

% Set terminal penalty
Pf = idare(A,B,Q,R);
model.x.with('terminalPenalty');
model.x.terminalPenalty = QuadFunction(Pf);

% Synthesize controller
mpc  = MPCController(model, N);
loop = ClosedLoop(mpc, model);
data  = loop.simulate(x0, tf);

subplot(1,2,2)
plot(Xf, 'alpha', 0.2,'color',custom_color)
hold on
plot(XN, 'alpha', 0.6,'color',custom_color)
plot(data.X(1,:),data.X(2,:), 'Linewidth', 3,'color','k')
scatter(x0(1), x0(2), 'o','markerfacecolor','white','MarkerEdgeColor','k')
scatter(data.X(1,end), data.X(2,end),'o','markerfacecolor','white','MarkerEdgeColor','k')
legend('X_f','X_0','State trajectory')
title('P_f = riccati sol')



