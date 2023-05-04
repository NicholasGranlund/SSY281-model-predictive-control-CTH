%% Assignment 1
% Nicholas Granlund

% Gather all functions in folder './functions'
% The following functions should be in the folder './functions':
%           find_minimum_horizon_DP.m
%           find_minimum_horizon_batch.m
%           find_stationary_sol.m
%           simulate_system_DP.m
%           plot_system.m
%           simulate_constrained_system.m
%           ConstrainedRHC.m

addpath('./functions');
close all
clear all
clc
format short

% Parameters
a = -0.9421;
b = 82.7231;
c = 14.2306;
p = -3.7808;
q = 4.9952;
r = 57.1120;

% Sampling interval
h = 0.1;

Ac = [0 1 0 0
      b 0 0 a
      0 0 0 1
      q 0 0 p];

Bc = [0 c 0 r]';

Cc = [1 0 0 0];

Dc = 0;


% (a) Discretize
sysd = expm([Ac, Bc; [0 0 0 0], 0].*h);
Ad = sysd(1:4,1:4)
Bd = sysd(1:4,5)


% (b) Delayed system
tau = 0.8*h;

sysdtau = expm([Ac, Bc; [0 0 0 0], 0].*tau);
Atau = sysdtau(1:4,1:4);
Btau = sysdtau(1:4,5);

sysdhtau = expm([Ac, Bc; [0 0 0 0], 0].*(h-tau));
Ahtau = sysdhtau(1:4,1:4);
Bhtau = sysdhtau(1:4,5);

A = Ahtau*Atau; 
B1 = Ahtau*Btau;
B2 = Bhtau;

% Define A_a and B_a
A_a = [A ,B1;[0 0 0 0], 0]
B_a = [B2; 1]

% Check eigenvalues
eigenvalues_Ad = eig(Ad)
eigenvalues_Aa = eig(A_a)



%% DP solution of the LQ problem
clc
format shortE

h = 0.01;
A = [1.0041    0.0100         0   -0.0000
     0.8281    1.0041         0   -0.0093
     0.0002    0.0000    1.0000    0.0098
     0.0491    0.0002         0    0.9629];

B = [0.0007
     0.1398
     0.0028
     0.5605];

C = [1 0 0 0
     0 0 1 0]; 

% Set parameters
Q = eye(4);
Pf = 10*eye(4);
R = 1;
clc


%2(a) Call function to find minimum horizon
[P_N, N_DP, K_DP] = find_minimum_horiz_DP(A,B,Q,R,Pf);

%2(b) Call function to find stationary solution

% idare
[P_idare,K_idare,~] = idare(A,B,Q,R);

% DP
[P_inf, N, ~] = find_stationary_sol(A,B,Q,R,Pf);

%2(c) Try again with P_inf as terminal P
[~,N_terminal1, ~] = find_minimum_horiz_DP(A,B,Q,R,P_idare);



%% BATCH solution of the LQ problem

[N_batch, K_batch] = find_minimum_horiz_batch(A,B,Q,R,Pf);


%% RHC with DP
clc
close all
Q = eye(4);
Pf = 10*eye(4);

% Simulate the unconstrained system
[x1, u1] = simulate_system_DP(A,B,C,Q,  1,Pf,40);
[x2, u2] = simulate_system_DP(A,B,C,Q,  1,Pf,80);
[x3, u3] = simulate_system_DP(A,B,C,Q,0.1,Pf,40);
[x4, u4] = simulate_system_DP(A,B,C,Q,0.1,Pf,80);

% Plot unconstrained system
plot_system;


%% Constrained receeding horizon

% Define constraints [~, x2, ~, ~, u]
const = [0 1 0 0 8];

% Simulate the constrained system
% OBS Please give up to 20 sec for this to solve...
[x1, u1] = simulate_constrained_system(A,B,C,Q,  1,Pf,40,const);
[x2, u2] = simulate_constrained_system(A,B,C,Q,  1,Pf,80,const);
[x3, u3] = simulate_constrained_system(A,B,C,Q,0.1,Pf,40,const);
[x4, u4] = simulate_constrained_system(A,B,C,Q,0.1,Pf,80,const);
clc
fprintf('QP computations done...')

% Plot constrained system
plot_system;


