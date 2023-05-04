%% Assignment 2
% By: Nicholas Granlund
close all
clear all
clc
format short
%addpath('./functions');        


%%  QUESTION 1 - SETPOINT TRACKING
% (a)
clc



% Parameters for the system
A = [1.0041   0.0100   0.0000   0.0000;
     0.8281   1.0041   0.0000  -0.0093;
     0.0002   0.0000   1.0000   0.0098;
     0.0491   0.0002   0.0000   0.9629];

B = [0.0007   0.0100;
     0.1398   1.0000;
     0.0028   0.0000;
     0.5605   0.0000];

C = [1 0 0 0;
     0 0 1 0];

% Setpoint
ys = [pi/18  -pi]';

% Solve system of equations
sol = [eye(length(A))-A   -B;
          C       zeros(2)] \[zeros(4,1); ys];

% xs and us
xs = sol(1:4,1)
us = sol(5:end,1)

%%
% (b)
clc

B1 = B(:,2);

% There are more outputs than inputs (p > m)
p = size(C,1);              % outputs
m = size(B1,2);             % inputs
n = size(A,1);              % states

% min  (C*xs-z_sp)'*Q*(C*xs-z_sp) = xs'*(C'*Q*C)*xs + (-2*z_sp'*Q*C)*xs + z_sp'*Q*z_sp
Q = eye(p);

% Now we define H and f so it fits Matlab format
% H = 2*[C'QC   0
%         0     0];
% f' = [-2ys'QC  0];

H = 2* blkdiag( C'*Q*C , zeros(m) );
f = [ (-2*ys'*Q*C)' ; zeros(m,1) ];

% Define equality constraints. given by Ax = B
Aeq = [eye(n)-A,-B1];
beq = zeros(n,1);

% Solve
sol = quadprog(H ,f,[],[],Aeq,beq,[],[],[]);
xs = sol(1:4,1)
us = sol(end,1)


%%
% (c)
clc

C2 = C(1,:);
zsp = pi/18;

% There are less inputs than outputs (p < m)
p = size(C2,1);         % outputs
m = size(B,2);          % inputs
n = size(A,1);          % states

% min   (us-usp)'*Rs*(us-usp)
Rs = eye(m);

% Now we define H and f so it fits Matlab format
% H = 2*[ 0   0
%         0   Rs];
% f' = [-2usp'Rs  0] = [0  0];

H = 2* blkdiag( zeros(n) ,Rs );

f = [ zeros(n,1) ; zeros(m,1) ];

Aeq = [eye(n)-A -B; C2 0*C2*B];
beq = [zeros(n,1) ; zsp];

options = optimoptions('quadprog','Display','iter');
sol = quadprog(H ,f,[],[],Aeq,beq,[],[],[],options);

xs = sol(1:4,1)
us = sol(5:end,1)


%%  QUESTION 2 - CONTROL OF A BALL AND WHEEL SYSTEM
% (a) Construct the augmented model
clc

% Define system
%    x(k+1) = Ax(k) + Bu(k) + Bpp(k)
%      y(k) = Cx(k) + Cpp(k)

B = B(:,1);
Bp = B;
Cp = [0 1]';

% system 1
Bd1 = zeros(4,1);
Cd1 = [1;1];

% system 2
Bd2 = zeros(4,2);
Cd2 = eye(2);

% system 3
Bd3 = [zeros(4,1) Bp];
Cd3 = eye(2);


% Call function to augment the system
[Aaug1, Baug1, Caug1] = construct_aug_model(A,B,C,Bd1,Cd1);
[Aaug2, Baug2, Caug2] = construct_aug_model(A,B,C,Bd2,Cd2);
[Aaug3, Baug3, Caug3] = construct_aug_model(A,B,C,Bd3,Cd3);

% System 1 and 3 are full rank. 
% System 2 is rank deficient...



%%
% (b) Design a Kalman filter for each detectable augmented model
clc

% Kalman filter controlller 1
Q = eye(5);
R = eye(2);
% Solve discrete algebraic riccati equation
[~,L1,~] = idare(Aaug1',Caug1',Q,R)

% Kalman filter controlller 3
Q = eye(6);
R = eye(2);
% Solve discrete algebraic riccati equation
[~,L3,~] = idare(Aaug3',Caug3',Q,R)

%%
% (c) Find steady state target and matrix Mss
clc

H = [0 1];
Mss1 = construct_Mss(A,B,C,Bd1,Cd1,H)
Mss3 = construct_Mss(A,B,C,Bd3,Cd3,H)


%%
% (d) simulate system
clc
close all


% Define simulation parameters
tf = 1000;                      % tf =  time
p = size(C2,1);                 % outputs
m = size(B,2);                  % inputs
n = size(A,1);                  % states

N = 50;                         % prediction horizon
M = 40;                         % control horizon

Q = diag([5, 2, 0.5, 0.1]);     % state penalty
Pf = Q;                         % terminal state penalty
R = 0.1*eye(m);                 % control cost

% -------------------------------------------------------------
% Choose distrubance model to simulate
model = 3;          % <-- change model here
% -------------------------------------------------------------


if model == 1
    % Disturbance model 1
    % These variables will be used in the state estimate
    % important that the observer is based on the augmented system!!!
    nd = 1;
    Mss = Mss1;
    L_est = L1' 
    C_est = Caug1;
    A_est = Aaug1;
    B_est = Baug1;

elseif model == 3
    % Disturbance model 3
    % These variables will be used in the state estimate
    % important that the observer is based on the augmented system!!!
    nd = 2;
    Mss = Mss3;
    L_est = L3'; 
    C_est = Caug3;
    A_est = Aaug3;
    B_est = Baug3;
else
    error('model %.0f not defined, please choose 1 or 3',model);
end
    

% Setup for disturbance and initial condition
x = zeros(n,tf);
x0 = [pi/36 0 0 0]';                        
x(:,1)  = x0;
x_est_hat = zeros(n+nd,tf);
x_est_hat(:,1) = [zeros(n,1) ; zeros(nd,1)];

% initialize disturbance at k = dinit
dinit = 50;         
p = 0.2*[zeros(1,dinit), ones(1,tf-dinit)];

% Setpoint (to origin for all states)
ysp = zeros(4,tf); 

% Simulation
    for k = 1:tf
        
        % Calculate steady state target
        % Disturbance = error in kalman estimation
        d_hat = x_est_hat(end-nd+1:end, k);     % \hat{d} = \hat{x}
        xs_us = Mss*d_hat;                      % [xs;us] = Mss*\hat{d}
        xs = xs_us(1:n);                        % xs = [xs]
        us = xs_us(n+1:end);                    % us = [us]
        

        % x_hat for time instant k (states, not disturbances)
        x_hat = x_est_hat(1:n, k);

        % Calculate state deviation.
        % Our RHC will work on deviation variables. Similar to figure 10 in
        % L.N
        deviation_x = x_hat - xs;

        % Solve the quadratic programing problem based on disturbance model
        % to get the optimal control
        u0 = simulate_RHC(A,B,Q,R,Pf,N,M,deviation_x);
        u(:,k) = u0 + us;
 

        % Update the observer state
        % Measure outputs (actual system)
        y(:,k) = C*x(:,k);

        % Correct the state estimation (based on eq 2a 2b)
        % x^ = x^ +L(y - Cx^^)
        x_est_hat_priori = x_est_hat(:,k) + L_est*(y(:,k) - C_est*x_est_hat(:,k));
       
        % Update state estimate (based on eq 2a 2b)
        % OBS use augmented system
        x_est_hat(:,k+1) = A_est*x_est_hat_priori + B_est*u(:,k);
        
    
        % Update the process state (based on eq 1a 1b)
        % OBS Use original system with disturbance unknown to the designer
        x(:,k+1) = A*x(:,k) + B*u(:,k) + Bp*p(k);

        % start over

    end


% Plot the simulated system
plot_disturbed_system;
