close all
clear all
clc
format short

y1 = @(x) 0.5.*x.^2;
y2 = @(x) x.^3 + 2.*x.^2 - 2.*x;

x = linspace(-3,2);

figure()
subplot(1,2,1)
plot(x,y1(x),'k','linewidth',3)
axis off

subplot(1,2,2)
plot(x,y2(x),'k','linewidth',3)
axis off

%% Linear programming
clc
close all

% Matrices A and b
A = [0.4889   0.2939;
     1.0347  -0.7873;
     0.7269   0.8884;
     -0.3034  -1.1471];

b = [-1.0689;
     -0.8095;
     -2.9443;
      1.4384];

% Linear program
z = solve_linear_prog(A,b)

% Dual problem
mu = solve_dual_prog(A,b)

% Primal problem
% F = [A, -ones(4,1);
%     -A -ones(4,1)]
% g = [b;-b]
% c = [0;0;1];
% 
% f = (c' + mu'*F)
% 
% [sol,fval,~,~,lambda] = linprog(f,F,g,[],[])




%% Quadratic programming
clc
close all

A = 0.4;
b =1;

[x, u, lambda] = solve_quad_prog(A,b);
x
u
mu = lambda.ineqlin
lambda = lambda.eqlin








