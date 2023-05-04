function [x,u,lambda] = solve_quad_prog(A,b)
    % SOLVE_QUAD_PROG
    % By: Nicholas Granlund

    % Number of variables
    n = 4;

    % initial condition
    x0 = 1.5;

    % Construct H
    H = eye(n);
    f = zeros(n,1)';

    % Aeq and beq
    Aeq = [1  0  -b  0;
           -A 1  0  -b];
    beq = [A*x0; 0];

    % Aineq and bineq
    Aineq = blkdiag([-1;1],[-1;1],[-1;1],[-1;1]);
    bineq = [-2.5 5 0.5 0.5 2 2 2 2]';

    % Remove lower constraint on x1
    %bineq = [10 5 0.5 0.5 2 2 2 2]';

    % Remove upper constraint on x1
   % bineq = [-2.5 10 0.5 0.5 2 2 2 2]';

    % Solve
    [sol,fval,~,~,lambda] = quadprog(H,f,Aineq,bineq,Aeq,beq);

    x = sol(1:2);
    u = sol(3:4);

    % Check if it satisfies KKT
    % Last KKT condition
    Aineq'*lambda.ineqlin;
    Aeq'*lambda.eqlin;

    % return

end
