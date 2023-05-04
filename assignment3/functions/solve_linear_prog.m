function z = solve_linear_prog(A,b)
    % SOLVE_LINEAR_PROG
    % By: Nicholas Granlund

    n = size(A,2);
    m =  size(A,1);

    % minimize epsilon
    f = [zeros(n,1); 1];  

    % Constuct constraint matrix F
    AA = [ A  -ones(m,1);
          -A, -ones(m,1)];

    % Constraint vector b
    bb = [b;-b];

    % Solve linear problem
    [z,fval,~,~,lambda] = linprog(f,AA,bb);

    mu = lambda.ineqlin;

    % return

end

