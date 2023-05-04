function mu = solve_dual_prog(A,b)
    % SOLVE_DUAL_PROG
    % By: Nicholas Granlund

    n = size(A,2);  
    m =  size(A,1);

    % f
    f = [b; -b];  

    % Equality Aeq
    Aeq = [ A  -ones(m,1);
           -A, -ones(m,1)]';

    % Equality beq
    beq = -[0;0;1];

    % bounds
    lb = zeros(length(f),1);
    ub = [];

    % Solve linear problem
    [sol,fval,~,~,lambda] = linprog(f,[],[],Aeq,beq,lb,ub);

    % extract mu
    mu = sol;

    % return
end

