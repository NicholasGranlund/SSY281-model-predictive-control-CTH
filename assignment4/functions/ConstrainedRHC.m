function [Z,exitflag]=ConstrainedRHC(A,B,N,Q,R,Pf,x0,n)
    % CONSTRAINEDRHC 
    % By: Nicholas Granlund

    % Ths function has influence from CRHC.m file used during
    % PSS which was created by Rémi Lacombe.


    % Construct the objective function
    Qbar = blkdiag(kron(eye(N-1),Q),Pf);
    Rbar = kron(eye(N),R);
    H    = blkdiag(Qbar,Rbar);
    f    = []';
    
    % Construct the equality constraints
    I    = eye(n);
    Aeq1 = kron(eye(N),I)+kron(diag(ones(N-1,1),-1),-A);
    Aeq2 = kron(eye(N),-B);
    Aeq  = [Aeq1 Aeq2]
    beq  = [A*x0;zeros(n*(N-1),1)]

    % Inequality constraint
    F      = [kron(eye(N*2),[-1;1]); zeros(2*N)];
    G      = [zeros(2*N,N); kron([zeros(N); eye(N)], [-1; 1])];
    h      = [3*ones(4*N,1); 0.1*ones(2*N,1)];

    Ain  = [F G]
    bin  = h
    
    % Solve the optimization problem with quadratic programming
    options = optimoptions('quadprog','Display','none');
    [Z,VN,exitflag,~,~] = quadprog(2*H,f,Ain,bin,Aeq,beq,[],[],[],options);
    
    % Check if the problem has been solved optimally
%     if exitflag ~= 1
%         disp(['The exitflag is ' num2str(exitflag) ', which means that something is wrong.'])
%     end

end