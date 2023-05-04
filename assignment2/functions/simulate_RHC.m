function u0 = simulate_RHC(A,B,Q,R,Pf,N,M,deviation_x)
    % SIMULATE_RHC
    % By: Nicholas Granlund


    % Objective:    min  1/2*x'*H*x + f'*x   
    % where         x=[x(1);x(2);...;x(N);u(0);...;u(M-1)];
    % subject to    Aeq * x = beq   

    
    n=length(deviation_x);           % n = number of states
    m=length(B(1,:));                % m = number of inputs

    % ------------------------------------------------------------------
    % Construct H matrix

    % This is the state estimation error covariance from k=1 to k=N-1
    H_pred_horiz = kron(eye(N-1),Q);                 % x(1) --> x(N-1)

    % This is the final state estimation error covariance at k=N
    H_pred_final = Pf;                                    %  x(N)
    
    % This is the second part of H, governing the control horizon M for u.
    H_contr_horiz = kron(eye(M),R);                       % u(1) --> u(M)

    % Merge these blocks together to create the entire H matrix
    H = blkdiag(H_pred_horiz, H_pred_final, H_contr_horiz);
    

    % ------------------------------------------------------------------
    % Construct f vector (no linear objective --> empty)
    
    f = [];
     
    % ------------------------------------------------------------------
    % Construct equality constraint (s.t Aeq * x = beq)

    % Aeq
    Aeq_states = [zeros(n,n*N);kron(eye(N-1),A),zeros(n*(N-1),n)]-eye(n*N);
    Aeq_control = kron(eye(M),B);  
    Aeq_ub= repmat(Aeq_control((M-1)*n+1:M*n,:),N-M,1);
    Aeq_control = [Aeq_control;Aeq_ub];
    
    Aeq   = [Aeq_states, Aeq_control];

    % beq 
    beq = [-A*deviation_x;zeros(n*(N-1),1)];

    
    % ------------------------------------------------------------------
    % Construct the inequality constraints  (s.t Ain * < = bin)
    % We have none thus empty
    Ain = [];
    Bin = [];


    % ------------------------------------------------------------------
    % Construct the lower and upper inequality constraints  (lb ≤ x ≤ ub)
    % We have none thus empty
    lb = [];
    ub = [];
    

    % ------------------------------------------------------------------
    % Solve for x and u in the min of the objective function
    sol = quadprog(2*H,f,Ain,Bin,Aeq,beq,lb,ub); 
    

    % Only commit to the first control from the optimal control sequence...
    % 4 states * 50 horizon. u0 should thus be at index 50*4+1=201;
    u0=sol(n*N+1:n*N+m);

    % return
    
end



