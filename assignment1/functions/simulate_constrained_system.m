function [x_vector, u_vector] = simulate_constrained_system(A,B,C,Q,R,Pf,N,const)
    % SIMULATE_CONSTRAINED_SYSTEM
    % By: Nicholas Granlund
    
    
    % Assemble constraints as: Fx + Gu <= h 
    x1_max = const(1);
    x2_max = const(2);
    x3_max = const(3);
    x4_max = const(4);
    u_max = const(5);
    
    % Initial condition
    x0 = [pi/38 0 0 0]';
    n  = length(A); % 4
    m  = size(B,2); % 1
    
    % Construct F, G and h
    F      = kron([eye(N); zeros(N)], [0 1 0 0;0 -1 0 0]);
    G      = kron([zeros(N); eye(N)], [1; -1]);
    h      = [x2_max*ones(2*N,1); u_max*ones(2*N,1)];
    
    % Simulation parameters
    tf        = 210;   
    
    % States and inputs
    x_vector  = [x0 zeros(n,tf)]; 
    u_vector  = zeros(m,tf);     
    
    % Set initial state (x=x0)
    x = x0;              
    
    % Simulate Constrained receeding horizon control
    for iter = 1:tf
        
        % Simulate system
        [Z,~] = ConstrainedRHC(A,B,N,Q,R,Pf,F,G,h,x,n);
    
        % Get next input u and state x
        x     = Z(1:n);
        u     = Z(n*N+1);
    
        % Save state and input trajectory
        x_vector(:,iter+1) = x;
        u_vector(:,iter)   = u;
    end
end

