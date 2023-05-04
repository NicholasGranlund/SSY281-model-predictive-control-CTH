function [x, u] = simulate_system_DP(A,B,C,Q,R,Pf,N)
% SIMULATE_SYSTEM_DP
% By: Nicholas Granlund

% initial condition
x = [pi/38 0 0 0]';

% iterate through k timesteps
for k = 1:210

    % Define P
    P = Pf;

    % Iterate backwards from N to the initial time
    for i = 1:N-1
        P = Q + A'*P*A - A'*P*B*inv(R + B'*P*B)*B'*P*A;
    end
    
    % Solve for feedback-gain K
    K_N = -inv(R+B'*P*B)*B'*P*A;



    % Determine feedback
    u(:,k) = K_N*x(:,k);

    % update system
    x(:,k+1)=A*x(:,k) + B*u(:,k);
    %y(:,k) = C*x(:,k);

end

end

