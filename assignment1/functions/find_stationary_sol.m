function [P_inf, N, K_N] = find_stationary_sol(A,B,Q,R,Pf)
% This while loop increases the horizont as long as the system is unstable.
% The loop breaks when the stationary solution is found
    P = Pf;
    N = 0;
    while 1
        N = N+1;
        P_prev = P;
        P = Q + A'*P*A - A'*P*B*inv(R + B'*P*B)*B'*P*A;
        K_N = -inv(R+B'*P*B)*B'*P*A;

        % Determine if stationary solution has been obtained
        if norm(P - P_prev) <= 0.1
            P_inf = P;
            N = N+1;
            fprintf('The stationary solution is obtained when N=%.0f\n\n',N)
            break
        end

        % break if no solution is found
        if N > 1000
            break
        end
    end
end