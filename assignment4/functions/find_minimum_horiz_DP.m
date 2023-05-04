function [P_N, N, K_N] = find_minimum_horiz_DP(A,B,Q,R,Pf)
% FIND_MINIMUM_HORIZ_DP
% By: Nicholas Granlund

% This while loop increases the horizon as long as the system is unstable.
% The loop breaks when a gain K has been found that ensures stable feedback.
% utilizing the dynamic programming approach

    % Set P(N) = Pf
    P = Pf;
    N = 0;
    while 1
        % Increment N
        N = N + 1;

        %Riccati equation
        P = Q + A'*P*A - A'*P*B*inv(R + B'*P*B)*B'*P*A;

        % Determine feedback gain
        K_N = -inv(R+B'*P*B)*B'*P*A;

        % Check stability of closed loop
        poles = abs(eig(A+(B*K_N)));
        unstbl = 0;
        for i=1:length(poles)
        if poles(i) >= 1
            unstbl = unstbl + 1;
        end
        end
    
        % If stability has been obtained, break while
        if unstbl == 0
            poles = eig(A+(B*K_N));
            if N==1
                N = N;
            else
                N = N+1;
            end
            P_N = P;
            fprintf('\n===============DYNAMIC PROGRAMMING===============\n')
            fprintf('The minimum N for stable feedback is N=%.0f\n',N)
            fprintf('The poles are;\n')
            for i=1:length(poles)
                fprintf('POLE %.f:     ', i)
                fprintf('%f%+fj\n', real(poles(i)), imag(poles(i)))
            end
            fprintf('\nThe feedback gain K is given by; \nK = [')
            fprintf('%g ', K_N); fprintf(']\n');
            fprintf('=================================================\n\n\n')
            break
        end

        % Break if no solution is found
        if N > 1000
            break
        end
    end
end

