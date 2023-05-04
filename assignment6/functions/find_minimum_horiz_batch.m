function [N, K_b] = find_minimum_horiz_batch(A,B,Q,R,Pf)
% FIND_MINIMUM_HORIZ_BATCH
% By: Nicholas Granlund

% This while loop increases the horizon as long as the system is unstable.
% The loop breaks when a gain K has been found that ensures stable feedback.
% utilizing the batch approach

    N = 0;
    while 1
        % Increment N
        N = N+1;
    
        % Construct Q_bar
        Q_bar = [];
        for i=1:N-1
            Q_bar = blkdiag(Q_bar,Q);
        end
        Q_bar = blkdiag(Q_bar,Pf);
    
        
        % Construct R_bar
        R_bar = [];
        for i=1:N
            R_bar = blkdiag(R_bar,R);
        end
        
        
        % Construct omega
        omega = [];
        for i=1:N
            omega = [omega; A^i];
        end
    
 
        % Construct gamma
        gamma = [];
        nolla = zeros(length(B),1);
        lowest_row = [];
        
        for i=1:N
            lowest_row = [lowest_row A^(N-i)*B];
        end
        
        gamma = [lowest_row];
        for i=1:N-1
            above_row = [lowest_row(:,i+1:end)];
            for j=1:i
                above_row = [above_row, nolla];
            end
            gamma = [above_row; gamma];
        end
        
    
        % Determine feedback gain
        K_b = -inv(gamma'*Q_bar*gamma + R_bar)*gamma'*Q_bar*omega;
        
        % K_B = K_b(0)
        K_b = K_b(1,:);
        
        % Check stability of closed loop
        poles = abs(eig(A+(B*K_b)));
        unstbl = 0;
        for i=1:length(poles)
            if poles(i) >= 1
                unstbl = unstbl + 1;
            end
        end

        % If stability has been obtained, break while
        if unstbl == 0
            poles = eig(A+(B*K_b));
            fprintf('\n======================BATCH======================\n')
            fprintf('The minimum N for stable feedback is N=%.0f\n',N)
            fprintf('The poles are;\n')
            for i=1:length(poles)
                fprintf('POLE %.f:     ', i)
                fprintf('%f%+fj\n', real(poles(i)), imag(poles(i)))
            end
            fprintf('\nThe feedback gain K is given by; \nK = [')
            fprintf('%g ', K_b); fprintf(']\n');
            fprintf('=================================================\n\n\n')
            break
        end
    
        % Break if no solution is found
        if N > 1000
            break
        end
    end
end


