function [Mss] = construct_Mss(A,B,C,Bd,Cd,H)
    % CONSTRUCT_MSS 
    % By: Nicholas Granlund
    
    % Create the Mss matrix and return
    Mss = [eye(length(A))-A, -B;
              H*C     zeros(size(H*C,1),size(B,2))]\[Bd; -H*Cd];
end

