function [Aaug, Baug, Caug] = construct_aug_model(A,B,C,Bd,Cd)
    %CONSTRUCT_AUG_MODEL
    % By: Nicholas Granlund

    % construct A_augmented
    Aaug = [A Bd; zeros(size(Bd,2),size(A,2)), eye(size(Bd,2))];

    % construct B_augmented
    Baug = [B; zeros(size(Bd,2),size(B,2))];

    % construct C_augmented
    Caug = [C Cd];

    % determine rank of the original system
    rankoriginal = rank(obsv(A,C));

    if rankoriginal == length(A)
        fprintf('\nOriginal system is full rank:         rank(O(A,C)) = %.0f\n',rankoriginal)
    else
        fprintf('\nOrignal system is NOT full rank:        rank(O(A,C)) = %.0f\n',rankoriginal\n')
    end

    % determine rank of the system
    rankaug = rank([eye(length(A))-A  -Bd; C  Cd ]);

    if rankaug == length(Aaug)
        fprintf('Augmented system is full rank:        rank(aug) = %.0f\n',rankaug)
    else
        fprintf('Augmented system is NOT full rank:    rank(aug) = %.0f\n',rankaug)
    end
    
end

