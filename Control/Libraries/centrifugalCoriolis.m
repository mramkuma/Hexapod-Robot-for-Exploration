function [C] = centrifugalCoriolis(D, q, dq, steps)

%% Matrix construction
    
    % Get number of columns of inertia matrix
    [~, n] = size(D);
    
    % Initialize matrix
    C = sym(zeros(n));

    % Iterate through all the columns of inertia matrix
    for k = 1 : n

        % Calculate derivative with respect to generalized coordinates
        dD_da = jacobian(D(:, k), q);

        % Calculate matrix
        C = C + ((dD_da - (0.5 * transpose(dD_da))) * dq(k));
    end

    % Reduce fractions to floating point numbers (5 digits)
    C = vpa(simplify(C, 'Steps', steps), 5);

end