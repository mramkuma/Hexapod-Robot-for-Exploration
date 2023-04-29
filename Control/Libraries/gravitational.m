function [G] = gravitational(m, g, DH, comDH, q, steps)
    
%% Rigid bodies' data

    % Get the number of rigid bodies
    [~, rb] = size(m);
    
%% Joints data    
    % Get the number of joints
    [n, ~] = size(q);

%% Derivative of Potential Energy construction
    
    % Initialize with zeros
    G = sym(zeros(n, 1));

    % Iterate throug all the rigid bodies
    for j = 1 : rb

        % Calculate jacobian matrix of current rigid body
        [Jcom] = jacobianMatrixCOM(DH, comDH, q, j);

        % Calculate current derivative of potential energy
        G = G + (m(j) * transpose(Jcom(1 : 3, :)) * g);

    end

    % Reduce fractions to floating point numbers (5 digits)
    G = vpa(simplify(G, 'Steps', steps), 5);

end