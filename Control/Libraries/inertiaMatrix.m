function [D] = inertiaMatrix(m, DH, comDH, q, steps)
    
%% Rigid bodies' data

    % Get the number of rigid bodies
    [~, rb] = size(m);
    
%% Joints data    
    % Get the number of joints
    [n, ~] = size(q);

%% Inertia Matrix construction
    % Initialize inertia matrix with zeros
    D = sym(zeros(n, n));

    % Iterate throug all the rigid bodies
    for j = 1 : rb
        
        % Calculate forward kinematics to current center of mass
        [Hcom] = forwardKinematicsDHCOM(DH, comDH, j);

        % Calculate jacobian matrix of current rigid body
        [Jcom] = jacobianMatrixCOM(DH, comDH, q, j);
        
        % Create inertia tensor for current rigid body (SYMMETRIC MATRIX)
        I = [+sym(append('Ixx', num2str(j))) -sym(append('Ixy', num2str(j))) -sym(append('Ixz', num2str(j)))
             -sym(append('Ixy', num2str(j))) +sym(append('Iyy', num2str(j))) -sym(append('Iyz', num2str(j)))
             -sym(append('Ixz', num2str(j))) -sym(append('Iyz', num2str(j))) +sym(append('Izz', num2str(j)))];
        
        % Change reference frame of inertia tensor
        I = transpose(Hcom(1 : 3, 1 : 3)) * I * Hcom(1 : 3, 1 : 3);

        % Calculate current inertia matrix
        D = D + (m(j) * transpose(Jcom(1 : 3, :)) * Jcom(1 : 3, :)) + (transpose(Jcom(4 : 6, :)) * I * Jcom(4 : 6, :));

    end
    
    % Reduce fractions to floating point numbers (5 digits)
    D = vpa(simplify(D, 'Steps', steps), 5);

end