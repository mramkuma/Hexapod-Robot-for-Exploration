function [Jcom] = jacobianMatrixCOM(DH, comDH, q, rb)
    
    % Find the reference frame related to requested rigid body (rb)
    [comFrame, ~] = find(comDH == sym(append('Lcom', num2str(rb))));
    
    % Calculate forward kinematics to requested center of mass
    Hcom = forwardKinematicsDHCOM(DH, comDH, rb);

    % Get number of generalized coordinates
    [n, ~] = size(q);
    
    % Initialize Jacobian Matrix with zeros
    Jcom = sym(zeros(6, n));

    % Iterate through all the generalized coordinates
    for i = 1 : n

        % Find the reference frame related to current joint
        [jointFrame, ~] = find(comDH == sym(append('q', num2str(i))));
        
        % If current joint is in a reference frame that doesn't affect the COM
        if jointFrame > comFrame

            % Stop construction
            break;

        end
        
        % Calculate forward kinematics to the previous frame of current joint
        H = forwardKinematicsDH(DH, jointFrame - 1);

        % Include joint actuation ONLY
        H = H * Rz(DH(jointFrame, 1));
        
        % Get actuation axis of current joint
        z = H(1 : 3, 3);

        % Calculate the distance between center of mass and current joint
        r = Hcom(1 : 3, 4) - H(1 : 3, 4);

        % Create linear term of jacobian matrix
        Jcom(1 : 3, i) = cross(z, r);

        % Create angular term of jacobian matrix
        Jcom(4 : 6, i) = z;

    end
end