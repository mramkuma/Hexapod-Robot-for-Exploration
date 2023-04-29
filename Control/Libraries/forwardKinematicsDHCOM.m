function [Hcom] = forwardKinematicsDHCOM(DH, comDH, rb)

%% Find the row where rigid body is in the Denavit - Hartenberg matrix

    % Set the symbolic expression of the requested rigid body (rb)
    Lcom = sym(append('Lcom', num2str(rb)));
    
    % Find the reference frame related to requested rigid body (rb)
    [frame, ~] = find(comDH == Lcom);

%% Forward kinematics through each joint

    % Calculate forward kinematics to previous reference frame
    H = forwardKinematicsDH(DH, frame - 1);

%% Forward kinematics to requested center of mass

    % Calculate forward kinematics to rigid body's center of mass
    Hcom = simplify(H * (Rz(comDH(frame, 1)) * Tz(comDH(frame, 2)) * Tx(comDH(frame, 3))), 'Steps', 3);

end