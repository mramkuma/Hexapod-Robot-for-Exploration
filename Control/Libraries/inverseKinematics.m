function [q] = inverseKinematics(a, L, Hd, K, m, dq)
    
    Xd = axisAngle(Hd);
    Rd = [Hd(1 : 3, 1 : 3) zeros(3, 1); zeros(1, 3) 1];
    i = 1;
    q(:, i) = a;

    while i <= 1500
        DH = denavitHartenberg(q(:, i), L);
        H = forwardKinematicsDH(DH, m);
        R = [H(1 : 3, 1 : 3) zeros(3, 1); zeros(1, 3) 1];
        X = axisAngle(H);
        Xr = axisAngle(Rd * R');
        e = [Xd(1 : 3, :) - X(1 : 3, :); Xr(4 : 6, :)];

        if abs(e) <= 1e-3
            break
        else
            q(:, i + 1) = q(:, i) + (pinv(jacobianMatrix(DH, q(:, i))) * K * e * dq);
        end
        i = i + 1;
    end
end