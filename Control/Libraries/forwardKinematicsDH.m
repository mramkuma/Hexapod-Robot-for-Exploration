function [H] = forwardKinematicsDH(DH, m)
    H = eye(4);
    for i = 1 : m
        H = H * (Rz(DH(i, 1)) * Tz(DH(i, 2)) * Tx(DH(i, 3)) * Rx(DH(i, 4)));
    end
end