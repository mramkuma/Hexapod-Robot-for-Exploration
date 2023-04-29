function [DH] = denavitHartenberg(q, L)

%{
INSTRUCCIONES (QUE CHINGUEN A SU MADRE LOS GRINGOS):

q : vector («n» filas por una columna, OBLIGATORIO)
L : vector («n» filas o n columnas)

%}

%     Rz Tz Tx Rx
DH = [0 0 0 0
      q(1) L(1) 0.00 (pi / 2)
      q(2) 0.00 L(2) 0.000000
      q(3) 0.00 L(3) -(pi / 2)];

%{
    %% Three degrees of freedom robot
    DH = [0 0 0 0
          a(1) L(1) 0.00 pi/2
          a(2) 0.00 L(2) pi/2
          a(3) L(3) 0.00 0.];

%% Simple planar robot
    DH = [0 0 0 0
          a(1) 0 L(1) 0
          a(2) 0 L(2) 0];


%% Four DoF robot
    DH = [0 0 0 0
          a(1) L(1) 0 pi/2
          a(2) 0 L(2) 0
          a(3) 0 0 pi/2
          a(4) L(3) 0 0];

%% ROTRADI Robot
    DH = [0 0 0 0
          0.00 B(1) 0.00 0.00
          a(1) L(1) 0.00 pi/2
          a(2) B(2) L(2) 0.00
          a(3) B(3) L(3) 0.00];
%}

end