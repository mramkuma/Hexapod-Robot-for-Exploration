function [comDH] = denavitHartenbergCOM(q, Lcom)

%{
INSTRUCCIONES (QUE CHINGUEN A SU MADRE LOS GRINGOS):

q    : vector («n» filas por una columna, OBLIGATORIO)
Lcom : vector («n» filas o n columnas)

%}

%     Rz Tz Tx Rx
comDH = [0 0 0 0
         q(1) Lcom(1) 0.00 (pi / 2)
         q(2) 0.00 Lcom(2) 0.000000
         q(3) 0.00 Lcom(3) -(pi / 2)];

%{
%% Three degrees of freedom robot
    comDH = [0 0 0 0
             a(1) Lcom(1) 0.00000 pi/2
             a(2) 0.00000 Lcom(2) pi/2
             a(3) Lcom(3) 0.00000 0.00];

%% Simple planar robot
    comDH = [0 0 0 0
             a(1) 0 Lcom(1) 0
             a(2) 0 Lcom(2) 0];

%% Four DoF robot
    comDH = [0 0 0 0
             a(1) Lcom(1) 0 pi/2
             a(2) 0 Lcom(2) 0
             a(3) 0 0 pi/2
             a(4) Lcom(3) 0 0];

%% ROTRADI Robot
    comDH = [0 0 0 0
             0.00 B(1) 0.00 0.00
             a(1) Lcom(1) 0.00 pi/2
             a(2) B(2) Lcom(2) 0.00
             a(3) B(3) Lcom(3) 0.00];
%}

end