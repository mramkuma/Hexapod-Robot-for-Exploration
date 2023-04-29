clc; clear all

%% Add the sub folders of MATLAB

addpath("Libraries")
addpath("Data");
addpath("Data\Rx 0 Ry 0 Variables\")
addpath("Data\Rx 23.5 Ry 23.5 Variables\")
addpath("Data\Rx 35 Ry 0 Variables\")

%% simbolics values  (OPTIONALES)

% Joints
q = sym('q', [3, 1]);

% Joint velocities
dq = sym('dq', [3, 1]);

% leght links 
L = sym('L', [3, 1]);
Lcom = sym('Lcom', [3, 1]);

% Body mass 
m = sym('m', [1, 3]);

%  gravitational Acceleration
g = [0; 0; -9.80665];

%%  Denavit - Hartenberg (OPTIONAL)

DH = denavitHartenberg(q, L);
comDH = denavitHartenbergCOM(q, Lcom);

%% Forward kinematics (OPTIONAL)

H = forwardKinematicsDH(DH, 3 + 1);

Hcom = forwardKinematicsDHCOM(DH, comDH, 2);

%% differential kinematics  (OPTIONAL)

% Jacobian Matrix
J = jacobianMatrix(DH, q);

% Jacobian matrix of rigid body 2
Jcom = jacobianMatrixCOM(DH, comDH, q, 2);

%% Dynamicla model (OPTIONAL)

D = inertiaMatrix(m, DH, comDH, q, 3);

C = centrifugalCoriolis(D, q, dq, 1);

G = gravitational(m, g, DH, comDH, q, 3);

%% change matrices in MATLAB (OPTIONAL)

matlabFunction(D,'file','functionD');
matlabFunction(C,'file','functionC');
matlabFunction(G,'file','functionG');

%% Numérical Numbers

q_0 = randn(3, 1);      % Joint initial position

dq_0 = zeros(3, 1);     % initial velocity in joints

m_n = [29.1735 40.0393 22.3124];        % joints Mass 
L_n = [0 0.64 0.96];                    % lenght of links
Lcom_n = L_n / 2;                       % Centerof mass

% I1_n = randn(3, 3);          % Inertial Tensor 1 (center of mass)
% I2_n = randn(3, 3);          % Inertial Tensor 2 (center of mass)
% I3_n = randn(3, 3);          % Inertial Tensor 3 (center of mass)
I1_n = eye(3)*0.4561;          % Inertial Tensor 4 (center of mass)
I2_n = eye(3)*0.3876;          % Inertial Tensor 5 (center of mass)
I3_n = eye(3)*0.2209;          % Inertial Tensor 6 (center of mass)



I1_n = 0.5 * (I1_n + I1_n');   % To make simetric
I2_n = 0.5 * (I2_n + I2_n');   % To make simetric
I3_n = 0.5 * (I3_n + I3_n');   % To make simetric

%% Control

% load data in the workspace 
load("Data\Rx 35 Ry 0 Variables\Joint_Angles.mat")

% Store the position in a vector
[~, ~, k] = size(Joint_Angles);
t = 500;                                                % simulatión time 
t_g = 10;                                               % Ttime to plot sec.
t_d = seconds(linspace(0, t, k));                       %  initial time, final time, numberof samples
leg = 4;                                                %leg to analize
q_s = reshape(Joint_Angles(:, leg, :), 3, []);          % (joint, leg, time)
q1_s = array2timetable(transpose(q_s(1, :)), ...
                        'RowTimes', t_d, ...
                        'VariableNames', {'q1_d'});     % Conversion to Timetable
q2_s = array2timetable(transpose(q_s(2, :)), ...
                        'RowTimes', t_d, ...
                        'VariableNames', {'q2_d'});     % Conversion to Timetable
q3_s = array2timetable(transpose(q_s(3, :)), ...
                        'RowTimes', t_d, ...
                        'VariableNames', {'q3_d'});     % Conversion to Timetable

% Store the velocities of the joints in a vector
dq_s = diff(q_s, 1, 2);                                 % (First order derivative)
dq_s(:, end + 1) = dq_s(:, end);
dq1_s = array2timetable(transpose(dq_s(1, :)), ...
                        'RowTimes', t_d, ...
                        'VariableNames', {'dq1_d'});    % Conversion to Timetable
dq2_s = array2timetable(transpose(dq_s(2, :)), ...
                        'RowTimes', t_d, ...
                        'VariableNames', {'dq2_d'});    % Conversion to Timetable
dq3_s = array2timetable(transpose(dq_s(3, :)), ...
                        'RowTimes', t_d, ...
                        'VariableNames', {'dq3_d'});    % Conversion to Timetable
% Store the acceleration of the joints in a vector
ddq_s = diff(dq_s, 1, 2);                                 % (First order derivative )
ddq_s(:, end + 1) = ddq_s(:, end);
ddq1_s = array2timetable(transpose(ddq_s(1, :)), ...
                        'RowTimes', t_d, ...
                        'VariableNames', {'ddq1_d'});    % Conversion to Timetable
ddq2_s = array2timetable(transpose(ddq_s(2, :)), ...
                        'RowTimes', t_d, ...
                        'VariableNames', {'ddq2_d'});    % Conversion to Timetable
ddq3_s = array2timetable(transpose(ddq_s(3, :)), ...
                        'RowTimes', t_d, ...
                        'VariableNames', {'ddq3_d'});    % Conversion to Timetable

% Gain Matrices PD
Kp = 10 * eye(3);
Kv = 10 * eye(3);

%% Execute Simulink

sim("Libraries\DynamicModel.slx")

%% Forward kinematics

% position number of joint
[k , ~] = size(q_d.data);

% Position and desired velocity in zero
X_d = zeros(6, k);
dX_d = zeros(6, k);

% position for joints 
for j = 1 : k

    %  Denavit - Hartenberg: 
    DH = denavitHartenberg(q_d.data(j, :)', L_n);

    % Forward Kinematic
    H = forwardKinematicsDH(DH, 4);

    % Change vector Axis - Angle
    X_d(:, j) = axisAngle(H);

    % Jacobian Matrix
    J_d = jacobianMatrix(DH, q_d.data(j, :)');

    % Velocity End effector
    dX_d(:, j) = J_d * dq_d.data(j, :)';

end

k_g = round(k * (t_g / t));

%% PLots

% Position in x
figure()
subplot(3,1,1)
hold on;
plot(X.time(1 : k_g), reshape(X.data(1, :, 1 : k_g), 1, []), 'r')
plot(X.time(1 : k_g), X_d(1, 1 : k_g), 'b--')
xlabel("Time (s) $\left[ sec \right]$", 'Interpreter', 'latex', 'FontSize', 12)
ylabel("Position [m]", 'Interpreter', 'latex', 'FontSize', 12)
legend(["$x Pos$", "$x_d Des P$"], 'Interpreter', 'latex', 'FontSize', 14)
grid on;
% hold off;

% Posición en y
% figure()
subplot(3,1,2)
hold on;
plot(X.time(1 : k_g), reshape(X.data(2, :, 1 : k_g), 1, []), 'r')
plot(X.time(1 : k_g), X_d(2, 1 : k_g), 'b--')
xlabel("Time (s) $\left[ sec \right]$", 'Interpreter', 'latex', 'FontSize', 12)
ylabel("Position [m]", 'Interpreter', 'latex', 'FontSize', 12)
legend(["$y Pos$", "$y_d Des P$"], 'Interpreter', 'latex', 'FontSize', 14)
grid on;
% hold off;

% Posición en z
% figure()
subplot(3,1,3)
hold on;
plot(X.time(1 : k_g), reshape(X.data(3, :, 1 : k_g), 1, []), 'r')
plot(X.time(1 : k_g), X_d(3, 1 : k_g), 'b--')
xlabel("Time (s)$\left[ sec \right]$", 'Interpreter', 'latex', 'FontSize', 12)
ylabel("Position [m]", 'Interpreter', 'latex', 'FontSize', 12)
legend(["$z Pos$", "$z_d Des P$"], 'Interpreter', 'latex', 'FontSize', 14)
grid on;
hold off;

% Rotation in X
figure()
subplot(3,1,1)
hold on;
plot(X.time(1 : k_g), reshape(X.data(4, :, 1 : k_g), 1, []), 'r')
plot(X.time(1 : k_g), X_d(4, 1 : k_g), 'b--')
xlabel("Time (s) $\left[ sec \right]$", 'Interpreter', 'latex', 'FontSize', 12)
ylabel("Angle (radians)", 'Interpreter', 'latex', 'FontSize', 12)
legend(["$n_x Rot X$", "$n_{x_d} Des.R$"], 'Interpreter', 'latex', 'FontSize', 14)
grid on;
% hold off;

% Rotation in y
% figure()
subplot(3,1,2)
hold on;
plot(X.time(1 : k_g), reshape(X.data(5, :, 1 : k_g), 1, []), 'r')
plot(X.time(1 : k_g), X_d(5, 1 : k_g), 'b--')
xlabel("Time (s) $\left[ sec \right]$", 'Interpreter', 'latex', 'FontSize', 12)
ylabel("Angle (radians)", 'Interpreter', 'latex', 'FontSize', 12)
legend(["$n_y Rot Y$", "$n_{y_d}Des.R Y$"], 'Interpreter', 'latex', 'FontSize', 14)
grid on;
% hold off;

% Rotation in z
% figure()
subplot(3,1,3)
hold on;
plot(X.time(1 : k_g), reshape(X.data(6, :, 1 : k_g), 1, []), 'r')
plot(X.time(1 : k_g), X_d(6, 1 : k_g), 'b--')
xlabel("Time (s) $\left[ sec \right]$", 'Interpreter', 'latex', 'FontSize', 12)
ylabel("Angle (radians)", 'Interpreter', 'latex', 'FontSize', 12)
legend(["$n_z Rot Z$", "$n_{z_d}Des.R Z$"], 'Interpreter', 'latex', 'FontSize', 14)
grid on;
hold off;


% Linear velocity in X
hold on;
plot(dX.time(1 : k_g), reshape(dX.data(1, :, 1 : k_g), 1, []), 'r')
plot(dX.time(1 : k_g), dX_d(1, 1 : k_g), 'b--')
xlabel("Time $\left[ sec \right]$", 'Interpreter', 'latex', 'FontSize', 12)
ylabel("meters", 'Interpreter', 'latex', 'FontSize', 12)
legend(["$\dot{x}L Velo X$", "$\dot{x}_dL Des $"], 'Interpreter', 'latex', 'FontSize', 14)
grid on;
hold off;

% Linear velocity in Y
figure()
hold on;
plot(dX.time(1 : k_g), reshape(dX.data(2, :, 1 : k_g), 1, []), 'r')
plot(dX.time(1 : k_g), dX_d(2, 1 : k_g), 'b--')
xlabel("Time $\left[ sec \right]$", 'Interpreter', 'latex', 'FontSize', 12)
ylabel("meters", 'Interpreter', 'latex', 'FontSize', 12)
legend(["$\dot{y} L Velo Y$", "$\dot{y}_d$ L Des"], 'Interpreter', 'latex', 'FontSize', 14)
grid on;
hold off;

% Linear velocity in z
figure()
hold on;
plot(dX.time(1 : k_g), reshape(dX.data(3, :, 1 : k_g), 1, []), 'r')
plot(dX.time(1 : k_g), dX_d(3, 1 : k_g), 'b--')
xlabel("Time $\left[ sec \right]$", 'Interpreter', 'latex', 'FontSize', 12)
ylabel("meters", 'Interpreter', 'latex', 'FontSize', 12)
legend(["$\dot{z}$ L Velo Z", "$\dot{z}_d$ L Des"], 'Interpreter', 'latex', 'FontSize', 14)
grid on;
hold off;

% Angular Velocity in x
figure()
hold on;
plot(dX.time(1 : k_g), reshape(dX.data(4, :, 1 : k_g), 1, []), 'r')
plot(dX.time(1 : k_g), dX_d(4, 1 : k_g), 'b--')
xlabel("Time $\left[ sec \right]$", 'Interpreter', 'latex', 'FontSize', 12)
ylabel("Radians", 'Interpreter', 'latex', 'FontSize', 12)
legend(["$\omega_x Ang X$", "$\omega_{x_d}Des $"], 'Interpreter', 'latex', 'FontSize', 14)
grid on;
hold off;

% Angular Velocity in y
figure()
hold on;
plot(dX.time(1 : k_g), reshape(dX.data(5, :, 1 : k_g), 1, []), 'r')
plot(dX.time(1 : k_g), dX_d(5, 1 : k_g), 'b--')
xlabel("Time $\left[ sec \right]$", 'Interpreter', 'latex', 'FontSize', 12)
ylabel("Radians", 'Interpreter', 'latex', 'FontSize', 12)
legend(["$\omega_y Ang Y$", "$\omega_{y_d}Des $"], 'Interpreter', 'latex', 'FontSize', 14)
grid on;
hold off;

% Angular Velocity in  z
figure()
hold on;
plot(dX.time(1 : k_g), reshape(dX.data(6, :, 1 : k_g), 1, []), 'r')
plot(dX.time(1 : k_g), dX_d(6, 1 : k_g), 'b--')
xlabel("Time $\left[ sec \right]$", 'Interpreter', 'latex', 'FontSize', 12)
ylabel("Radians", 'Interpreter', 'latex', 'FontSize', 12)
legend(["$\omega_z Ang Z $", "$\omega_{z_d} Des $"], 'Interpreter', 'latex', 'FontSize', 14)
grid on;
hold off;

% Joint 1
figure()
subplot(3,1,1)
hold on;
plot(q.time(1 : k_g), q.data(1 : k_g, 1), 'r')
plot(q_d.time(1 : k_g), q_d.data(1 : k_g, 1), 'b--')
xlabel("Time $\left[ sec \right]$", 'Interpreter', 'latex', 'FontSize', 12)
ylabel("Position", 'Interpreter', 'latex', 'FontSize', 12)
legend(["$\theta_{1}Joint 1$", "$\theta_{1_d}Des$"], 'Interpreter', 'latex', 'FontSize', 14)
grid on;
%hold off;

% Joint 2
% figure()
subplot(3,1,2)
hold on;
plot(q.time(1 : k_g), q.data(1 : k_g, 2), 'r')
plot(q_d.time(1 : k_g), q_d.data(1 : k_g, 2), 'b--')
xlabel("Time (s)$\left[ sec \right]$", 'Interpreter', 'latex', 'FontSize', 12)
ylabel("Angle (Radians)", 'Interpreter', 'latex', 'FontSize', 12)
legend(["$\theta_{2}Joint 2 $", "$\theta_{2_d}Des$"], 'Interpreter', 'latex', 'FontSize', 14)
grid on;
% hold off;

% Joint 3
% figure()
subplot(3,1,3)
hold on;
plot(q.time(1 : k_g), q.data(1 : k_g, 3), 'r')
plot(q_d.time(1 : k_g), q_d.data(1 : k_g, 3), 'b--')
xlabel("Time $\left[ sec \right]$", 'Interpreter', 'latex', 'FontSize', 12)
ylabel("Position", 'Interpreter', 'latex', 'FontSize', 12)
legend(["$\theta_{3} Joint 3 $", "$\theta_{3_d}Des$"], 'Interpreter', 'latex', 'FontSize', 14)
grid on;
hold off;

% Joint 1 Velocity
figure()
subplot(3,1,1)
hold on;
plot(dq.time(1 : k_g), dq.data(1 : k_g, 1), 'r')
plot(dq_d.time(1 : k_g), dq_d.data(1 : k_g, 1), 'b--')
xlabel("Time $\left[ sec \right]$", 'Interpreter', 'latex', 'FontSize', 12)
ylabel("Radians/s", 'Interpreter', 'latex', 'FontSize', 12)
legend(["$\dot{\theta}_{1}J1 Velo$", "$\dot{\theta}_{1_d}Des $"], 'Interpreter', 'latex', 'FontSize', 14)
grid on;
% hold off;

% Joint 2 Velocity
% figure()
subplot(3,1,2)
hold on;
plot(dq.time(1 : k_g), dq.data(1 : k_g, 2), 'r')
plot(dq_d.time(1 : k_g), dq_d.data(1 : k_g, 2), 'b--')
xlabel("Time $\left[ sec \right]$", 'Interpreter', 'latex', 'FontSize', 12)
ylabel("Radians/s", 'Interpreter', 'latex', 'FontSize', 12)
legend(["$\dot{\theta}_{2}J2 Velo $", "$\dot{\theta}_{2_d}Des $"], 'Interpreter', 'latex', 'FontSize', 14)
grid on;
% hold off;

% Joint 3 Velocity
% figure()
subplot(3,1,3)
hold on;
plot(dq.time(1 : k_g), dq.data(1 : k_g, 3), 'r')
plot(dq_d.time(1 : k_g), dq_d.data(1 : k_g, 3), 'b--')
xlabel("Time $\left[ sec \right]$", 'Interpreter', 'latex', 'FontSize', 12)
ylabel("Radians/s", 'Interpreter', 'latex', 'FontSize', 12)
legend(["$\dot{\theta}_{3}J3 Velo $", "$\dot{\theta}_{3_d}Des $"], 'Interpreter', 'latex', 'FontSize', 14)
grid on;
hold off;

% Control Torque (tau)
figure()
hold on;
plot(tau.time(1 : k_g), reshape(tau.data(1, :, 1 : k_g), 1, []), 'LineWidth', 1)
plot(tau.time(1 : k_g), reshape(tau.data(2, :, 1 : k_g), 1, []), 'LineWidth', 1)
plot(tau.time(1 : k_g), reshape(tau.data(3, :, 1 : k_g), 1, []), 'LineWidth', 1)
title("Torque $\tau \left( t \right)$", 'Interpreter', 'latex', 'FontSize', 12)
xlabel("Time $\left[ sec \right]$", 'Interpreter', 'latex', 'FontSize', 12)
ylabel("Newton/m", 'Interpreter', 'latex', 'FontSize', 12)
legend(["$\tau_1$", "$\tau_2$", "$\tau_3$"], 'Interpreter', 'latex', 'FontSize', 14)
grid on;
hold off;

%% Error plots

% Error joints
figure()
plot(Error, 'LineWidth', 1)
title("Error $e_{\theta} \left( t \right)$", 'Interpreter', 'latex', 'FontSize', 12)
xlabel("Time $\left[ sec \right]$", 'Interpreter', 'latex', 'FontSize', 12)
ylabel("Position[rad]", 'Interpreter', 'latex', 'FontSize', 12)
legend(["$\theta_1 Error J1$", "$\theta_2 Error J2 $", "$\theta_3 Error J3$"], 'Interpreter', 'latex', 'FontSize', 14)
grid on;

%  Error joints velocities
figure()
plot(dError, 'LineWidth', 1)
title("Error $e_{\omega} \left( t \right)$", 'Interpreter', 'latex', 'FontSize', 12)
xlabel("Time $\left[ sec \right]$", 'Interpreter', 'latex', 'FontSize', 12)
ylabel("Position[rad]", 'Interpreter', 'latex', 'FontSize', 12)
legend(["$\omega_1 E Ang Vel 1 $", "$\omega_2  E Ang Vel 2 $", "$\omega_3  E Ang Vel 3 $"], 'Interpreter', 'latex', 'FontSize', 14)
grid on;