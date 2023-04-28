function []= RRRRobot_With_and_Without_GC()
clc;
clear;
close all;

q1=sym('q1');  q2=sym('q2');  q3=sym('q3');

l1=0.3;  l2=l1;  l3=l1;  m1=0.5; m2=m1;  m3=m1;  g=9.8;

Point_3 = [cos(q1)*(l3*cos(q2 + q3) + l2*cos(q2)), sin(q1)*(l3*cos(q2 + q3) + l2*cos(q2)), l1 + l3*sin(q2 + q3) + l2*sin(q2)];
prompt = {'Cartesian coordinates X [mm]:','Cartesian coordinates Y [mm]:','Cartesian coordinates Z [mm]:'};
dlgtitle = 'Desired Final End-Effector Position';
dims = [1 60];
definput = {'0','300','600'};
user_input = inputdlg(prompt,dlgtitle,dims,definput);

if size(user_input) == 0
    disp('User closed out of Cartesian position input. Using default values [0 300 600]')
    user_input = {'0','300','600'};
end

eqn = Point_3 == [str2double(user_input(1)) str2double(user_input(2)) str2double(user_input(3))]/1000;
[joint1,joint2,joint3] = solve(eqn,[q1, q2, q3],'PrincipalValue',true);
joint1 = double(joint1);
joint2 = double(joint2);
joint3 = double(joint3);
if joint3 > 0
   joint2 = joint3;
   joint3 = -joint3;
%    if joint1 > 2*pi()
%        joint1 = rem(joint1,2*pi());
%    end
%    if joint2 < 0
%        msg = 'Error occurred. The position you are entered to reach is unreachable by the robot. (Information: sqrt(X^2+Y^2) >= 100';
%        error(msg)
%    end
% elseif joint2 < 0 && joint2 > -0.1
%    msg = 'Error occurred. The position you are entered to reach is unreachable by the robot. (Information: sqrt(X^2+Y^2) >= 100';
%    error(msg)
end
% disp([joint1 joint2 joint3])

x0 = [0,0,1.5708,0,0,0]; % Initial Condition - Format:[q1,q2,q3,dq1,dq2,dq3]

tf=10;

%% Solve the closed-loop system nonlinear differential equation (PlanarArmODE) via ode45
%%ode45 solves the differential equation and returns X with respect to T.
global torque
global error
global position
global position_j2
global q_position
global print_figures
torque=[];
q_position=[];
print_figures = false; % True to save figures to current folder, False to not
print_figures = questdlg({'This function is capable of saving the generated plots automatically.';
    'Would you like to save the plots?';'Press "Yes" to save the plots.';'Press "No" to just display the final plots.'}, ...
	'Save Plots', ...
	'Yes','No','No');
if strcmpi(print_figures,'Yes')
    print_figures = true;
else
    print_figures = false;
end
without_GC = true; % True to run with Gravity Compensation, False without
without_GC = questdlg({'Would you run without Gravity Compensation on this PD controller?';'Press "Yes" to run without GC.';'Press "No" to run with GC.'}, ...
	'Without Gravity Compensation', ...
	'Yes','No','No');
if strcmpi(without_GC,'Yes')
    without_GC = true;
else
    without_GC = false;
end

disp('Starting Trajectory Calculation...')
[T_u,X_u] = ode45(@(t,x)ArmODE(t,x),[0 tf],x0);
disp('Starting Joint Position Calculations...')
position = RobotFK(q_position);
position_j2 = RobotFK_L2(q_position);
disp('Starting Error Calculation...')
error = [(str2double(user_input(1))/1000)-position(1,:); (str2double(user_input(2))/1000)-position(2,:); (str2double(user_input(3))/1000)-position(3,:)];

%% Plot Data
% With or without Gravity Compensation
disp('Generating Plots...')
if without_GC
    name1 = 'Joints under PD SetPoint Control Without Gravity Compensation';
else
    name1 = 'Joints under PD SetPoint Control With Gravity Compensation';
end
figure('Name',name1);
subplot(2,1,1)
plot(T_u, X_u(:,1),'-');
hold on
plot(T_u, X_u(:,2),'r--');
plot(T_u, X_u(:,3),'-.','Color', '#77AC30','LineWidth',1);
title('Position')
xlabel('Time [s]')
ylabel('Position [radians]')
legend('Joint 1','Joint 2','Joint 3','Location','east')
subplot(2,1,2)
plot(T_u, X_u(:,4),'-');
hold on
plot(T_u, X_u(:,5),'r--');
plot(T_u, X_u(:,6),'-.','Color', '#77AC30','LineWidth',1);
title('Velocity')
xlabel('Time [s]')
ylabel('Velocity [radians/s]')
legend('Joint 1','Joint 2','Joint 3','Location','southeast')
if without_GC
    name2 = 'Joints Without Gravity Compensation';
else
    name2 = 'Joints With Gravity Compensation';
end
sgtitle(name2)
if print_figures
    set(gcf,'Units','inches'); % Set the units of the figure
    screenposition = get(gcf,'Position'); % Measure the size of the figure
    set(gcf,'PaperPosition', [0 0 screenposition(3:4)],'PaperSize' , (screenposition(3:4))); % Configures the print paper size to fit the figure size
    print(name1,'-dpng','-r600') % Save figure as PNG
end

if without_GC
    name3 = 'Input_PD control Without Gravity Compensation';
else
    name3 = 'Input_PD control With Gravity Compensation';
end
figure('Name', name3);
plot(T_u, torque(1,1:size(T_u,1)),'-' );
hold on
plot(T_u, torque(2,1:size(T_u,1)),'r--');
plot(T_u, torque(3,1:size(T_u,1)),'-.','Color', '#77AC30','LineWidth',1);

if without_GC
    name4 = 'Joint Input Without Gravity Compensation';
else
    name4 = 'Joint Input With Gravity Compensation';
end
title(name4)
xlabel('Time [s]')
ylabel('Input')
legend('Joint 1 [torque]','Joint 2 [torque]','Joint 3 [torque]')
if print_figures
    set(gcf,'Units','inches'); % Set the units of the figure
    screenposition = get(gcf,'Position'); % Measure the size of the figure
    set(gcf,'PaperPosition', [0 0 screenposition(3:4)],'PaperSize' , (screenposition(3:4))); % Configures the print paper size to fit the figure size
    print(name3,'-dpng','-r600') % Save figure as PNG
end

if without_GC
    name5 = 'Input_PD control Without Gravity Compensation - Zoomed';
else
    name5 = 'Input_PD control With Gravity Compensation - Zoomed';
end
figure('Name',name5);
plot(T_u, torque(1,1:size(T_u,1)),'-' );
hold on
plot(T_u, torque(2,1:size(T_u,1)),'r--');
plot(T_u, torque(3,1:size(T_u,1)),'-.','Color', '#77AC30','LineWidth',1);
if without_GC
    name6 = 'Joint Input Without Gravity Compensation - Zoomed';
else
    name6 = 'Joint Input With Gravity Compensation - Zoomed';
end
title(name6)
xlabel('Time [s]')
xlim([0 1.5])
ylabel('Input')
legend('Joint 1 [torque]','Joint 2 [torque]','Joint 3 [torque]')
if print_figures
    set(gcf,'Units','inches'); % Set the units of the figure
    screenposition = get(gcf,'Position'); % Measure the size of the figure
    set(gcf,'PaperPosition', [0 0 screenposition(3:4)],'PaperSize' , (screenposition(3:4))); % Configures the print paper size to fit the figure size
    print(name5,'-dpng','-r600') % Save figure as PNG
end

if without_GC
    name7 = 'End-Effector Position Without Gravity Compensation';
else
    name7 = 'End-Effector Position With Gravity Compensation';
end
figure('Name',name7)
subplot(3,1,1)
plot(T_u,position(1,1:size(T_u,1)),'r-')
if without_GC
    name8 = 'End-Effector X Position Without Gravity Compensation';
else
    name8 = 'End-Effector X Position With Gravity Compensation';
end
title(name8)
xlabel('Time [seconds]')
ylabel('Position [m]')
yyaxis right
plot(T_u,error(1,1:size(T_u,1)),'--','Color','#D95319')
ylabel('Position Error [m]')
subplot(3,1,2)
plot(T_u,position(2,1:size(T_u,1)),'-','Color', '#77AC30')
if without_GC
    name9 = 'End-Effector Y Position Without Gravity Compensation';
else
    name9 = 'End-Effector Y Position With Gravity Compensation';
end
title(name9)
xlabel('Time [seconds]')
ylabel('Position [m]')
yyaxis right
plot(T_u,error(2,1:size(T_u,1)),'--','Color', '#D95319')
ylabel('Position Error [m]')
subplot(3,1,3)
plot(T_u,position(3,1:size(T_u,1)),'b-')
if without_GC
    name10 = 'End-Effector Z Position Without Gravity Compensation';
else
    name10 = 'End-Effector Z Position With Gravity Compensation';
end
title(name10)
xlabel('Time [seconds]')
ylabel('Position [m]')
yyaxis right
plot(T_u,error(3,1:size(T_u,1)),'--','Color','#D95319')
ylabel('Position Error [m]')
if print_figures
    set(gcf,'Units','inches'); % Set the units of the figure
    screenposition = get(gcf,'Position'); % Measure the size of the figure
    set(gcf,'PaperPosition', [0 0 screenposition(3:4)],'PaperSize' , (screenposition(3:4))); % Configures the print paper size to fit the figure size
    print(name7,'-dpng','-r600') % Save figure as PNG
end

no_animator(position_j2,position,T_u);

torque=[];
%% Defining Functions

    function dx = ArmODE(t,x)
        theta_d=[joint1;joint2;joint3]; % Desired Set-Point Position
        dtheta_d=[0;0;0]; % Desired velocity (Derivative of theta_d)
        ddtheta_d=[0;0;0]; % Desired acceleration (Derivative of dtheta_d)
        theta= x(1:3,1);
        dtheta= x(4:6,1);
        q_position =[q_position, theta];
        
        global Mmat Cmat Gmat
        Mmat = [(l2^2*m2)/2 + (l2^2*m3)/2 + (l3^2*m3)/2 + (l2^2*m2*cos(2*x(2)))/2 + (l2^2*m3*cos(2*x(2)))/2 + (l3^2*m3*cos(2*x(2) + 2*x(3)))/2 + l2*l3*m3*cos(x(3)) + l2*l3*m3*cos(2*x(2) + x(3)), 0, 0;
            0, l2^2*m2 + l2^2*m3 + l3^2*m3 + 2*l2*l3*m3*cos(x(3)), l3*m3*(l3 + l2*cos(x(3)));
            0, l3*m3*(l3 + l2*cos(x(3))), l3^2*m3];

        Cmat = [-x(4)*(x(5)*l2^2*m2*sin(2*x(2)) + x(5)*l2^2*m3*sin(2*x(2)) + x(5)*l3^2*m3*sin(2*x(2) + 2*x(3)) + x(6)*l3^2*m3*sin(2*x(2) + 2*x(3)) + 2*x(5)*l2*l3*m3*sin(2*x(2) + x(3)) + x(6)*l2*l3*m3*sin(2*x(2) + x(3)) + x(6)*l2*l3*m3*sin(x(3)));
            (x(4)^2*l2^2*m2*sin(2*x(2)))/2 + (x(4)^2*l2^2*m3*sin(2*x(2)))/2 + (x(4)^2*l3^2*m3*sin(2*x(2) + 2*x(3)))/2 - x(6)^2*l2*l3*m3*sin(x(3)) + x(4)^2*l2*l3*m3*sin(2*x(2) + x(3)) - 2*x(5)*x(6)*l2*l3*m3*sin(x(3));
            (l3*m3*(x(4)^2*l3*sin(2*x(2) + 2*x(3)) + x(4)^2*l2*sin(x(3)) + 2*x(5)^2*l2*sin(x(3)) + x(4)^2*l2*sin(2*x(2) + x(3))))/2];
        
        Gmat = [0;
            (g*l2*m2*cos(x(2))) + (g*l2*m3*cos(x(2))) + (g*l3*m3*cos(x(2) + x(3)));
            (g*l3*m3*cos(x(2) + x(3)))];
                                     
        invMC = Mmat\Cmat;
        
        tau = PDControl(theta_d,dtheta_d,ddtheta_d,theta,dtheta);
        
        torque =[torque, tau];

        dx=zeros(6,1);
        dx(1) = x(4); %dtheta1
        dx(2) = x(5); %dtheta2
        dx(3) = x(6); %dtheta3
        if without_GC
            dx(4:6) = -transpose(invMC)* x(4:6) + Mmat\tau - Mmat\Gmat; % because ddot theta = -M^{-1}(C \dot Theta) + M^{-1} tau - M^{-1}G
        else
            dx(4:6) = -transpose(invMC)* x(4:6) + Mmat\tau; % because ddot theta = -M^{-1}(C \dot Theta) + M^{-1} tau
        end
    end


    function tau = PDControl(theta_d,dtheta_d,ddtheta_d,theta,dtheta)
            Kp=[20 0 0; 0 15 0; 0 0 30];
            Kv=Kp;
            e=theta_d-theta; % position error
            de = dtheta_d - dtheta; % velocity error
            tau = Kp*e + Kv*de;
    end

    function position = RobotFK(joint_values)
        position=[];
        for i=1:size(joint_values,2)
           j1= joint_values(1,i);
           j2= joint_values(2,i);
           j3= joint_values(3,i);
            Point_3j = [cos(j1)*(l3*cos(j2 + j3) + l2*cos(j2));
                sin(j1)*(l3*cos(j2 + j3) + l2*cos(j2));
                l1 + l3*sin(j2 + j3) + l2*sin(j2)];
           position = [position, Point_3j];
        end
    end

    function position = RobotFK_L2(joint_values)
        position=[];
        for i=1:size(joint_values,2)
           j1= joint_values(1,i);
           j2= joint_values(2,i);
           Point_2j = [l2*cos(j1)*cos(j2);
                l2*cos(j2)*sin(j1);
                l1 + l2*sin(j2)];
           position = [position, Point_2j];
        end
    end


    function [] = no_animator(link_2,link_3,time)
        if without_GC
            name11 = 'End-Effector Trajectory Without Gravity Compensation';
        else
            name11 = 'End-Effector Trajectory With Gravity Compensation';
        end
        figure('Name',name11);
        subplot(2,2,2);
            plot3([0 0],[0 0],[0 l1],'k-','LineWidth', 3);
            axis equal
            xlabel('X Position [m]')
            ylabel('Y Position [m]')
            zlabel('Z Position [m]')
            title('3-D View')
            grid on;
            hold on;
            % Initial Position
            plot3([0 link_2(1,1)],[0 link_2(2,1)],[l1 link_2(3,1)],'k--','LineWidth', 2);
            plot3([link_2(1,1) link_3(1,1)],[link_2(2,1) link_3(2,1)],[link_2(3,1) link_3(3,1)],'k--','LineWidth', 2);
            plot3(link_3(1,1),link_3(2,1),link_3(3,1),'Marker','o','MarkerFaceColor','g');
            % Final Position
            plot3([0 link_2(1,size(time,1))],[0 link_2(2,size(time,1))],[l1 link_2(3,size(time,1))],'k-','LineWidth', 3);
            plot3([link_2(1,size(time,1)) link_3(1,size(time,1))],[link_2(2,size(time,1)) link_3(2,size(time,1))],[link_2(3,size(time,1)) link_3(3,size(time,1))],'k-','LineWidth', 3);
            plot3(link_3(1,1:size(time,1)),link_3(2,1:size(time,1)),link_3(3,1:size(time,1)),'b-');
            plot3(link_3(1,size(time,1)),link_3(2,size(time,1)),link_3(3,size(time,1)),'Marker','o','MarkerFaceColor','r');
        subplot(2,2,1);
            plot3([0 0],[0 0],[0 l1],'k-','LineWidth', 3);
            axis equal
            xlabel('X Position [m]')
            ylabel('Y Position [m]')
            zlabel('Z Position [m]')
            title('Top View')
            grid on;
            hold on;
            % Initial Position
            plot3([0 link_2(1,1)],[0 link_2(2,1)],[l1 link_2(3,1)],'k--','LineWidth', 2);
            plot3([link_2(1,1) link_3(1,1)],[link_2(2,1) link_3(2,1)],[link_2(3,1) link_3(3,1)],'k--','LineWidth', 2);
            plot3(link_3(1,1),link_3(2,1),link_3(3,1),'Marker','o','MarkerFaceColor','g');
            % Final Position
            plot3([0 link_2(1,size(time,1))],[0 link_2(2,size(time,1))],[l1 link_2(3,size(time,1))],'k-','LineWidth', 3);
            plot3([link_2(1,size(time,1)) link_3(1,size(time,1))],[link_2(2,size(time,1)) link_3(2,size(time,1))],[link_2(3,size(time,1)) link_3(3,size(time,1))],'k-','LineWidth', 3);
            plot3(link_3(1,1:size(time,1)),link_3(2,1:size(time,1)),link_3(3,1:size(time,1)),'b-');
            plot3(link_3(1,size(time,1)),link_3(2,size(time,1)),link_3(3,size(time,1)),'Marker','o','MarkerFaceColor','r');
            view(0,90)
        subplot(2,2,3);
            plot3([0 0],[0 0],[0 l1],'k-','LineWidth', 3);
            axis equal
            xlabel('X Position [m]')
            ylabel('Y Position [m]')
            zlabel('Z Position [m]')
            title('Front View')
            grid on;
            hold on;
            % Initial Position
            plot3([0 link_2(1,1)],[0 link_2(2,1)],[l1 link_2(3,1)],'k--','LineWidth', 2);
            plot3([link_2(1,1) link_3(1,1)],[link_2(2,1) link_3(2,1)],[link_2(3,1) link_3(3,1)],'k--','LineWidth', 2);
            plot3(link_3(1,1),link_3(2,1),link_3(3,1),'Marker','o','MarkerFaceColor','g');
            % Final Position
            plot3([0 link_2(1,size(time,1))],[0 link_2(2,size(time,1))],[l1 link_2(3,size(time,1))],'k-','LineWidth', 3);
            plot3([link_2(1,size(time,1)) link_3(1,size(time,1))],[link_2(2,size(time,1)) link_3(2,size(time,1))],[link_2(3,size(time,1)) link_3(3,size(time,1))],'k-','LineWidth', 3);
            plot3(link_3(1,1:size(time,1)),link_3(2,1:size(time,1)),link_3(3,1:size(time,1)),'b-');
            plot3(link_3(1,size(time,1)),link_3(2,size(time,1)),link_3(3,size(time,1)),'Marker','o','MarkerFaceColor','r');
            view(90,0)
        subplot(2,2,4);
            plot3([0 0],[0 0],[0 l1],'k-','LineWidth', 3);
            axis equal
            xlabel('X Position [m]')
            ylabel('Y Position [m]')
            zlabel('Z Position [m]')
            title('Right View')
            grid on;
            hold on;
            % Initial Position
            plot3([0 link_2(1,1)],[0 link_2(2,1)],[l1 link_2(3,1)],'k--','LineWidth', 2);
            plot3([link_2(1,1) link_3(1,1)],[link_2(2,1) link_3(2,1)],[link_2(3,1) link_3(3,1)],'k--','LineWidth', 2);
            plot3(link_3(1,1),link_3(2,1),link_3(3,1),'Marker','o','MarkerFaceColor','g');
            % Final Position
            plot3([0 link_2(1,size(time,1))],[0 link_2(2,size(time,1))],[l1 link_2(3,size(time,1))],'k-','LineWidth', 3);
            plot3([link_2(1,size(time,1)) link_3(1,size(time,1))],[link_2(2,size(time,1)) link_3(2,size(time,1))],[link_2(3,size(time,1)) link_3(3,size(time,1))],'k-','LineWidth', 3);
            plot3(link_3(1,1:size(time,1)),link_3(2,1:size(time,1)),link_3(3,1:size(time,1)),'b-');
            plot3(link_3(1,size(time,1)),link_3(2,size(time,1)),link_3(3,size(time,1)),'Marker','o','MarkerFaceColor','r');
            view(0,0)
            sgtitle('Robot Stick Model and End-Effector Trajectory')
            if print_figures
                set(gcf,'Units','inches'); % Set the units of the figure
                screenposition = get(gcf,'Position'); % Measure the size of the figure
                set(gcf,'PaperPosition', [0 0 screenposition(3:4)],'PaperSize' , (screenposition(3:4))); % Configures the print paper size to fit the figure size
                print(name11,'-dpng','-r600') % Save figure as PNG
            end
    end

disp('Finish.');
end