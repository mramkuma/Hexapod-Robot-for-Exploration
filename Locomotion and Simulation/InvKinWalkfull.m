function [l, L, l_prime, L_prime,s_1, alpha_real, R,u,R_prime] = InvKinWalkfull(P, euler, Rm, Rf, L1,F, slope, t2)

    %% Desired Pose (Given):

%     P = [10 0 100 5 5 0]';

    %% Robot Parameters (Given):
% 
%     Rm = 300/2; %mm
%     Rf = 480/2; %mm
    
%     Rm = 625; %mm
%     Rf = Rm + 1000; %mm


    alpha = deg2rad(60);
    beta = deg2rad(60);

    %% Extract Position and Euler Angles from given pose:

    O = P(1:3,1);
    a = deg2rad(P(4));
    b = deg2rad(P(5));
    c = deg2rad(P(6));
    a_prime = deg2rad(0);
    b_prime = deg2rad(0);
%     c_prime = deg2rad(0);
    c_prime = t2;

    %% Calculate Upper Joint Positions (si) wrt. upper coordinate frame

%     %Calculate leg attachment angles
%     i = 0;
%     theta_odd = [];
%     theta_even = [];
% 
%     for n = 1:6
%         if mod(n,2) ~=0
% 
%             theta_odd = [theta_odd  (i*pi/3)+beta/2];
%             i = i + 2;
%         else
%             theta_even = [theta_even  (i*pi/3)-beta/2];
%         end 
%     end
% 
%     %Calculate joint positions
%     s1 = [Rm*cos(theta_odd(1)), Rm*sin(theta_odd(1)), 0]';
%     s2 = [Rm*cos(theta_even(1)), Rm*sin(theta_even(1)), 0]';
%     s3 = [Rm*cos(theta_odd(2)), Rm*sin(theta_odd(2)), 0]';
%     s4 = [Rm*cos(theta_even(2)), Rm*sin(theta_even(2)), 0]';
%     s5 = [Rm*cos(theta_odd(3)), Rm*sin(theta_odd(3)), 0]';
%     s6 = [Rm*cos(theta_even(3)), Rm*sin(theta_even(3)), 0]';
% 
%     s = [s1, s2, s3, s4, s5, s6];
    %% 
    %Calculate joint positions
    s1 = [Rm*cos(2*beta), Rm*sin(2*beta), 0]';
    s2 = [Rm*cos(1*beta), Rm*sin(1*beta), 0]';
    s3 = [Rm*cos(3*beta), Rm*sin(3*beta), 0]';
    s4 = [Rm*cos(0*beta), Rm*sin(0*beta), 0]';
    s5 = [Rm*cos(4*beta), Rm*sin(4*beta), 0]';
    s6 = [Rm*cos(5*beta), Rm*sin(5*beta), 0]';

    s_1 = [s1, s2, s3, s4, s5, s6];
    

    %% Calculate Lower Joint Positions (si) wrt. lower coordinate frame

%     %Calculate leg attachment angles
%     i = 0;
%     theta_odd = [];
%     theta_even = [];
% 
%     for n = 1:6
%         if mod(n,2) ~=0
% 
%             theta_odd = [theta_odd  (i*pi/3)+alpha/2];
%             i = i + 2;
%         else
%             theta_even = [theta_even  (i*pi/3)-alpha/2];
%         end 
%     end
% 
%     %Calculate joint positions
%     u1 = [Rf*cos(theta_odd(1)), Rf*sin(theta_odd(1)), 0]';
%     u2 = [Rf*cos(theta_even(1)), Rf*sin(theta_even(1)), 0]';
%     u3 = [Rf*cos(theta_odd(2)), Rf*sin(theta_odd(2)), 0]';
%     u4 = [Rf*cos(theta_even(2)), Rf*sin(theta_even(2)), 0]';
%     u5 = [Rf*cos(theta_odd(3)), Rf*sin(theta_odd(3)), 0]';
%     u6 = [Rf*cos(theta_even(3)), Rf*sin(theta_even(3)), 0]';
% 
%     u = [u1, u2, u3, u4, u5, u6];

    %% 
     %% 
    %Calculate joint positions
%     u1 = [Rf*cos(alpha/2), Rf*sin(alpha/2), 0]';
%     u2 = [-Rf*sin(pi/6-alpha/2), Rf*cos(pi/6-alpha/2), 0]';
%     u3 = [-Rf*sin(pi/6+alpha/2), Rf*cos(pi/6+alpha/2), 0]';
%     u4 = [-Rf*cos(pi/3-alpha/2), -Rf*sin(pi/3-alpha/2), 0]';
%     u5 = [-Rf*cos(pi/3+alpha/2), -Rf*sin(pi/3+alpha/2), 0]';
%     u6 = [Rf*cos(alpha/2), -Rf*sin(alpha/2), 0]';
%     u = O+F;
    
%     u1 = [Rf*cos(2*alpha), Rf*sin(2*alpha), 0]';
%     u2 = [Rf*cos(1*alpha), Rf*sin(1*alpha), 0]';
%     u3 = [Rf*cos(3*alpha), Rf*sin(3*alpha), 0]';
%     u4 = [Rf*cos(0*alpha), Rf*sin(0*alpha), 0]';
%     u5 = [Rf*cos(4*alpha), Rf*sin(4*alpha), 0]';
%     u6 = [Rf*cos(5*alpha), Rf*sin(5*alpha), 0]';
% 
%     u = [u1, u2, u3, u4, u5, u6]
    %% Calculate Rotatation Matrix from Euler Angles
    
    if euler == 'ZYZ'
        R1 = Rotz(a);
        R2 = Roty(b);
        R3 = Rotz(c);
%         R_r = R1*R2*R3
%         R1 = [cos(a), -sin(a),0;sin(a), cos(a),0;0,0,1]; %RZ,a
%         R2 = [cos(b), 0, sin(b); 0,1,0;-sin(b), 0, cos(b)]; %RY,b
%         R3 = [cos(c), -sin(c),0;sin(c), cos(c),0;0,0,1]; %RZ,c
        R = R1*R2*R3; %Rzyz
    elseif euler == 'XYZ'
        R1 = Rotx(a);
        R2 = Roty(b);
        R3 = Rotz(c);
        
        R1_prime = Rotx(a_prime);
        R2_prime = Roty(b_prime);
        R3_prime = Rotz(c_prime);
%         R_r = R1*R2*R3
%         R1 = [cos(a), -sin(a),0;sin(a), cos(a),0;0,0,1]; %RZ,a
%         R2 = [cos(b), 0, sin(b); 0,1,0;-sin(b), 0, cos(b)]; %RY,b
%         R3 = [cos(c), -sin(c),0;sin(c), cos(c),0;0,0,1]; %RZ,c
        R = R1*R2*R3;%Rxyz
        R_prime = R1_prime*R2_prime*R3_prime;%Rxyz
    end
        
    u = R_prime*F;
    %% Calculate leg vectors
    n = [];
    for i = 1:6
        L(:,i) = O + R*s_1(:,i) - u(:,i);
%         L(:,i) =  -R*s_1(:,i) + u(:,i);
%         L(:,i) =  O+R*s_1(:,i) - u(:,i);
        l(i) = norm(L(:,i),2);
        n(:,i) = L(:,i)/l(i);
    end
%Calculate alpha angles form teh equations
alpha = round(atan(L(2,:)./L(1,:)),4);
% rad2deg(alpha)
%Calcuate home or nominal hip joint location angles
home_hip = round(atan(s_1(2,:)./s_1(1,:)),4);
% rad2deg(home_hip)

%Obtain "normalized" hip joint angles
alpha_real = alpha-home_hip;
        s_2 = [];
% %     l1 = 0; %Coxa Segment Length
    for i = 1:6
        s_2 = [s_2 , [(s_1(1,i) + (-1)^i*L1*cos(alpha(i))); (s_1(2,i) + (-1)^i*L1*sin(alpha(i))); s_1(3,i)]];
    end
    s_2;
    %% Calculate leg vectors
    n_prime = [];
    for i = 1:6
        L_prime(:,i) = O + R*s_2(:,i) - u(:,i);
%         L_prime(:,i) = R*s_2(:,i) - u(:,i);
        l_prime(i) = norm(L_prime(:,i),2);
        n_prime(:,i) = L_prime(:,i)/l_prime(i);

    end

end

function Rx = Rotx(theta)
%     theta = deg2rad(theta);
    Rx = [1 0 0;
          0  cos(theta) -sin(theta)
          0 sin(theta) cos(theta)];
end

function Ry = Roty(theta)
%     theta = deg2rad(theta);
    Ry = [cos(theta) 0 sin(theta);
          0   1 0
          -sin(theta) 0 cos(theta)];
end

function Rz = Rotz(theta)
%     theta = deg2rad(theta);
    Rz = [cos(theta) -sin(theta) 0;
          sin(theta)  cos(theta) 0
          0 0 1];
end