%% ==========================================================
% 3-DOF Articulated Manipulator (3R Serial Arm)
% Complete Simulation: Kinematics, Jacobian, Singularities,
% Dynamics, and Trajectory Safety Checking
%
% Author: Sagar Ali
% Requires: MATLAB Robotics System Toolbox
% ==========================================================

clc
clear
close all

%% ==========================================================
% Section 1: Robot Model (DH Parameters)
% ==========================================================
% Joint 1: a=0,   alpha=pi/2, d=90,  theta = variable
% Joint 2: a=210, alpha=0,    d=0,   theta = variable
% Joint 3: a=170, alpha=0,    d=0,   theta = variable

robot = rigidBodyTree('DataFormat','column');

body1 = rigidBody('body1');
joint1 = rigidBodyJoint('joint1','revolute');
setFixedTransform(joint1,[0 pi/2 90 0],'dh');
body1.Joint = joint1;
addBody(robot,body1,'base');

body2 = rigidBody('body2');
joint2 = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint2,[210 0 0 0],'dh');
body2.Joint = joint2;
addBody(robot,body2,'body1');

body3 = rigidBody('body3');
joint3 = rigidBodyJoint('joint3','revolute');
setFixedTransform(joint3,[170 0 0 0],'dh');
body3.Joint = joint3;
addBody(robot,body3,'body2');

disp('Robot object successfully created.');
showdetails(robot)

%% ==========================================================
% Section 2: Home Configuration
% ==========================================================
% Home selected as [0, 45, -45] deg; [0,0,0] was found to be
% a workspace-boundary singularity during Jacobian analysis.

homeConfig = deg2rad([0; 45; -45]);

figure('Name','Home Configuration')
show(robot,homeConfig,'Frames','on','Visuals','off');
axis equal
xlim([-450 450]); ylim([-450 450]); zlim([0 500]);
view(135,25)
grid on
title('3R Manipulator - Home Configuration')

%% ==========================================================
% Section 3: Transformation Matrix Verification
% ==========================================================

T01 = getTransform(robot,homeConfig,'body1','base');
T02 = getTransform(robot,homeConfig,'body2','base');
T03 = getTransform(robot,homeConfig,'body3','base');

fprintf('\n=========================================\n');
fprintf('Transformation Matrix T01\n');
fprintf('=========================================\n');
disp(T01);

fprintf('\n=========================================\n');
fprintf('Transformation Matrix T02\n');
fprintf('=========================================\n');
disp(T02);

fprintf('\n=========================================\n');
fprintf('Transformation Matrix T03\n');
fprintf('=========================================\n');
disp(T03);

%% ==========================================================
% Section 4: Forward Kinematics Verification (single point)
% ==========================================================

theta1 = deg2rad(30);
theta2 = deg2rad(40);
theta3 = deg2rad(60);

testConfig = [theta1; theta2; theta3];

% Analytical FK
x = cos(theta1) * (210*cos(theta2) + 170*cos(theta2+theta3));
y = sin(theta1) * (210*cos(theta2) + 170*cos(theta2+theta3));
z = 90 + 210*sin(theta2) + 170*sin(theta2+theta3);
P_analytical = [x; y; z];

% MATLAB FK
T = getTransform(robot,testConfig,'body3','base');
P_matlab = T(1:3,4);

fprintf('\n=========================================\n');
fprintf('Forward Kinematics Verification\n');
fprintf('=========================================\n');
disp('Analytical Position (mm)'); disp(P_analytical);
disp('MATLAB Position (mm)');     disp(P_matlab);

positionError = P_matlab - P_analytical;
disp('Position Error (mm)'); disp(positionError);
fprintf('Maximum Position Error = %.3e mm\n', max(abs(positionError)));

%% ==========================================================
% Section 5: Forward Kinematics Batch Validation
% ==========================================================
% Cross-checks MATLAB FK against the analytical FK equation
% across ten different joint-angle test configurations.

fprintf('\n=========================================\n');
fprintf('Forward Kinematics Batch Validation\n');
fprintf('=========================================\n');

testAngles = [
    0    0     0
    20   30    40
    45   60   -20
    90   45    30
    120  90     0
    180  30   -45
    150 120    60
    60   90   -90
    35   50    80
    75   20   -30
];

for i = 1:size(testAngles,1)
    t1 = deg2rad(testAngles(i,1));
    t2 = deg2rad(testAngles(i,2));
    t3 = deg2rad(testAngles(i,3));
    q = [t1; t2; t3];

    T = getTransform(robot,q,'body3','base');
    P_matlab = T(1:3,4);

    x = cos(t1) * (210*cos(t2) + 170*cos(t2+t3));
    y = sin(t1) * (210*cos(t2) + 170*cos(t2+t3));
    z = 90 + 210*sin(t2) + 170*sin(t2+t3);
    P_analytical = [x; y; z];

    err = P_matlab - P_analytical;

    fprintf('\nTest %d\n', i);
    fprintf('Maximum Error = %.3e mm\n', max(abs(err)));
end

%% ==========================================================
% Section 6: Joint Sweep Animation
% ==========================================================

figure('Name','Joint 3 Sweep Animation')
for theta = linspace(0,pi,80)
    q = [0; 0; theta];
    show(robot,q,'Frames','on','Visuals','off','PreservePlot',false);
    axis equal
    xlim([-450 450]); ylim([-450 450]); zlim([0 500]);
    view(135,25)
    drawnow
end

%% ==========================================================
% Section 7: Workspace Generation
% ==========================================================

figure('Name','3R Manipulator Workspace')
hold on
grid on
axis equal
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('3R Manipulator Workspace')
xlim([-450 450]); ylim([-450 450]); zlim([0 500]);
view(135,25)

theta1_range = deg2rad(0:10:180);
theta2_range = deg2rad(0:10:180);
theta3_range = deg2rad(-90:10:90);

for t1 = theta1_range
    for t2 = theta2_range
        for t3 = theta3_range
            q = [t1; t2; t3];
            T = getTransform(robot,q,'body3','base');
            P = T(1:3,4);
            plot3(P(1),P(2),P(3),'.','MarkerSize',6);
        end
    end
end

%% ==========================================================
% Section 8: Jacobian Verification (Manual vs MATLAB)
% ==========================================================

theta1 = deg2rad(35);
theta2 = deg2rad(50);
theta3 = deg2rad(80);
q = [theta1; theta2; theta3];

% MATLAB geometric Jacobian
J_matlab = geometricJacobian(robot,q,'body3');
Jv_matlab = J_matlab(4:6,:);

fprintf('\n=========================================\n');
fprintf('MATLAB Geometric Jacobian\n');
fprintf('=========================================\n');
disp(J_matlab);

% Manual Jacobian via skew/cross-product method
O0 = [0;0;0];
O1v = getTransform(robot,q,'body1','base'); O1v = O1v(1:3,4);
O2v = getTransform(robot,q,'body2','base'); O2v = O2v(1:3,4);
O3v = getTransform(robot,q,'body3','base'); O3v = O3v(1:3,4);

Z0 = [0;0;1];
Z1 = getTransform(robot,q,'body1','base'); Z1 = Z1(1:3,3);
Z2 = getTransform(robot,q,'body2','base'); Z2 = Z2(1:3,3);

Jv1 = cross(Z0, O3v - O0);
Jv2 = cross(Z1, O3v - O1v);
Jv3 = cross(Z2, O3v - O2v);

Jv_manual = [Jv1 Jv2 Jv3];

fprintf('\n=========================================\n');
fprintf('Manual Jacobian\n');
fprintf('=========================================\n');
disp(Jv_manual);

% Analytical closed-form Jacobian (from Lagrangian derivation)
P = 210*cos(theta2) + 170*cos(theta2+theta3);
Q = 210*sin(theta2) + 170*sin(theta2+theta3);

Jv_analytical = [
    -sin(theta1)*P, -cos(theta1)*Q, -170*cos(theta1)*sin(theta2+theta3);
     cos(theta1)*P, -sin(theta1)*Q, -170*sin(theta1)*sin(theta2+theta3);
     0,               P,             170*cos(theta2+theta3)
];

fprintf('\n=========================================\n');
fprintf('Analytical Linear Jacobian\n');
fprintf('=========================================\n');
disp(Jv_analytical);

JacobianError = Jv_matlab - Jv_analytical;
fprintf('\n=========================================\n');
fprintf('Jacobian Error (MATLAB vs Analytical)\n');
fprintf('=========================================\n');
disp(JacobianError);
fprintf('Maximum Jacobian Error = %.3e\n', max(abs(JacobianError),[],'all'));

%% ==========================================================
% Section 9: Jacobian Singularity Analysis
% ==========================================================

fprintf('\n=========================================\n');
fprintf('Jacobian Singularity Analysis\n');
fprintf('=========================================\n');

% Normal configuration
q_normal = deg2rad([35; 50; 30]);
J_normal = geometricJacobian(robot,q_normal,'body3');
Jv_normal = J_normal(4:6,:);
fprintf('\nNormal Configuration\n');
fprintf('Rank = %d\n', rank(Jv_normal));
fprintf('Determinant = %.4f\n', det(Jv_normal));

% Expected singular configuration (theta3 = 0, elbow singularity)
q_singular = deg2rad([35; 50; 0]);
J_singular = geometricJacobian(robot,q_singular,'body3');
Jv_singular = J_singular(4:6,:);
fprintf('\nExpected Singular Configuration (theta3 = 0)\n');
fprintf('Rank = %d\n', rank(Jv_singular));
fprintf('Determinant = %.4f\n', det(Jv_singular));

% Home position singularity check ([0,0,0])
q_home = [0; 0; 0];
J_home = geometricJacobian(robot,q_home,'body3');
Jv_home = J_home(4:6,:);
fprintf('\nOrigin Configuration [0,0,0]\n');
fprintf('Rank = %d\n', rank(Jv_home));
fprintf('Determinant = %.4f\n', det(Jv_home));

%% ==========================================================
% Section 10: Workspace Manipulability Analysis
% ==========================================================

fprintf('\n=========================================\n');
fprintf('Workspace Manipulability Analysis\n');
fprintf('=========================================\n');

figure('Name','Workspace Manipulability')
hold on
grid on
axis equal
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('3R Manipulator Workspace Manipulability')
xlim([-450 450]); ylim([-450 450]); zlim([0 500]);
view(135,25)

theta1_range = deg2rad(0:15:180);
theta2_range = deg2rad(0:15:180);
theta3_range = deg2rad(-90:15:90);

manip_points = [];

for t1 = theta1_range
    for t2 = theta2_range
        for t3 = theta3_range
            q = [t1; t2; t3];
            T = getTransform(robot,q,'body3','base');
            P = T(1:3,4);

            J = geometricJacobian(robot,q,'body3');
            Jv = J(4:6,:);
            w = sqrt(abs(det(Jv*Jv')));

            manip_points = [manip_points; P(1) P(2) P(3) w]; %#ok<AGROW>
        end
    end
end

scatter3(manip_points(:,1),manip_points(:,2),manip_points(:,3), ...
    20, manip_points(:,4), 'filled')
colorbar

%% ==========================================================
% Section 11: Physical Parameters
% ==========================================================

% Base geometry (mm)
d1 = 90;
L2 = 210;
L3 = 170;

% Center of mass locations (mm)
lc2 = 105;
lc3 = 85;

% Link masses (kg)
m1 = 0.273;
m2 = 0.207;
m3 = 0.200;

% Moments of inertia (kg.mm^2)
I1 = 150.22;
I2 = 3095.08;
I3 = 1977.08;

% Gravity (m/s^2)
g = 9.81;

fprintf('\n=========================================\n');
fprintf('Physical Parameters\n');
fprintf('=========================================\n');
fprintf('Base Height (d1)      = %.1f mm\n', d1);
fprintf('Link 2 Length (L2)    = %.1f mm\n', L2);
fprintf('Link 3 Length (L3)    = %.1f mm\n', L3);
fprintf('COM Link 2 (lc2)      = %.1f mm\n', lc2);
fprintf('COM Link 3 (lc3)      = %.1f mm\n', lc3);
fprintf('Mass Link 1 (m1)      = %.3f kg\n', m1);
fprintf('Mass Link 2 (m2)      = %.3f kg\n', m2);
fprintf('Mass Link 3 (m3)      = %.3f kg\n', m3);
fprintf('Inertia Link 1 (I1)   = %.2f kg.mm^2\n', I1);
fprintf('Inertia Link 2 (I2)   = %.2f kg.mm^2\n', I2);
fprintf('Inertia Link 3 (I3)   = %.2f kg.mm^2\n', I3);
fprintf('Gravity (g)           = %.2f m/s^2\n', g);

%% ==========================================================
% Section 12: Center of Mass Verification
% ==========================================================

fprintf('\n=========================================\n');
fprintf('Center of Mass Verification\n');
fprintf('=========================================\n');

q = deg2rad([35; 50; 30]);

T01 = getTransform(robot,q,'body1','base');
T02 = getTransform(robot,q,'body2','base');
T03 = getTransform(robot,q,'body3','base');

O1 = T01(1:3,4);
O2 = T02(1:3,4);
O3 = T03(1:3,4);

direction12 = (O2 - O1)/norm(O2 - O1);
COM2 = O1 + lc2*direction12;

direction23 = (O3 - O2)/norm(O3 - O2);
COM3 = O2 + lc3*direction23;

disp('COM of Link 2 (mm)'); disp(COM2);
disp('COM of Link 3 (mm)'); disp(COM3);

figure('Name','Center of Mass Verification')
show(robot,q,'Frames','on','Visuals','off');
hold on
grid on
axis equal
xlim([-450 450]); ylim([-450 450]); zlim([0 500]);
view(135,25)
plot3(COM2(1),COM2(2),COM2(3),'ro','MarkerSize',10,'LineWidth',2);
plot3(COM3(1),COM3(2),COM3(3),'bo','MarkerSize',10,'LineWidth',2);
text(COM2(1),COM2(2),COM2(3),'  COM2','FontWeight','bold');
text(COM3(1),COM3(2),COM3(3),'  COM3','FontWeight','bold');
title('Center of Mass Verification')

%% ==========================================================
% Section 13: Mass Matrix Verification
% ==========================================================

fprintf('\n=========================================\n');
fprintf('Mass Matrix Verification\n');
fprintf('=========================================\n');

theta1 = deg2rad(35);
theta2 = deg2rad(50);
theta3 = deg2rad(30);

c2  = cos(theta2);
c3  = cos(theta3);
c23 = cos(theta2 + theta3);

M11 = I1 + m2*(lc2^2)*(c2^2) + I2 + m3*(L2*c2 + lc3*c23)^2 + I3;
M22 = m2*(lc2^2) + I2 + m3*(L2^2 + lc3^2 + 2*L2*lc3*c3) + I3;
M23 = m3*(lc3^2 + L2*lc3*c3) + I3;
M33 = m3*(lc3^2) + I3;

M = [
    M11   0     0
    0    M22   M23
    0    M23   M33
];

disp('Mass Matrix M(q)'); disp(M);

% Symmetry check
symmetryError = M - transpose(M);
maximumSymmetryError = max(abs(symmetryError(:)));
fprintf('\nSymmetry Check - Maximum Difference = %.3e\n', maximumSymmetryError);

if maximumSymmetryError < 1e-10
    disp('Mass matrix is symmetric.')
else
    warning('Mass matrix is NOT symmetric.')
end

% Positive definiteness check
eigenValues = eig(M);
fprintf('\nPositive Definiteness Check\n');
disp('Eigenvalues'); disp(eigenValues);

if all(eigenValues > 0)
    disp('Mass matrix is positive definite.')
else
    warning('Mass matrix is NOT positive definite.')
end

%% ==========================================================
% Section 14: Gravity Vector Verification
% ==========================================================

fprintf('\n=========================================\n');
fprintf('Gravity Vector Verification\n');
fprintf('=========================================\n');

theta2 = deg2rad(50);
theta3 = deg2rad(30);

c2  = cos(theta2);
c23 = cos(theta2 + theta3);

G1 = 0;
G2 = -(m2*g*lc2*c2 + m3*g*(L2*c2 + lc3*c23));
G3 = m3*g*lc3*c23;

G = [G1; G2; G3];

disp('Gravity Vector G(q)'); disp(G);

%% ==========================================================
% Section 15: Joint Torques at a Test Configuration
% ==========================================================

fprintf('\n=========================================\n');
fprintf('Dynamics Test Configuration\n');
fprintf('=========================================\n');

theta1 = deg2rad(35);
theta2 = deg2rad(50);
theta3 = deg2rad(30);

theta1_dot = 0.60;
theta2_dot = 0.40;
theta3_dot = 0.20;

theta1_ddot = 0.50;
theta2_ddot = 0.30;
theta3_ddot = 0.10;

disp('Joint Angles (rad)');        disp([theta1 theta2 theta3]);
disp('Joint Velocities (rad/s)');  disp([theta1_dot theta2_dot theta3_dot]);
disp('Joint Accelerations (rad/s^2)'); disp([theta1_ddot theta2_ddot theta3_ddot]);

c2 = cos(theta2); s2 = sin(theta2);
c3 = cos(theta3); s3 = sin(theta3);
c23 = cos(theta2 + theta3); s23 = sin(theta2 + theta3);

tau1 = -(I1 + m2*lc2^2*c2^2 + I2 + m3*(L2*c2 + lc3*c23)^2 + I3) * theta1_ddot ...
       - 2*m2*lc2^2*c2*s2*theta1_dot*theta2_dot ...
       - 2*m3*(L2*c2 + lc3*c23)*L2*s2*theta1_dot*theta2_dot ...
       - 2*m3*(L2*c2 + lc3*c23)*lc3*s23*theta1_dot*theta2_dot ...
       - 2*m3*(L2*c2 + lc3*c23)*lc3*s23*theta1_dot*theta3_dot;

tau2 = (m2*lc2^2 + I2 + m3*(L2^2 + lc3^2 + 2*L2*lc3*c3) + I3) * theta2_ddot ...
       - (m3*(lc3^2 + L2*lc3*c3) + I3) * theta3_ddot ...
       - 2*m3*L2*lc3*s3*theta2_dot*theta3_dot ...
       - m3*L2*lc3*s3*theta3_dot^2 ...
       - (m2*lc2^2*c2*s2 + m3*(L2*c2 + lc3*c23)*(L2*s2 + lc3*s23)) * theta1_dot^2 ...
       - m2*g*lc2*c2 ...
       - m3*g*(L2*c2 + lc3*c23);

tau3 = (m3*(lc3^2 + L2*lc3*c3) + I3) * theta2_ddot ...
       - (m3*lc3^2 + I3) * theta3_ddot ...
       - m3*L2*lc3*s3*theta2_dot*theta3_dot ...
       + m3*lc3*(L2*c2 + lc3*c23)*s23*theta1_dot^2 ...
       + m3*L2*lc3*s3*theta2_dot^2 ...
       + m3*g*lc3*c23;

tau = [tau1; tau2; tau3];

fprintf('\nTau1 = %.4f\n', tau1);
fprintf('Tau2 = %.4f\n', tau2);
fprintf('Tau3 = %.4f\n', tau3);
disp('Joint Torque Vector'); disp(tau);

%% ==========================================================
% Section 16: Dynamic Torque Analysis Over Time
% ==========================================================

fprintf('\n=========================================\n');
fprintf('Dynamic Torque Analysis\n');
fprintf('=========================================\n');

time = linspace(0,5,100);

tau1_history = zeros(size(time));
tau2_history = zeros(size(time));
tau3_history = zeros(size(time));

for k = 1:length(time)
    theta1 = deg2rad(90*sin(2*pi*time(k)/5));
    theta2 = deg2rad(45 + 20*sin(2*pi*time(k)/5));
    theta3 = deg2rad(-45 + 20*cos(2*pi*time(k)/5));

    theta1_dot = deg2rad(90)*(2*pi/5)*cos(2*pi*time(k)/5);
    theta2_dot = deg2rad(20)*(2*pi/5)*cos(2*pi*time(k)/5);
    theta3_dot = -deg2rad(20)*(2*pi/5)*sin(2*pi*time(k)/5);

    theta1_ddot = -deg2rad(90)*(2*pi/5)^2*sin(2*pi*time(k)/5);
    theta2_ddot = -deg2rad(20)*(2*pi/5)^2*sin(2*pi*time(k)/5);
    theta3_ddot = -deg2rad(20)*(2*pi/5)^2*cos(2*pi*time(k)/5);

    c2 = cos(theta2); s2 = sin(theta2);
    c3 = cos(theta3); s3 = sin(theta3);
    c23 = cos(theta2+theta3); s23 = sin(theta2+theta3);

    tau1_k = -(I1 + m2*lc2^2*c2^2 + I2 + m3*(L2*c2 + lc3*c23)^2 + I3)*theta1_ddot ...
        - 2*m2*lc2^2*c2*s2*theta1_dot*theta2_dot ...
        - 2*m3*(L2*c2+lc3*c23)*L2*s2*theta1_dot*theta2_dot ...
        - 2*m3*(L2*c2+lc3*c23)*lc3*s23*theta1_dot*theta2_dot ...
        - 2*m3*(L2*c2+lc3*c23)*lc3*s23*theta1_dot*theta3_dot;

    tau2_k = (m2*lc2^2 + I2 + m3*(L2^2 + lc3^2 + 2*L2*lc3*c3)+I3)*theta2_ddot ...
        - (m3*(lc3^2+L2*lc3*c3)+I3)*theta3_ddot ...
        - 2*m3*L2*lc3*s3*theta2_dot*theta3_dot ...
        - m3*L2*lc3*s3*theta3_dot^2 ...
        - (m2*lc2^2*c2*s2 + m3*(L2*c2+lc3*c23)*(L2*s2+lc3*s23))*theta1_dot^2 ...
        - m2*g*lc2*c2 ...
        - m3*g*(L2*c2+lc3*c23);

    tau3_k = (m3*(lc3^2+L2*lc3*c3)+I3)*theta2_ddot ...
        - (m3*lc3^2+I3)*theta3_ddot ...
        - m3*L2*lc3*s3*theta2_dot*theta3_dot ...
        + m3*lc3*(L2*c2+lc3*c23)*s23*theta1_dot^2 ...
        + m3*L2*lc3*s3*theta2_dot^2 ...
        + m3*g*lc3*c23;

    tau1_history(k) = tau1_k;
    tau2_history(k) = tau2_k;
    tau3_history(k) = tau3_k;
end

figure('Name','Joint Torque vs Time')
plot(time,tau1_history,'LineWidth',2)
hold on
plot(time,tau2_history,'LineWidth',2)
plot(time,tau3_history,'LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('Torque')
title('Joint Torque vs Time')
legend('\tau_1','\tau_2','\tau_3')

%% ==========================================================
% Section 17: Maximum Torque Analysis
% ==========================================================

fprintf('\n=========================================\n');
fprintf('Maximum Torque Analysis\n');
fprintf('=========================================\n');

fprintf('\nJoint 1\n');
fprintf('Maximum Torque = %.2f\n', max(tau1_history));
fprintf('Minimum Torque = %.2f\n', min(tau1_history));

fprintf('\nJoint 2\n');
fprintf('Maximum Torque = %.2f\n', max(tau2_history));
fprintf('Minimum Torque = %.2f\n', min(tau2_history));

fprintf('\nJoint 3\n');
fprintf('Maximum Torque = %.2f\n', max(tau3_history));
fprintf('Minimum Torque = %.2f\n', min(tau3_history));

fprintf('\nAbsolute Maximum Torques\n');
fprintf('Joint 1 = %.2f\n', max(abs(tau1_history)));
fprintf('Joint 2 = %.2f\n', max(abs(tau2_history)));
fprintf('Joint 3 = %.2f\n', max(abs(tau3_history)));

%% ==========================================================
% Section 18: Singularity-Safe Trajectory Planning
% ==========================================================
% Sweeps a smooth trajectory and rejects any waypoint that is
% at or near a singular configuration, using checkTrajectoryPoint.

fprintf('\n=========================================\n');
fprintf('Singularity-Safe Trajectory Check\n');
fprintf('=========================================\n');

figure('Name','Singularity-Safe Trajectory')
hold on
grid on
axis equal
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
xlim([-450 450]); ylim([-450 450]); zlim([0 500]);
view(135,25)

trajectory = [];

for k = linspace(0,1,120)
    theta1 = deg2rad(180*k);
    theta2 = deg2rad(45 + 20*sin(pi*k));
    theta3 = deg2rad(-45 + 20*cos(pi*k));

    q = [theta1; theta2; theta3];

    if checkTrajectoryPoint(robot,q) == false
        warning('Trajectory stopped: singularity detected');
        break
    end

    show(robot,q,'Frames','on','Visuals','off','PreservePlot',false);

    T = getTransform(robot,q,'body3','base');
    P = T(1:3,4);
    trajectory = [trajectory P]; %#ok<AGROW>

    plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:),'LineWidth',2);

    drawnow
end

%% ==========================================================
% Section 19: Final Singularity/Manipulability Check
% ==========================================================

checkSingularity(robot,q_normal);


%% ==========================================================
% Local Functions
% ==========================================================

function safe = checkTrajectoryPoint(robot,q)
% Returns false if the given joint configuration is singular
% or near-singular (low manipulability), true otherwise.

    J = geometricJacobian(robot,q,'body3');
    Jv = J(4:6,:);

    rankJ = rank(Jv);
    manipulability = sqrt(abs(det(Jv*Jv')));

    minimumManipulability = 1e6;

    safe = true;

    if rankJ < 3
        warning('Singular configuration detected.');
        safe = false;
    elseif manipulability < minimumManipulability
        warning('Near singular configuration detected.');
        safe = false;
    end
end


function checkSingularity(robot,q)
% Prints the Jacobian rank and manipulability index for a given
% configuration and reports whether it is safe, near-singular,
% or singular.

    J = geometricJacobian(robot,q,'body3');
    Jv = J(4:6,:);

    rank_J = rank(Jv);
    manipulability = sqrt(abs(det(Jv*Jv')));

    fprintf('\n-------------------------------\n');
    fprintf('Singularity Check\n');
    fprintf('-------------------------------\n');
    fprintf('Jacobian Rank = %d\n', rank_J);
    fprintf('Manipulability = %.3f\n', manipulability);

    threshold = 1e5;

    if rank_J < 3
        warning('Robot is in a singular configuration!');
    elseif manipulability < threshold
        warning('Robot is close to singularity!');
    else
        disp('Configuration is safe.');
    end
end
