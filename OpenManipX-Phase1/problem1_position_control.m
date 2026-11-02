clc, clear all, close all;

%% Load rigid body tree
robot = createRigidBodyTree;
robot.DataFormat = "column";
robot.Gravity = [0; 0; -9.81];

%% Controller setup
eeName = "end_effector";

%% Time Variables
dt = 0.05;
Tfinal = 5;
t = 0:dt:Tfinal;

%% The URDF has 6 movable joints: 4 arm joints + 2 gripper joints.
q = [0; 0; 0; 0];
qFull = homeConfiguration(robot);
qFull(:) = 0;

%% The Joints we can control for position control
armJointIdx = 1:4;

%% Geometric parameters
params.a0 = 0.012;
params.d1 = 0.0595;
params.a1 = 0.024;
params.d2 = 0.128;
params.a2 = 0.124;
params.ell_e = 0.12;

%% Desired Location of the end effector
x_des = [0.20; 0.20; 0.15];
Kp = 5.0;

qHist = zeros(numel(q), length(t));
qdotHist = zeros(numel(q), length(t));
xHist = zeros(3, length(t));
errHist = zeros(3, length(t));
mHist = zeros(1, length(t));
%% Simulate differential inverse kinematics controller
figure;
for k = 1:length(t)
    %% Forward Kinematic
    x = openManipulatorPosition(q, params);

    %% Save data
    qHist(:, k) = q;
    xHist(:, k) = x;
    errHist(:, k) = x_des - x;
    %% Control Law
    [qdot, m] = ControLaw(q, x, x_des, params, Kp);
    qdotHist(:, k) = qdot;
    mHist(:, k) = m;
    %% Integration of the velocities
    q = q + dt*qdot;

    %% Update joints
    qFull(armJointIdx) = q;
    show(robot, qFull, "PreservePlot", false, "Frames", "off");
    hold on;
    plot3(x_des(1), x_des(2), x_des(3), "ro", "MarkerSize", 8, "LineWidth", 2);
    hold off;
    axis([-0.1 0.4 -0.3 0.3 0 0.4]);
    grid on;
    drawnow;
end

%% Results logging
% Figures are saved next to the report regardless of the current folder.
figDir = fullfile(fileparts(mfilename("fullpath")), "report", "figures");

% Q1. Angular Displacement of each joint (qHist) and save the plot file
fig = paperFigure;
plot(t, qHist, "LineWidth", 1.2);
xlabel("Time (s)"); ylabel("Joint angle (rad)");
legend("$q_1$", "$q_2$", "$q_3$", "$q_4$", "Interpreter", "latex", "Location", "best");
grid on;
savePaperFigure(fig, fullfile(figDir, "q1_joint_angular_displacement.pdf"));

% Q2. Angular velocity of each joint (qdotHist) and save the plot file
fig = paperFigure;
plot(t, qdotHist, "LineWidth", 1.2);
xlabel("Time (s)"); ylabel("Joint velocity (rad/s)");
legend("$\dot q_1$", "$\dot q_2$", "$\dot q_3$", "$\dot q_4$", "Interpreter", "latex", "Location", "best");
grid on;
savePaperFigure(fig, fullfile(figDir, "q1_joint_angular_velocity.pdf"));

% Q3. End-effector position tracking (dashed desired lines match each axis colour)
fig = paperFigure;
h = plot(t, xHist, "LineWidth", 1.2);
hold on;
for i = 1:3
    yline(x_des(i), "--", "Color", h(i).Color, "LineWidth", 1.0);
end
hold off;
xlabel("Time (s)"); ylabel("Position (m)");
legend(h, "$p_x$", "$p_y$", "$p_z$", "Interpreter", "latex", "Location", "best");
grid on;
savePaperFigure(fig, fullfile(figDir, "q1_end_effector_position.pdf"));

% Norm of position error
positionErrorNorm = vecnorm(errHist, 2, 1);

fig = paperFigure;
plot(t, positionErrorNorm, "LineWidth", 1.2);
xlabel("Time (s)"); ylabel("$\|\mathbf{e}\|$ (m)", "Interpreter", "latex");
grid on;
savePaperFigure(fig, fullfile(figDir, "q1_position_error_norm.pdf"));

% Manipulability History
fig = paperFigure;
plot(t, mHist, "LineWidth", 1.2);
xlabel("Time (s)"); ylabel("$m_t$ (m$^3$)", "Interpreter", "latex");
grid on;
savePaperFigure(fig, fullfile(figDir, "q1_manipulability.pdf"));

%% Value Summary
fprintf("Final joint angles q(T): [%.4f %.4f %.4f %.4f] rad\n", qHist(:, end));
fprintf("Final position error ||e(T)||: %.3e m\n", norm(errHist(:, end)));
fprintf("Final manipulability m_t(T): %.4e m^3\n", mHist(end));
fprintf("Minimum manipulability: %.4e m^3\n", min(mHist));
fprintf("Final sigma_min(J): %.4f m\n", min(svd(analyticJacobian(qHist(:, end), params))));

%% Local functions (MATLAB requires these at the end of a script)
function x = openManipulatorPosition(q, params)
    x = [params.a0 + cos(q(1))*(params.a1*cos(q(2)) + params.d2*sin(q(2)) + params.a2*cos(q(2) + q(3)) + params.ell_e*(cos(q(2) + q(3) + q(4))));
    sin(q(1))*(params.a1*cos(q(2)) + params.d2*sin(q(2)) + params.a2*cos(q(2) + q(3)) + params.ell_e*(cos(q(2) + q(3) + q(4))));
    params.d1 - params.a1*sin(q(2)) + params.d2*cos(q(2)) - params.a2*sin(q(2) + q(3)) - params.ell_e*(sin(q(2) + q(3) + q(4)))];
end

%%
function J = analyticJacobian(q, params)
        c1 = cos(q(1));
        s1 = sin(q(1));
        c2 = cos(q(2));
        s2 = sin(q(2));
        c23 = cos(q(2) + q(3));
        s23 = sin(q(2) + q(3));
        c234 = cos(q(2) + q(3) + q(4));
        s234 = sin(q(2) + q(3) + q(4));

        A = params.a1*c2 + params.d2*s2 + params.a2*c23 + params.ell_e*c234;
        B = -params.a1*s2 + params.d2*c2 - params.a2*s23 - params.ell_e*s234;
        C = -params.a2*s23 - params.ell_e*s234;
        D = -params.ell_e*s234;
        E = -params.a2*c23 - params.ell_e*c234;
        F = -params.ell_e*c234;

        J = [-s1*A, c1*B, c1*C, c1*D;
              c1*A, s1*B, s1*C, s1*D;
              0,    -A,    E,    F];
end

%%
function [qdot, M] = ControLaw(q, x, x_des, params, kp)
        % Calculate Analytical Jacobian
        J = analyticJacobian(q, params);

        % Product of singular values = sqrt(det(J*J')), without negative roundoff
        M = prod(svd(J));
        % Control Law
        positionError = x_des - x;
        qdot = pinv(J) * (kp * positionError);
end

%%
function fig = paperFigure
    % Sized for one IEEE column so exported fonts stay legible in the paper.
    fig = figure("Units", "inches", "Position", [1 1 3.5 2.2], "Color", "white");
    set(fig, "DefaultAxesFontSize", 9, "DefaultLegendFontSize", 8, ...
        "DefaultLineLineWidth", 1.2);
end

function savePaperFigure(fig, fileName)
    exportgraphics(fig, fileName, "ContentType", "vector", "BackgroundColor", "white");
end
