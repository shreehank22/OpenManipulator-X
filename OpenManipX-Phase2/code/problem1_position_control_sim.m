%% Task 1 simulation: end-effector position control (Phase 1 model)
clc; clear; close all;

codeDir = fileparts(mfilename('fullpath'));
dataDir = fullfile(codeDir, '..', 'data');

%% Load parameters 
params = robotParams();
x_des = [0.20; 0.20; 0.15];
Kp = 1.5;          % match the robot run (Phase 1 used 5)
dt = 0.05;
Tfinal = 10;       % robot runs are 10 s
qd_max = 0.5;      % rad/s; set to Inf to reproduce Phase 1 exactly
animate = false;

t = 0:dt:Tfinal;
q = [0; 0; 0; 0];

%% logs 
q_log = zeros(numel(q), length(t));
qdot_cmd_log = zeros(numel(q), length(t));
x_log = zeros(3, length(t));
e_log = zeros(3, length(t));
m_log = zeros(1, length(t));

%% simulation loop
for k = 1:length(t)
    x = openManipulatorPosition(q, params);
    q_log(:,k) = q;
    x_log(:,k) = x;
    e_log(:,k) = x_des - x;
    [qdot,m] = controlLawTask1(q, x, x_des, params, Kp);
    qdot = qdot * min(1, qd_max / max(abs(qdot)));   % same saturation as the robot
    qdot_cmd_log(:,k) = qdot;
    m_log(k) = m;
    q = q + dt*qdot;
end

qdot_meas_log = qdot_cmd_log;
dt_log = dt*ones(1,length(t));

%% Save
save(fullfile(dataDir, sprintf("sim_task1_Kp%g_sat%g.mat", Kp, qd_max)), ...
     "t","q_log","qdot_cmd_log","qdot_meas_log","x_log","e_log","m_log", ...
     "dt_log","Kp","qd_max","x_des","params");

fprintf("Final q: [%.4f %.4f %.4f %.4f] rad\n", q_log(:,end));
fprintf("Final ||e||: %.3e m   Final m_t: %.4e m^3   max|qdot|: %.3f rad/s\n", ...
        norm(e_log(:,end)), m_log(end), max(abs(qdot_cmd_log),[],"all"));
