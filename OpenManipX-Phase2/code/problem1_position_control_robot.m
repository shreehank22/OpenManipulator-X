function problem1_position_control_robot()

    %% Paths 
    codeDir = fileparts(mfilename("fullpath"));
    addpath(fullfile(codeDir, "..", "..", "Communication_Code"));
    dataDir = fullfile(codeDir, "..", "data");

    %% Load parameters
    params = robotParams();
    x_des = [0.20; 0.20; 0.15];
    Kp = 1.5;
    Ts = 0.05;
    Tfinal = 10;
    qd_max = 0.5; % rad/s

    %% joint limits from URDF
    q_min  = [-1.5; -1.5; -1.5; -1.7];
    q_max  = [ 1.5;  1.5;  1.4;  1.97];
    margin = deg2rad(10);

    %% Logs 
    N = round(Tfinal/Ts);
    t_log = zeros(1, N);
    dt_log = zeros(1, N);
    m_log = zeros(1, N);
    q_log = zeros(4, N);
    qdot_meas_log = zeros(4, N);
    qdot_cmd_log = zeros(4, N);
    x_log = zeros(3, N);
    e_log = zeros(3, N);
    kLast = 0;
    stopFlag = "interrupt";

    robot = Robot();
    cleanup = onCleanup(@finishRun);   % runs on normal exit, error, or Ctrl+C
    input("SUPPORT THE ARM, then press Enter to home the robot (position mode)... ", "s");
    robot.writeMode('p');
    robot.writeTime(2);
    robot.writeMotorState(true);
    robot.writeJoints([0 0 0 0]);
    pause(3);

    input("SUPPORT THE ARM, then press Enter to switch to velocity mode... ", "s");
    robot.writeMode('v');
    robot.writeVelocities([0 0 0 0]);


    %%% Control loop
    t0 = tic;
    for k = 1:N
        tloop = tic;

        r=robot.getJointsReadings();
        q = deg2rad(r(1,:)');
        qd_meas = deg2rad(r(2,:)');

        % safety check 
        if any(q < q_min + margin) || any(q > q_max - margin)
            warning("Joint limit reached. Stopping the robot.");
            stopFlag = "limit";
            break;
        end

        % control law
        x = openManipulatorPosition(q, params);
        [qdot_cmd, m] = controlLawTask1(q, x, x_des, params, Kp);

        % limit the commanded velocity
        qdot_cmd = qdot_cmd * min(1, qd_max / max(abs(qdot_cmd)));

        robot.writeVelocities(rad2deg(qdot_cmd'));

        %%% Logging
        t_log(k) = toc(t0);
        q_log(:,k) = q;
        qdot_meas_log(:,k) = qd_meas;
        qdot_cmd_log(:,k) = qdot_cmd;
        x_log(:,k) = x;
        e_log(:,k) = x_des - x;
        m_log(k) = m;
        kLast = k;

        while toc(tloop) < Ts
        end
        dt_log(k) = toc(tloop);
    end

    if kLast == N
        stopFlag = "complete";
    end

    %% Always stop the robot and save whatever was logged
    % Torque is left ON: the arm holds itself at zero velocity. Support the
    % arm before the next writeMode call.
    function finishRun()
        try robot.writeVelocities([0 0 0 0]); catch, end
        if kLast == 0
            fprintf("Stopped (%s) before any samples were logged. Nothing saved.\n", stopFlag);
            return;
        end
        idx = 1:kLast;                                  % drop unused preallocated samples
        t = t_log(idx);  dt_log = dt_log(idx);  m_log = m_log(idx);
        q_log = q_log(:,idx);  qdot_meas_log = qdot_meas_log(:,idx);
        qdot_cmd_log = qdot_cmd_log(:,idx);  x_log = x_log(:,idx);  e_log = e_log(:,idx);
        stamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
        fname = fullfile(dataDir, sprintf("robot_task1_Kp%g_sat%g_%s_%s.mat", ...
                         Kp, qd_max, stopFlag, stamp));
        out = struct("t", t, "dt_log", dt_log, "q_log", q_log, ...
                     "qdot_meas_log", qdot_meas_log, "qdot_cmd_log", qdot_cmd_log, ...
                     "x_log", x_log, "e_log", e_log, "m_log", m_log, ...
                     "Kp", Kp, "qd_max", qd_max, "Ts", Ts, ...
                     "stopFlag", stopFlag);
        out.x_des = x_des;  out.params = params;
        save(fname, "-struct", "out");
        fprintf("Stopped (%s). Saved %d samples to %s\n", stopFlag, kLast, fname);
        fprintf("Final ||e||: %.3e m   mean dt: %.4f s   max dt: %.4f s\n", ...
                norm(e_log(:,end)), mean(dt_log), max(dt_log));
    end
end

