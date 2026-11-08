function problem2_manipulability_control_robot()
%PROBLEM2_MANIPULABILITY_CONTROL_ROBOT Task 2 (position control + manipulability
%maximization in the null space) on the
%real OpenManipulator-X. Closes the loop on measured joint angles and commands
%joint velocities in velocity mode.

    %% Paths
    codeDir = fileparts(mfilename("fullpath"));
    addpath(fullfile(codeDir, "..", "..", "Communication_Code"));
    % Dynamixel SDK paths (headers + shared library). Needed whenever the SDK
    % library has to be (re)loaded, e.g. after "clear all" or a MATLAB restart.
    run(fullfile(codeDir, "..", "..", "init_libraries.m"));
    dataDir = fullfile(codeDir, "..", "data");

    %% Parameters (re-run the sim with the same values for comparison)
    params = robotParams();
    x_des  = [0.20; 0.20; 0.15];
    Kp     = 1.5;
    k0     = 50;         % null-space gain (Phase 1 sim: 100); start low on hardware
    Ts     = 0.05;        % target sample time (s)
    Tfinal = 10;          % run duration (s)
    qd_max = 0.5;         % joint velocity limit (rad/s)

    %% Joint limits from the URDF (rad); joint 1 restricted for cable safety
    q_min  = [-1.5; -1.5; -1.5; -1.7];
    q_max  = [ 1.5;  1.5;  1.4;  1.97];
    margin = deg2rad(10);

    %% Logs (one struct so they can be passed to saveRun)
    N = round(Tfinal/Ts);
    L.t             = zeros(1, N);
    L.dt_log        = zeros(1, N);
    L.m_log         = zeros(1, N);
    L.q_log         = zeros(4, N);
    L.qdot_meas_log = zeros(4, N);
    L.qdot_cmd_log  = zeros(4, N);
    L.x_log         = zeros(3, N);
    L.e_log         = zeros(3, N);
    kLast    = 0;
    stopFlag = "complete";
    runError = [];

    %% Connect. The guard commands zero velocities whenever this function exits,
    %% including Ctrl+C. It holds its own reference to the robot, so it does not
    %% depend on this function's workspace still existing.
    robot = Robot();
    stopGuard = onCleanup(@() stopRobot(robot)); %#ok<NASGU>

    try
        %% Home (q = 0) in position mode
        input("SUPPORT THE ARM, then press Enter to home the robot (position mode)... ", "s");
        robot.writeMode('p');
        robot.writeTime(2);
        robot.writeMotorState(true);
        robot.writeJoints([0 0 0 0]);
        pause(3);

        input("SUPPORT THE ARM, then press Enter to switch to velocity mode... ", "s");
        robot.writeMode('v');
        % writeMode re-applies the homing profile (667 ms acceleration). In the
        % servos' time-based profile mode every new velocity command restarts that
        % ramp, so with a new command every Ts the joints lag by ~1 s and small
        % commands never execute. Acceleration time 0 disables the profile so the
        % servos follow each command directly (homing calls writeTime(2) again).
        robot.writeTime(2, 0);
        robot.writeVelocities([0 0 0 0]);

        %% Control loop
        fprintf("Control loop running for %g s (Ctrl+C to abort)...\n", Tfinal);
        t0 = tic;
        for k = 1:N
            tLoop = tic;

            % 1) Measure (deg -> rad)
            r = robot.getJointsReadings();
            q       = deg2rad(r(1,:)');
            qd_meas = deg2rad(r(2,:)');

            % 1b) Reject implausible readings. A failed bulk read returns 0 deg;
            %     at qd_max a joint moves < 2 deg per sample, so a 10 deg jump is a bad read.
            if k > 1 && max(abs(q - L.q_log(:,k-1))) > deg2rad(10)
                warning("Implausible joint reading at t = %.2f s (jump > 10 deg). Stopping.", toc(t0));
                stopFlag = "badread";
                break;
            end

            % 2) Safety: stop before any joint gets near its limit
            if any(q < q_min + margin) || any(q > q_max - margin)
                warning("Joint limit reached at t = %.2f s. Stopping the robot.", toc(t0));
                stopFlag = "limit";
                break;
            end

            % 3) Control law (same function as the simulation)
            x = openManipulatorPosition(q, params);
            [qd_cmd, m] = controlLawTask2(q, x, x_des, params, Kp, k0);

            % 4) Saturate by uniform scaling (keeps the direction of qdot)
            qd_cmd = qd_cmd * min(1, qd_max / max(abs(qd_cmd)));

            % 5) Command (rad/s -> deg/s)
            robot.writeVelocities(rad2deg(qd_cmd'));

            % 6) Log
            L.t(k)               = toc(t0);
            L.q_log(:,k)         = q;
            L.qdot_meas_log(:,k) = qd_meas;
            L.qdot_cmd_log(:,k)  = qd_cmd;
            L.x_log(:,k)         = x;
            L.e_log(:,k)         = x_des - x;
            L.m_log(k)           = m;
            kLast = k;

            % (printed before the wait so its time is included in dt_log)
            if mod(k, round(1/Ts)) == 0       % progress once per second
                fprintf("  t = %4.1f s   ||e|| = %.4f m\n", L.t(k), norm(L.e_log(:,k)));
            end

            % 7) Fixed sample time
            while toc(tLoop) < Ts
            end
            L.dt_log(k) = toc(tLoop);
        end
    catch runError
        stopFlag = "error";
    end

    %% Stop and save (normal completion, joint-limit stop, or MATLAB error).
    %% Torque stays ON: the arm holds itself at zero velocity. Support the arm
    %% before the next writeMode call.
    stopRobot(robot);
    meta = struct("Kp", Kp, "qd_max", qd_max, "Ts", Ts, "stopFlag", stopFlag);
    meta.x_des  = x_des;
    meta.params = params;
    meta.k0     = k0;
    stamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
    fname = fullfile(dataDir, sprintf("robot_task2_Kp%g_k0%g_sat%g_%s_%s.mat", Kp, k0, qd_max, stopFlag, stamp));
    saveRun(fname, L, kLast, meta);

    if ~isempty(runError), rethrow(runError); end
end

%% ---------------------------------------------------------------- helpers
function stopRobot(robot)
    % Command zero joint velocities; never throw (used from onCleanup).
    try
        robot.writeVelocities([0 0 0 0]);
    catch
    end
end

function saveRun(fname, L, kLast, meta)
    % Save samples 1:kLast of every log plus the run settings in meta.
    if kLast == 0
        fprintf("Stopped (%s) before any samples were logged. Nothing saved.\n", meta.stopFlag);
        return;
    end
    out = meta;
    names = fieldnames(L);
    for i = 1:numel(names)
        out.(names{i}) = L.(names{i})(:, 1:kLast);
    end
    save(fname, "-struct", "out");
    fprintf("Stopped (%s). Saved %d samples to %s\n", meta.stopFlag, kLast, fname);
    valid = out.dt_log > 0;
    fprintf("Final ||e||: %.3e m   Final m_t: %.4e m^3   mean dt: %.4f s   max dt: %.4f s\n", ...
            norm(out.e_log(:,end)), out.m_log(end), mean(out.dt_log(valid)), max(out.dt_log(valid)));
end
