function plotComparison(simFile, robotFile, tag)
%PLOTCOMPARISON Overlay simulation and robot results for the Phase 2 report.
%   plotComparison(simFile, robotFile, tag)
%
%   simFile   : .mat saved by problem*_sim.m
%   robotFile : .mat saved by problem*_robot.m
%               (a bare file name is looked up in ../data)
%   tag       : prefix for the figure files, e.g. "task1" saves
%               report/figures/task1_joint_angles.pdf, ...
%
%   Figures (each with simulation and robot overlaid):
%     <tag>_joint_angles       q_i(t)                      2x2, double column
%     <tag>_joint_velocities   commanded/measured qdot_i   2x2, double column
%     <tag>_ee_position        p_x, p_y, p_z vs desired    1x3, double column
%     <tag>_error_norm         ||e(t)||                    single column
%     <tag>_manipulability     m_t(q(t))   (Task 2 only)   single column
%     <tag>_sample_time        achieved sample time        single column
%
%   Example:
%     plotComparison("sim_task1_Kp1.5_sat0.5.mat", ...
%                    "robot_task1_Kp1.5_sat0.5_complete_20261111_093512.mat", "task1")

    tag = string(tag);
    codeDir = fileparts(mfilename("fullpath"));
    dataDir = fullfile(codeDir, "..", "data");
    figDir  = fullfile(codeDir, "..", "report", "figures");
    if ~isfolder(figDir), mkdir(figDir); end

    S = load(resolvePath(simFile, dataDir));
    R = load(resolvePath(robotFile, dataDir));

    % The spec compares results "for the same gains": warn if they differ
    checkSame("Kp", S, R);
    checkSame("k0", S, R);
    checkSame("qd_max", S, R);

    % Sim files have no Ts/stopFlag; fill them in so a sim file can stand in
    % for the robot file when testing this function
    if ~isfield(R, "Ts"),       R.Ts = median(R.dt_log); end
    if ~isfield(R, "stopFlag"), R.stopFlag = "n/a (simulation file)"; end

    %% Styles: simulation dashed grey, robot solid blue, commands orange
    cSim = [0.45 0.45 0.45];
    cRob = [0.0000 0.4470 0.7410];
    cCmd = [0.8500 0.3250 0.0980];
    cDes = [0.15 0.15 0.15];
    lw   = 1.1;
    tEnd = max(S.t(end), R.t(end));

    %% 1) Joint angles q_i(t)
    fig = newFigure(7.16, 3.6);
    tl = tiledlayout(fig, 2, 2, "TileSpacing", "compact", "Padding", "compact");
    for i = 1:4
        ax = nexttile(tl);
        plot(ax, S.t, S.q_log(i,:), "--", "Color", cSim, "LineWidth", lw); hold(ax, "on");
        plot(ax, R.t, R.q_log(i,:), "-",  "Color", cRob, "LineWidth", lw); hold(ax, "off");
        styleAxes(ax, tEnd, i > 2, sprintf("$q_%d$ (rad)", i));
        if i == 1, legend(ax, "Simulation", "Robot", "Location", "best"); end
    end
    saveFig(fig, figDir, tag + "_joint_angles");

    %% 2) Joint velocities: robot commanded vs measured, plus simulation
    fig = newFigure(7.16, 3.6);
    tl = tiledlayout(fig, 2, 2, "TileSpacing", "compact", "Padding", "compact");
    for i = 1:4
        ax = nexttile(tl);
        plot(ax, S.t, S.qdot_cmd_log(i,:),  "--", "Color", cSim, "LineWidth", lw); hold(ax, "on");
        plot(ax, R.t, R.qdot_cmd_log(i,:),  "-",  "Color", cCmd, "LineWidth", lw);
        plot(ax, R.t, R.qdot_meas_log(i,:), "-",  "Color", cRob, "LineWidth", lw); hold(ax, "off");
        styleAxes(ax, tEnd, i > 2, sprintf("$\\dot q_%d$ (rad/s)", i));
        if i == 1
            legend(ax, "Simulation", "Robot commanded", "Robot measured", "Location", "northeast");
        end
    end
    saveFig(fig, figDir, tag + "_joint_velocities");

    %% 3) End-effector position from measured joint angles vs desired
    fig = newFigure(7.16, 2.3);
    tl = tiledlayout(fig, 1, 3, "TileSpacing", "compact", "Padding", "compact");
    names = ["x", "y", "z"];
    for i = 1:3
        ax = nexttile(tl);
        plot(ax, S.t, S.x_log(i,:), "--", "Color", cSim, "LineWidth", lw); hold(ax, "on");
        plot(ax, R.t, R.x_log(i,:), "-",  "Color", cRob, "LineWidth", lw);
        yline(ax, R.x_des(i), ":", "Color", cDes, "LineWidth", 1.2); hold(ax, "off");
        styleAxes(ax, tEnd, true, sprintf("$p_%s$ (m)", names(i)));
        if i == 1, legend(ax, "Simulation", "Robot", "Desired", "Location", "best"); end
    end
    saveFig(fig, figDir, tag + "_ee_position");

    %% 4) Position error norm (log scale shows the final error level)
    eSim = vecnorm(S.e_log, 2, 1);
    eRob = vecnorm(R.e_log, 2, 1);
    fig = newFigure(3.5, 2.3);
    ax = axes(fig);
    semilogy(ax, S.t, eSim, "--", "Color", cSim, "LineWidth", lw); hold(ax, "on");
    semilogy(ax, R.t, eRob, "-",  "Color", cRob, "LineWidth", lw); hold(ax, "off");
    styleAxes(ax, tEnd, true, "$\|\mathbf{e}\|$ (m)");
    % Scale to the robot's error: the sim keeps converging to ~1e-14, which
    % would squash the robot's steady-state floor. Lower limit = one decade
    % below the robot's smallest error; the sim line simply leaves the plot.
    eMin = min(eRob(eRob > 0));
    ylim(ax, [10^(floor(log10(eMin)) - 1), 10^ceil(log10(max([eSim eRob])))]);
    legend(ax, "Simulation", "Robot", "Location", "best");
    saveFig(fig, figDir, tag + "_error_norm");

    %% 5) Translational manipulability (Task 2 only)
    if isfield(R, "k0")
        fig = newFigure(3.5, 2.3);
        ax = axes(fig);
        plot(ax, S.t, S.m_log, "--", "Color", cSim, "LineWidth", lw); hold(ax, "on");
        plot(ax, R.t, R.m_log, "-",  "Color", cRob, "LineWidth", lw); hold(ax, "off");
        styleAxes(ax, tEnd, true, "$m_t$ (m$^3$)");
        legend(ax, "Simulation", "Robot", "Location", "best");
        saveFig(fig, figDir, tag + "_manipulability");
    end

    %% 6) Achieved sample time on the robot (for the discussion section)
    valid = R.dt_log > 0;          % last sample may be 0 if the run was interrupted
    fig = newFigure(3.5, 2.3);
    ax = axes(fig);
    plot(ax, R.t(valid), 1000*R.dt_log(valid), ".", "Color", cRob, "MarkerSize", 6); hold(ax, "on");
    yline(ax, 1000*R.Ts, "--", "Color", cDes, "LineWidth", 1.0); hold(ax, "off");
    styleAxes(ax, tEnd, true, "Sample time (ms)");
    legend(ax, "Achieved", "Target $T_s$", "Interpreter", "latex", "Location", "best");
    saveFig(fig, figDir, tag + "_sample_time");

    %% Summary numbers for the report
    fprintf("\n=== %s: simulation vs robot ===\n", tag);
    fprintf("Final ||e||    sim: %.3e m    robot: %.3e m\n", eSim(end), eRob(end));
    fprintf("Final q (rad)  sim:   [%s]\n", num2str(S.q_log(:,end)', "%9.4f"));
    fprintf("Final q (rad)  robot: [%s]\n", num2str(R.q_log(:,end)', "%9.4f"));
    fprintf("Final m_t      sim: %.4e    robot: %.4e m^3\n", S.m_log(end), R.m_log(end));
    fprintf("Robot sample time: mean %.1f ms, max %.1f ms (target %.0f ms)\n", ...
            1000*mean(R.dt_log(valid)), 1000*max(R.dt_log(valid)), 1000*R.Ts);
    fprintf("Robot run ended: %s\n", R.stopFlag);
    fprintf("Figures saved to %s\n", figDir);
end

%% ---------------------------------------------------------------- helpers
function p = resolvePath(f, dataDir)
    if isfile(f)
        p = f;
    elseif isfile(fullfile(dataDir, f))
        p = fullfile(dataDir, f);
    else
        error("plotComparison:fileNotFound", "Cannot find %s (also looked in %s)", f, dataDir);
    end
end

function checkSame(name, S, R)
    inS = isfield(S, name);  inR = isfield(R, name);
    if inS ~= inR
        warning("plotComparison:mismatch", "'%s' is saved in only one file: comparing different tasks?", name);
    elseif inS && ~isequal(S.(name), R.(name))
        warning("plotComparison:mismatch", "%s differs: sim = %g, robot = %g. Re-run the sim with the robot's value.", ...
                name, S.(name), R.(name));
    end
end

function fig = newFigure(w, h)
    % Width 3.5 in = one IEEE column, 7.16 in = full page width (figure*).
    fig = figure("Units", "inches", "Position", [1 1 w h]);
    % Force a light theme so figures stay white on paper even when the
    % MATLAB desktop is in dark mode (R2025a+). Older releases are light anyway.
    if isprop(fig, "Theme"), fig.Theme = "light"; end
    fig.Color = "white";
    set(fig, "DefaultAxesFontSize", 9, "DefaultLegendFontSize", 7, ...
             "DefaultTextInterpreter", "latex");
end

function styleAxes(ax, tEnd, showXLabel, yLabel)
    grid(ax, "on");
    axis(ax, "padded");      % margin so lines at the data limits (e.g. Desired) stay visible
    xlim(ax, [0 tEnd]);      % ...but keep the time axis tight
    ylabel(ax, yLabel, "Interpreter", "latex");
    if showXLabel, xlabel(ax, "Time (s)"); end
end

function saveFig(fig, figDir, name)
    exportgraphics(fig, fullfile(figDir, name + ".pdf"), ...
                   "ContentType", "vector", "BackgroundColor", "white");
end
