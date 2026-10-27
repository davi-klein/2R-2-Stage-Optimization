% ==============================================================================
% File: plot_stage1_trajectory.m
% Description: plots the results for the optimal reference trajectory in Stage 1.
% ==============================================================================
clear; close all; clc;
addpath('src', 'models', 'data');
dir_results = fullfile('results', 'figures');

if exist('trajetoria2R_high_res.mat', 'file')
    traj = load('trajetoria2R_high_res.mat');
else
    traj = load('opt_traj_2R.mat');
end

t = traj.xx;
q1 = traj.position(1, :); q2 = traj.position(2, :);
dq1 = traj.velocity(1, :); dq2 = traj.velocity(2, :);
ddq1 = traj.acceleration(1, :); ddq2 = traj.acceleration(2, :);

x_dummy = [33443.76, 561.98, 9685.47, 3233.95, 55.67, 941.37];
load_system('Robot_2R_Dynamics_and_Control');
set_param('Robot_2R_Dynamics_and_Control', 'FastRestart', 'off');

simIn = Simulink.SimulationInput('Robot_2R_Dynamics_and_Control');
simIn = setVariable(simIn, 'x', x_dummy); simIn = setVariable(simIn, 'xx', traj.xx);
simIn = setVariable(simIn, 'n', length(traj.xx)); simIn = setVariable(simIn, 'stpt', traj.xx(end));
simIn = setVariable(simIn, 'c', 15); 
simIn = setVariable(simIn, 'key_points', [0 -0.2241 1.0001; 0 1.0311 0.9999]);
simIn = setVariable(simIn, 'position', traj.position); 
simIn = setVariable(simIn, 'velocity', traj.velocity);
simIn = setVariable(simIn, 'acceleration', traj.acceleration);
simIn = setModelParameter(simIn, 'SaveOutput', 'on', 'SaveTime', 'on', 'SignalLogging', 'off');
out = sim(simIn);

t_sim = out.tout(:)';
t1_exec = get_vec(out.torque_signal_1); t2_exec = get_vec(out.torque_signal_2);
drt1 = get_vec(out.errtau1); drt2 = get_vec(out.errtau2);
tau_id_1 = t1_exec + drt1; tau_id_2 = t2_exec + drt2;
close_system('Robot_2R_Dynamics_and_Control', 0);

col_j1 = [0.00, 0.25, 0.45]; 
col_j2 = [0.00, 0.60, 0.55];

fig = figure('Color', 'w', 'Units', 'centimeters', 'Position', [5, 2, 8.89, 14.0]);

%Position
ax1 = subplot(4, 1, 1); hold on; grid on; box on;
p1 = plot(t, q1, 'Color', col_j1, 'LineWidth', 1.5);
p2 = plot(t, q2, 'Color', col_j2, 'LineWidth', 1.5);
ylabel('$q$ [rad]', 'Interpreter', 'latex', 'Color', 'k');
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'TickLabelInterpreter', 'latex', 'XTickLabel', []);
lgd = legend([p1, p2], {'Joint 1 ($j=1$)', 'Joint 2 ($j=2$)'}, 'Location', 'southeast', ...
             'Interpreter', 'latex', 'FontSize', 8, 'TextColor', 'k');
set(lgd, 'Color', 'w', 'EdgeColor', 'k');

%Velocity
ax2 = subplot(4, 1, 2); hold on; grid on; box on;
plot(t, dq1, 'Color', col_j1, 'LineWidth', 1.5);
plot(t, dq2, 'Color', col_j2, 'LineWidth', 1.5);
ylabel('$\dot{q}$ [rad/s]', 'Interpreter', 'latex', 'Color', 'k');
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'TickLabelInterpreter', 'latex', 'XTickLabel', []);

%Acceleration
ax3 = subplot(4, 1, 3); hold on; grid on; box on;
plot(t, ddq1, 'Color', col_j1, 'LineWidth', 1.5);
plot(t, ddq2, 'Color', col_j2, 'LineWidth', 1.5);
ylabel('$\ddot{q}$ [rad/s$^2$]', 'Interpreter', 'latex', 'Color', 'k');
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'TickLabelInterpreter', 'latex', 'XTickLabel', []);

%Nominal ID Torque & Saturation
ax4 = subplot(4, 1, 4); hold on; grid on; box on;
plot(t_sim, tau_id_1, 'Color', col_j1, 'LineWidth', 1.5);
plot(t_sim, tau_id_2, 'Color', col_j2, 'LineWidth', 1.5);
yline(25, ':', 'Color', col_j1, 'LineWidth', 1.2); 
yline(-25, ':', 'Color', col_j1, 'LineWidth', 1.2);
yline(9, '--', 'Color', col_j2, 'LineWidth', 1.2); 
yline(-9, '--', 'Color', col_j2, 'LineWidth', 1.2);
ylabel('$\tau_{\mathrm{ID}}$ [N$\cdot$m]', 'Interpreter', 'latex', 'Color', 'k');
xlabel('Time [s]', 'Interpreter', 'latex', 'Color', 'k');
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'TickLabelInterpreter', 'latex');
ylim([-30, 30]);

linkaxes([ax1, ax2, ax3, ax4], 'x'); xlim([0, 3.0]);
exportgraphics(fig, fullfile(dir_results, 'stage1_optimal_trajectory.pdf'), 'ContentType', 'vector', 'BackgroundColor', 'white');

function d = get_vec(sig)
    if isa(sig, 'timeseries'), d = sig.Data(:)'; else, [r, c] = size(sig); if r == 2 && c > 2, d = sig(2, :); elseif c == 2 && r > 2, d = sig(:, 2)'; else, d = sig(:)'; end; end
end