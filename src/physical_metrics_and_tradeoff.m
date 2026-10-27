% ==============================================================================
% File: physical_metrics_and_tradeoff.m
% Description: Evaluates Position (F_obj,p) vs. DRT (F_obj,t) physical metrics 
%              across 30 PSO runs, computing descriptive statistics, 
%              extracting optimal PID gains, performing Wilcoxon 
%              rank-sum tests, and generating the bivariate trade-off map.
% ==============================================================================

clear; close all; clc;
addpath('src', 'models', 'data');

% ------------------------------------------------------------------------------
% LOAD TRAJECTORY & PSO DATA
% ------------------------------------------------------------------------------
try

    traj = load('opt_traj_2R.mat');

catch
    error('Trajectory file not found in data folder.');
end
dt = traj.xx(2) - traj.xx(1);

dir_bench = fullfile('results', 'statistical_benchmark');
dir_fig = fullfile('results', 'figures');
if ~exist(dir_fig, 'dir'), mkdir(dir_fig); end

data_pos = load(fullfile(dir_bench, 'PSO_Pos_30runs.mat')); 
data_drt = load(fullfile(dir_bench, 'PSO_DRT_30runs.mat'));
num_runs = length(data_pos.history.fval);

% Identify indices of absolute best runs for optimal gain extraction
[~, idx_pos] = min(data_pos.history.fval);
[~, idx_drt] = min(data_drt.history.fval);
best_x_pos = data_pos.history.best_x(idx_pos, :);
best_x_drt = data_drt.history.best_x(idx_drt, :);

% Pre-allocate metrics arrays (8 metrics for stats + 2 metrics for bivariate)
num_metrics = 8;
metrics_pos = zeros(num_runs, num_metrics);
metrics_drt = zeros(num_runs, num_metrics);
pts_pso_p = zeros(num_runs, 2); 
pts_pso_d = zeros(num_runs, 2);

% ------------------------------------------------------------------------------
% CONFIGURE SIMULINK & EXTRACT METRICS
% ------------------------------------------------------------------------------
model_name = 'Robot_2R_Dynamics_and_Control';
load_system(model_name);
set_param(model_name, 'FastRestart', 'off');
set_param(model_name, 'SimulationMode', 'normal', ...
                      'SolverType', 'Fixed-step', ...
                      'Solver', 'ode4', ...
                      'FixedStep', '0.001', ...
                      'SaveOutput', 'on', ...
                      'SaveTime', 'on', ...
                      'SignalLogging', 'off');
set_param(model_name, 'FastRestart', 'on');
Simulink.sdi.setAutoArchiveMode(false);
Simulink.sdi.clear;

fprintf('==================================================================\n');
fprintf(' SIMULATING 30 PSO RUNS FOR PHYSICAL METRICS & BIVARIATE MAPPING\n');
fprintf('==================================================================\n');
for r = 1:num_runs
    fprintf('Evaluating Run %2d/%2d...\n', r, num_runs);
    
    % Extract comprehensive metrics
    metrics_pos(r, :) = extract_metrics(data_pos.history.best_x(r, :), traj, dt, model_name);
    metrics_drt(r, :) = extract_metrics(data_drt.history.best_x(r, :), traj, dt, model_name);
    
    % Extract specific bivariate metrics [RMSE_pos, Effort]
    pts_pso_p(r, :) = [metrics_pos(r, 1), metrics_pos(r, 4)];
    pts_pso_d(r, :) = [metrics_drt(r, 1), metrics_drt(r, 4)];
end

set_param(model_name, 'FastRestart', 'off');
close_system(model_name, 0);

% Filter invalid rows
valid_idx = ~any(isnan(metrics_pos) | isinf(metrics_pos), 2) & ...
            ~any(isnan(metrics_drt) | isinf(metrics_drt), 2);
metrics_pos = metrics_pos(valid_idx, :);
metrics_drt = metrics_drt(valid_idx, :);
pts_pso_p = pts_pso_p(valid_idx, :);
pts_pso_d = pts_pso_d(valid_idx, :);

fprintf('\nValid paired runs analyzed: %d/%d\n', sum(valid_idx), num_runs);

% ------------------------------------------------------------------------------
% OPTIMAL PID GAINS & STATISTICAL OUTPUT
% ------------------------------------------------------------------------------
fprintf('\n========================================================================\n');
fprintf(' OPTIMAL PID GAINS (CHAMPION ALGORITHM: PSO)\n');
fprintf('========================================================================\n');
fprintf('Algorithm | Cost Func | kp,1      | kd,1      | ki,1      | kp,2      | kd,2      | ki,2\n');
fprintf('----------------------------------------------------------------------------------------\n');
fprintf('%-9s | %-9s | %9.2f | %9.2f | %9.2f | %9.2f | %9.2f | %9.2f\n', 'PSO', 'F_obj,p', best_x_pos(1), best_x_pos(2), best_x_pos(3), best_x_pos(4), best_x_pos(5), best_x_pos(6));
fprintf('%-9s | %-9s | %9.2f | %9.2f | %9.2f | %9.2f | %9.2f | %9.2f\n', 'PSO', 'F_obj,t', best_x_drt(1), best_x_drt(2), best_x_drt(3), best_x_drt(4), best_x_drt(5), best_x_drt(6));
fprintf('========================================================================\n');

metric_labels = {
    '1. Tracking RMSE (rad)', ...
    '2. Max Position Error (rad)', ...
    '3. RMS Torque (N.m)', ...
    '4. Control Effort E_tau (N^2.s)', ...
    '5. Torque Total Var. TV_tau (N.m)', ...
    '6. DRT Total Var. TV_DRT (N.m)', ...
    '7. Max |tau_DRT| (N.m)', ...
    '8. Phase Alignment R_tau_u'
};

alpha = 0.05;
fprintf('\n========================================================================================================================\n');
fprintf('%-34s | %-25s | %-25s | %-12s | %-12s\n', ...
    'Physical Metric', 'PSO F_obj,p (Mean +- Std)', 'PSO F_obj,t (Mean +- Std)', 'p-value', 'Result');
fprintf('------------------------------------------------------------------------------------------------------------------------\n');

for m = 1:num_metrics
    vec_p = metrics_pos(:, m);
    vec_t = metrics_drt(:, m);
    
    mean_p = mean(vec_p); std_p = std(vec_p);
    mean_t = mean(vec_t); std_t = std(vec_t);
    
    [p_val, h_val] = ranksum(vec_p, vec_t, 'Alpha', alpha);
    
    if isnan(p_val)
        res_str = 'Identical';
    elseif h_val == 1
        res_str = 'Significant';
    else
        res_str = 'Not Sig.';
    end
    
    fprintf('%-34s | %.6e +- %.2e | %.6e +- %.2e | %-12.4e | %-12s\n', ...
        metric_labels{m}, mean_p, std_p, mean_t, std_t, p_val, res_str);
end
fprintf('========================================================================================================================\n');

save(fullfile(dir_bench, 'physical_metrics_comparison.mat'), 'metrics_pos', 'metrics_drt', 'metric_labels');

% ------------------------------------------------------------------------------
% BIVARIATE MAPPING VISUALIZATION
% ------------------------------------------------------------------------------
scale = 1e4;
all_pts = [pts_pso_p; pts_pso_d];
all_pts(:, 1) = all_pts(:, 1) * scale; 
num_pts = size(all_pts, 1);
is_dominated = false(num_pts, 1);

for i = 1:num_pts
    for j = 1:num_pts
        if i ~= j
            if (all_pts(j, 1) <= all_pts(i, 1) && all_pts(j, 2) <= all_pts(i, 2)) && ...
               (all_pts(j, 1) < all_pts(i, 1) || all_pts(j, 2) < all_pts(i, 2))
                is_dominated(i) = true;
                break;
            end
        end
    end
end
idx = find(~is_dominated);
pts = all_pts(idx, :);
[~, sort_order] = sort(pts(:, 1));
pts = pts(sort_order, :);

p_p = pts_pso_p; p_p(:, 1) = p_p(:, 1) * scale; 
p_d = pts_pso_d; p_d(:, 1) = p_d(:, 1) * scale; 
col_pso = [0.85, 0.33, 0.10]; 

fprintf('\nGenerating PSO bivariate mapping plot...\n');
f_zoom = figure('Color', 'w', 'Units', 'centimeters', 'Position', [16, 5, 10, 8.5]);
hold on; box on; grid on;

scatter(p_p(:, 1), p_p(:, 2), 35, col_pso, 'o', 'filled', 'MarkerFaceAlpha', 0.9, 'DisplayName', 'PSO (Position)');
scatter(p_d(:, 1), p_d(:, 2), 35, col_pso, '^', 'filled', 'MarkerFaceAlpha', 0.9, 'DisplayName', 'PSO (DRT)');
plot(pts(:, 1), pts(:, 2), 'k--', 'LineWidth', 1.2, 'DisplayName', 'Bivariate Frontier');
scatter(pts(:, 1), pts(:, 2), 50, 'k', 'o', 'LineWidth', 1.2, 'DisplayName', 'Non-Dominated');

set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'TickLabelInterpreter', 'latex', 'FontSize', 9);
xlabel('Tracking Precision ($\mathrm{RMSE}_{\mathrm{pos}} \times 10^4$) [rad]', 'Interpreter', 'latex', 'FontSize', 10, 'Color', 'k');
ylabel('Actuator Control Effort $E_\tau$ [$\mathrm{N}^2\cdot\mathrm{s}$]', 'Interpreter', 'latex', 'FontSize', 10, 'Color', 'k');
title('Bivariate performance map', 'Interpreter', 'latex', 'FontSize', 10, 'Color', 'k');

x_min_z = min(all_pts(:,1)); x_max_z = max(all_pts(:,1));
y_min_z = min(all_pts(:,2)); y_max_z = max(all_pts(:,2));
x_range = max(1e-5, x_max_z - x_min_z); 
y_range = max(1e-5, y_max_z - y_min_z);

x_pad = max(0.02, x_range * 0.15);
y_pad_bot = max(0.02, y_range * 0.15);
y_pad_top = max(0.05, y_range * 0.4); 

xlim([x_min_z - x_pad, x_max_z + x_pad]);
ylim([y_min_z - y_pad_bot, y_max_z + y_pad_top]);

lgd2 = legend('Location', 'north', 'NumColumns', 2, 'Interpreter', 'latex', 'FontSize', 7, 'TextColor', 'k', 'Box', 'on');
set(lgd2, 'Color', 'w', 'EdgeColor', 'k');
set(gca, 'Position', [0.13, 0.13, 0.82, 0.78]);

exportgraphics(f_zoom, fullfile(dir_fig, 'bivariate_front_tradeoff_pso_zoomed.pdf'), 'ContentType', 'vector', 'BackgroundColor', 'white');
fprintf('[BIVARIATE MAPPING SAVED SUCCESSFULLY]\n');

% ==============================================================================
% HELPER FUNCTIONS
% ==============================================================================
function m = extract_metrics(x, traj, dt, model_name)
    simIn = Simulink.SimulationInput(model_name);
    simIn = setVariable(simIn, 'x', x);
    simIn = setVariable(simIn, 'xx', traj.xx);
    simIn = setVariable(simIn, 'n', length(traj.xx));
    simIn = setVariable(simIn, 'stpt', traj.xx(end));
    simIn = setVariable(simIn, 'c', 15);
    simIn = setVariable(simIn, 'key_points', [0.0000 -0.2241 1.0001; 0 1.0311 0.9999]);
    simIn = setVariable(simIn, 'position', traj.position);
    simIn = setVariable(simIn, 'velocity', traj.velocity);
    simIn = setVariable(simIn, 'acceleration', traj.acceleration);
    
    out = sim(simIn);
    
    p1 = get_sim_data(out.pos_signal_1);
    p2 = get_sim_data(out.pos_signal_2);
    t1 = get_sim_data(out.torque_signal_1);
    t2 = get_sim_data(out.torque_signal_2);
    drt1 = get_sim_data(out.errtau1);
    drt2 = get_sim_data(out.errtau2);
    
    N_pts = length(t1);
    ref1 = traj.position(1, 1:N_pts);
    ref2 = traj.position(2, 1:N_pts);
    
    e1 = ref1(:)' - p1(:)';
    e2 = ref2(:)' - p2(:)';
    
    rmse_pos = sqrt(mean(e1.^2 + e2.^2));
    max_e = max(max(abs(e1)), max(abs(e2)));
    rms_torque = sqrt(mean(t1.^2 + t2.^2));
    effort = sum(t1.^2 + t2.^2) * dt;
    tv_torque = sum(abs(diff(t1))) + sum(abs(diff(t2)));
    tv_drt = sum(abs(diff(drt1))) + sum(abs(diff(drt2)));
    max_drt = max(max(abs(drt1)), max(abs(drt2)));
    
    tau_id_1 = t1 + drt1;
    tau_id_2 = t2 + drt2;
    R1 = sum(t1 .* tau_id_1) / sqrt(sum(t1.^2) * sum(tau_id_1.^2));
    R2 = sum(t2 .* tau_id_2) / sqrt(sum(t2.^2) * sum(tau_id_2.^2));
    phase_align = mean([R1, R2]);
    
    m = [rmse_pos, max_e, rms_torque, effort, tv_torque, tv_drt, max_drt, phase_align];
end

function data = get_sim_data(signal)
    if isa(signal, 'timeseries')
        data = signal.Data(:)';
    else
        [r, c] = size(signal);
        if r == 2 && c > 2
            data = signal(2, :);
        elseif c == 2 && r > 2
            data = signal(:, 2)';
        else
            data = signal(:)';
        end
    end
end