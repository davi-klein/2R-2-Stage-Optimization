% ==============================================================================
% File: compare_cost_functions.m
% Description: Evaluates Position (F_obj,p) vs. DRT (F_obj,t) physical metrics 
%              across 30 PSO runs, computing descriptive statistics and 
%              Wilcoxon rank-sum tests (Mann-Whitney U).
% ==============================================================================

clear; close all; clc;
addpath('src', 'models', 'data');

% 1. Load Stage 1 Trajectory
try
    if exist('trajetoria2R_high_res.mat', 'file')
        traj = load('trajetoria2R_high_res.mat');
    else
        traj = load('opt_traj_2R.mat');
    end
catch
    error('Trajectory file not found in data folder.');
end
dt = traj.xx(2) - traj.xx(1);

% 2. Load Champion Algorithm (PSO) 30-Run Datasets
dir_bench = fullfile('results', 'statistical_benchmark');
data_pos = load(fullfile(dir_bench, 'PSO_Pos_30runs.mat')); 
data_drt = load(fullfile(dir_bench, 'PSO_DRT_30runs.mat'));
num_runs = length(data_pos.history.fval);

% 8 Metrics to evaluate:
% 1: RMSE_pos (rad)
% 2: Max |e| (rad)
% 3: RMS Torque (N.m)
% 4: Control Effort E_tau (N^2.m^2.s)
% 5: Total Variation of Control Torque TV_tau (N.m)
% 6: Total Variation of Residual Torque TV_DRT (N.m)
% 7: Peak |tau_DRT| (N.m)
% 8: Phase Alignment Cross-Correlation R_tau_u (unitless)
num_metrics = 8;
metrics_pos = zeros(num_runs, num_metrics);
metrics_drt = zeros(num_runs, num_metrics);

% 3. Pre-configure Simulink Model with FastRestart
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
fprintf(' SIMULATING 30 PSO RUNS FOR PHYSICAL METRIC EXTRACTION\n');
fprintf('==================================================================\n');
for r = 1:num_runs
    fprintf('Evaluating Run %2d/%2d...\n', r, num_runs);
    metrics_pos(r, :) = extract_metrics(data_pos.history.best_x(r, :), traj, dt, model_name);
    metrics_drt(r, :) = extract_metrics(data_drt.history.best_x(r, :), traj, dt, model_name);
end

set_param(model_name, 'FastRestart', 'off');
close_system(model_name, 0);

% 4. Filter out any NaN or Inf rows AFTER simulation
valid_idx = ~any(isnan(metrics_pos) | isinf(metrics_pos), 2) & ...
            ~any(isnan(metrics_drt) | isinf(metrics_drt), 2);
metrics_pos = metrics_pos(valid_idx, :);
metrics_drt = metrics_drt(valid_idx, :);
fprintf('\nValid paired runs analyzed: %d/%d\n', sum(valid_idx), num_runs);

% 5. Statistical Evaluation & Console Output
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

% Save extracted metrics for quick inspection
save(fullfile(dir_bench, 'physical_metrics_comparison.mat'), 'metrics_pos', 'metrics_drt', 'metric_labels');

% ==============================================================================
% Helper Functions
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
    
    % 1. RMSE Position (rad) - matches Eq. (34)
    rmse_pos = sqrt(mean(e1.^2 + e2.^2));
    
    % 2. Max Position Error (rad)
    max_e = max(max(abs(e1)), max(abs(e2)));
    
    % 3. RMS Torque (N.m)
    rms_torque = sqrt(mean(t1.^2 + t2.^2));
    
    % 4. Control Effort E_tau (N^2.s) - matches Eq. (35)
    effort = sum(t1.^2 + t2.^2) * dt;
    
    % 5. Total Variation of Control Torque TV_tau (N.m) - matches Eq. (36)
    tv_torque = sum(abs(diff(t1))) + sum(abs(diff(t2)));
    
    % 6. Total Variation of Dynamic Residual Torque TV_DRT (N.m)
    tv_drt = sum(abs(diff(drt1))) + sum(abs(diff(drt2)));
    
    % 7. Peak Dynamic Residual Torque (N.m)
    max_drt = max(max(abs(drt1)), max(abs(drt2)));
    
    % 8. Zero-Lag Cross-Correlation R_tau_u - matches Eq. (37)
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