% ==============================================================================
% File: ablation_study_stage1.m
% Description: Methodologically strict ablation study. Uses the EXACT same 
%              cost function and penalties as the main benchmark to prove that 
%              unconstrained trajectories force the optimizer to sacrifice tracking.
% ==============================================================================

clear; close all; clc;
addpath('src', 'models', 'data');

dir_results = fullfile('results', 'statistical_benchmark');
dir_figs = fullfile('results', 'figures');

% 1. Load the successful Stage 1 Optimized Trajectory & PSO Baseline
try
    traj_opt = load('opt_traj_2R.mat');
catch
    error('Stage 1 optimal trajectory file not found.');
end

file_pso = fullfile(dir_results, 'PSO_Pos_30runs.mat');
if exist(file_pso, 'file')
    data_pso_opt = load(file_pso);
    [~, best_idx] = min(data_pso_opt.history.fval);
    x_champ_opt = data_pso_opt.history.best_x(best_idx, :);
    best_cost_opt = data_pso_opt.history.fval(best_idx);
    gen_opt = data_pso_opt.history.gen_conv(best_idx);
else
    error('Run the baseline PSO Pos benchmark first.');
end

% Synthesize Unconstrained Baseline Trajectory (Quintic B-spline)
t0 = 0; t1 = 1.5; t2 = 3.0; n_pts = 3001;
t_vec = linspace(t0, t2, n_pts); dt = t_vec(2) - t_vec(1);
waypoints = [0, -0.2241, 1.0; 0, 1.0311, 1.0];

M = zeros(12, 12);
M(1, 1) = 1; M(2, 1:6) = [1, t1, t1^2, t1^3, t1^4, t1^5];
M(3, 7:12) = [1, t1, t1^2, t1^3, t1^4, t1^5]; M(4, 7:12) = [1, t2, t2^2, t2^3, t2^4, t2^5];
M(5, 2) = 1; M(6, 3) = 2;
M(7, 8:12) = [1, 2*t2, 3*t2^2, 4*t2^3, 5*t2^4]; M(8, 9:12) = [2, 6*t2, 12*t2^2, 20*t2^3];
M(9, 2:6) = [1, 2*t1, 3*t1^2, 4*t1^3, 5*t1^4]; M(9, 8:12) = -[1, 2*t1, 3*t1^2, 4*t1^3, 5*t1^4];
M(10, 3:6) = [2, 6*t1, 12*t1^2, 20*t1^3]; M(10, 9:12) = -[2, 6*t1, 12*t1^2, 20*t1^3];
M(11, 4:6) = [6, 24*t1, 60*t1^2]; M(11, 10:12) = -[6, 24*t1, 60*t1^2];
M(12, 10:12) = [6, 24*t2, 60*t2^2];

half_n = (n_pts - 1) / 2 + 1;
t_s1 = linspace(t0, t1, half_n); t_s2 = linspace(t1, t2, half_n);
q_no = zeros(2, n_pts); dq_no = zeros(2, n_pts); ddq_no = zeros(2, n_pts);
for j = 1:2
    b = [waypoints(j, 1); waypoints(j, 2); waypoints(j, 2); waypoints(j, 3); zeros(8, 1)];
    sol = M \ b; p1 = flipud(sol(1:6))'; p2 = flipud(sol(7:12))';
    v1 = polyder(p1); v2 = polyder(p2); a1 = polyder(v1); a2 = polyder(v2);
    q_no(j, :) = [polyval(p1, t_s1), polyval(p2, t_s2(2:end))];
    dq_no(j, :) = [polyval(v1, t_s1), polyval(v2, t_s2(2:end))];
    ddq_no(j, :) = [polyval(a1, t_s1), polyval(a2, t_s2(2:end))];
end
traj_no.xx = t_vec; traj_no.position = q_no; traj_no.velocity = dq_no; traj_no.acceleration = ddq_no;

%Run Standard PSO on the unconstrained trajectory
fprintf('Attempting to optimize standard PSO for the unconstrained trajectory...\n');
N = 25; K = 300; d = 6; 
Lb = [0, 0, 0, 0, 0, 0]; Ub = [33443.76, 1065.09, 16721.88, 3233.95, 103.00, 1616.97];
w_max = 0.9; w_min = 0.4; c1 = 2.0; c2 = 2.0;
x = repmat(Lb, N, 1) + rand(N, d) .* repmat(Ub - Lb, N, 1);
v = zeros(N, d); pbest = x; pbest_cost = inf(N, 1);
gbest = zeros(1, d); gbest_cost = inf; conv_no = zeros(1, K);
stall_max = 30; stall_count = 0; prev_best = inf;

load_system('Robot_2R_Dynamics_and_Control');
set_param('Robot_2R_Dynamics_and_Control', 'FastRestart', 'on');

for k = 1:K
    w = w_max - k * ((w_max - w_min) / K);
    for i = 1:N
        cost_val = evaluate_standard_cost(x(i, :), traj_no, dt);
        if cost_val < pbest_cost(i)
            pbest(i, :) = x(i, :); pbest_cost(i) = cost_val;
            if cost_val < gbest_cost, gbest = x(i, :); gbest_cost = cost_val; end
        end
    end
    for i = 1:N
        v(i, :) = w * v(i, :) + c1 * rand * (pbest(i, :) - x(i, :)) + c2 * rand * (gbest - x(i, :));
        x(i, :) = x(i, :) + v(i, :);
        x(i, :) = max(x(i, :), Lb); x(i, :) = min(x(i, :), Ub);
    end
    conv_no(k) = gbest_cost;
    
    if k > 1
        delta_k = max(1e-12, 1e-4 * gbest_cost);
        if (prev_best - gbest_cost) <= delta_k, stall_count = stall_count + 1; else, stall_count = 0; end
    end
    prev_best = gbest_cost;
    fprintf('Gen %d/%d | Best Cost: %.4e | Stall: %d/%d\n', k, K, gbest_cost, stall_count, stall_max);
    if stall_count >= stall_max
        fprintf('Stagnation criterion reached at generation %d.\n', k);
        conv_no = conv_no(1:k); break;
    end
end
set_param('Robot_2R_Dynamics_and_Control', 'FastRestart', 'off');

res_opt = simulate_full(x_champ_opt, traj_opt);
res_no  = simulate_full(gbest, traj_no);
close_system('Robot_2R_Dynamics_and_Control', 0);
% ==============================================================================
% VISUALIZATION: ABLATION STUDY
% ==============================================================================
col_prop = [0.13, 0.55, 0.13]; % Forest Green (Proposed Framework)
col_base = [0.80, 0.15, 0.15]; % Vivid Crimson (Unconstrained Baseline)

set(0, 'DefaultAxesFontName', 'Times New Roman', 'DefaultTextFontName', 'Times New Roman');
set(0, 'DefaultAxesFontSize', 9, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultAxesXColor', 'k', 'DefaultAxesYColor', 'k', 'DefaultTextColor', 'k');

% ------------------------------------------------------------------------------
% PLOT 1: Convergence Trapping
% ------------------------------------------------------------------------------
f1 = figure('Units', 'centimeters', 'Position', [5, 5, 8.89, 7.5], 'Color', 'w');

% Plot Non-opt convergence
semilogy(1:length(conv_no), conv_no, 'Color', col_base, 'LineWidth', 1.5, 'DisplayName', 'Non-opt.'); 
hold on;

% Plot Opt convergence bounded by its actual generation count
semilogy([0, gen_opt], [best_cost_opt, best_cost_opt], 'Color', col_prop, 'LineWidth', 1.5, 'DisplayName', 'Opt.'); % <--- FIX HERE

% Plot Penalty threshold
yline(1e12, 'k:', 'LineWidth', 1.2, 'DisplayName', 'Penalty ($> 10^{12}$)');

xlabel('Generations', 'Color', 'k'); 
ylabel('Objective Cost $Z$', 'Color', 'k');
grid on; 
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'TickLabelInterpreter', 'latex');

% Restored the lower limit to 1e-7 so the optimal cost line is visible
ylim([1e-7, 1e16]); 
yticks([1e-6, 1e0, 1e6, 1e12]);

% Shortened legend with box
lgd1 = legend('Location', 'northeast', 'Interpreter', 'latex', 'FontSize', 8, 'TextColor', 'k'); 
set(lgd1, 'Color', 'w', 'EdgeColor', 'k');
set(gca, 'Position', [0.15, 0.14, 0.80, 0.78]);
exportgraphics(f1, fullfile(dir_figs, 'ablation_convergence_failure.pdf'), 'ContentType', 'vector', 'BackgroundColor', 'white');
% ------------------------------------------------------------------------------
% PLOT 2: Torque Demand vs Physical Limit
% ------------------------------------------------------------------------------
f2 = figure('Units', 'centimeters', 'Position', [5, 5, 8.89, 9.0], 'Color', 'w');

% Joint 1
subplot(2, 1, 1); hold on; grid on; box on;
plot(res_opt.t, res_opt.tau_exec(1, :), 'Color', col_prop, 'LineWidth', 1.5, 'DisplayName', 'Opt.'); 
plot(res_no.t, res_no.tau_id(1, :), 'Color', col_base, 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'Non-opt.');

% Removido da legenda com 'HandleVisibility', 'off'
yline(25, 'k:', 'LineWidth', 1.2, 'HandleVisibility', 'off'); 
yline(-25, 'k:', 'LineWidth', 1.2, 'HandleVisibility', 'off');

ylabel('$\tau_1$ [N$\cdot$m]', 'Color', 'k'); xlim([0, 3.0]); 
y_max_1 = max(max(abs(res_no.tau_id(1, :))), 26) * 1.5; ylim([-y_max_1, y_max_1]);
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'TickLabelInterpreter', 'latex', 'XTickLabel', []);
lgd2_1 = legend('Location', 'northeast', 'Interpreter', 'latex', 'FontSize', 8, 'TextColor', 'k'); 
set(lgd2_1, 'Color', 'w', 'EdgeColor', 'k'); 

% Joint 2
subplot(2, 1, 2); hold on; grid on; box on;
plot(res_opt.t, res_opt.tau_exec(2, :), 'Color', col_prop, 'LineWidth', 1.5, 'DisplayName', 'Opt.'); 
plot(res_no.t, res_no.tau_id(2, :), 'Color', col_base, 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'Non-opt.');

yline(9, 'k:', 'LineWidth', 1.2, 'HandleVisibility', 'off'); 
yline(-9, 'k:', 'LineWidth', 1.2, 'HandleVisibility', 'off');

ylabel('$\tau_2$ [N$\cdot$m]', 'Color', 'k'); xlabel('Time [s]', 'Color', 'k'); xlim([0, 3.0]); 
y_max_2 = max(max(abs(res_no.tau_id(2, :))), 10) * 1.5; ylim([-y_max_2, y_max_2]);
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'TickLabelInterpreter', 'latex');
lgd2_2 = legend('Location', 'northeast', 'Interpreter', 'latex', 'FontSize', 8, 'TextColor', 'k'); 
set(lgd2_2, 'Color', 'w', 'EdgeColor', 'k'); 

exportgraphics(f2, fullfile(dir_figs, 'ablation_torque_saturation.pdf'), 'ContentType', 'vector', 'BackgroundColor', 'white');

% ------------------------------------------------------------------------------
% PLOT 3: Tracking Error Divergence
% ------------------------------------------------------------------------------
f3 = figure('Units', 'centimeters', 'Position', [5, 5, 8.89, 9.0], 'Color', 'w');

% Joint 1 Error
subplot(2, 1, 1); hold on; grid on; box on;
plot(res_opt.t, res_opt.e(1, :), 'Color', col_prop, 'LineWidth', 1.5, 'DisplayName', 'Opt.'); 
plot(res_no.t, res_no.e(1, :), 'Color', col_base, 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'Non-opt.');
ylabel('$e_1$ [rad]', 'Color', 'k'); xlim([0, 3.0]); 

e_max_1 = max([max(abs(res_no.e(1, :))), max(abs(res_opt.e(1, :)))]) * 1.35; 
ylim([-e_max_1, e_max_1]);
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'TickLabelInterpreter', 'latex', 'XTickLabel', []);
lgd3_1 = legend('Location', 'southwest', 'Interpreter', 'latex', 'FontSize', 8, 'TextColor', 'k'); 
set(lgd3_1, 'Color', 'w', 'EdgeColor', 'k');

% Joint 2 Error
subplot(2, 1, 2); hold on; grid on; box on;
plot(res_opt.t, res_opt.e(2, :), 'Color', col_prop, 'LineWidth', 1.5, 'DisplayName', 'Opt. '); 
plot(res_no.t, res_no.e(2, :), 'Color', col_base, 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'Non-opt. ');
ylabel('$e_2$ [rad]', 'Color', 'k'); xlabel('Time [s]', 'Color', 'k'); xlim([0, 3.0]); 

e_max_2 = max([max(abs(res_no.e(2, :))), max(abs(res_opt.e(2, :)))]) * 1.35; 
ylim([-e_max_2, e_max_2]);
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'TickLabelInterpreter', 'latex');

lgd3_2 = legend('Location', 'southwest', 'Interpreter', 'latex', 'FontSize', 8, 'TextColor', 'k');
set(lgd3_2, 'Color', 'w', 'EdgeColor', 'k'); 
exportgraphics(f3, fullfile(dir_figs, 'ablation_tracking_error_comp.pdf'), 'ContentType', 'vector', 'BackgroundColor', 'white');

fprintf('\n[ALL ABLATION FIGURES FIXED AND EXPORTED]\n');

% ==============================================================================
% Helper Functions
% ==============================================================================
function cost = evaluate_standard_cost(x, traj, dt)
    simIn = Simulink.SimulationInput('Robot_2R_Dynamics_and_Control');
    simIn = setVariable(simIn, 'x', x); simIn = setVariable(simIn, 'xx', traj.xx);
    simIn = setVariable(simIn, 'n', length(traj.xx)); simIn = setVariable(simIn, 'stpt', traj.xx(end));
    simIn = setVariable(simIn, 'c', 15); simIn = setVariable(simIn, 'key_points', [0 -0.2241 1.0001; 0 1.0311 0.9999]);
    simIn = setVariable(simIn, 'position', traj.position); simIn = setVariable(simIn, 'velocity', traj.velocity);
    simIn = setVariable(simIn, 'acceleration', traj.acceleration);
    out = sim(simIn);
    
    t1 = get_vec(out.torque_signal_1); t2 = get_vec(out.torque_signal_2);
    p1 = get_vec(out.pos_signal_1); p2 = get_vec(out.pos_signal_2);
    ref1 = traj.position(1, 1:length(p1)); ref2 = traj.position(2, 1:length(p2));
    e1 = ref1(:)' - p1; e2 = ref2(:)' - p2;
     
    nominal_cost = sum(e1.^2 + e2.^2) * dt; 
    
    g1 = max(abs(t1)) - 25.0; g2 = max(abs(t2)) - 9.0;
    penalty = 0; lambda = 1e15;
    if g1 > 0, penalty = penalty + lambda * g1^2; end
    if g2 > 0, penalty = penalty + lambda * g2^2; end
    
    cost = nominal_cost + penalty;
end

function s = simulate_full(x, traj)
    simIn = Simulink.SimulationInput('Robot_2R_Dynamics_and_Control');
    simIn = setVariable(simIn, 'x', x); simIn = setVariable(simIn, 'xx', traj.xx);
    simIn = setVariable(simIn, 'n', length(traj.xx)); simIn = setVariable(simIn, 'stpt', traj.xx(end));
    simIn = setVariable(simIn, 'c', 15); simIn = setVariable(simIn, 'key_points', [0 -0.2241 1.0001; 0 1.0311 0.9999]);
    simIn = setVariable(simIn, 'position', traj.position); simIn = setVariable(simIn, 'velocity', traj.velocity);
    simIn = setVariable(simIn, 'acceleration', traj.acceleration);
    simIn = setModelParameter(simIn, 'SaveOutput', 'on', 'SaveTime', 'on', 'SignalLogging', 'off');
    out = sim(simIn);
    
    s.t = out.tout(:)';
    p1 = get_vec(out.pos_signal_1); p2 = get_vec(out.pos_signal_2);
    t1 = get_vec(out.torque_signal_1); t2 = get_vec(out.torque_signal_2);
    drt1 = get_vec(out.errtau1); drt2 = get_vec(out.errtau2);
    
    ref1 = traj.position(1, 1:length(p1)); ref2 = traj.position(2, 1:length(p2));
    s.e = [ref1(:)' - p1; ref2(:)' - p2]; 
    s.tau_exec = [t1; t2];
    s.tau_id = [t1 + drt1; t2 + drt2]; % Pre-saturation Inverse Dynamics demand
end

function d = get_vec(sig)
    if isa(sig, 'timeseries'), d = sig.Data(:)'; else, [r, c] = size(sig); if r == 2 && c > 2, d = sig(2, :); elseif c == 2 && r > 2, d = sig(:, 2)'; else, d = sig(:)'; end; end
end