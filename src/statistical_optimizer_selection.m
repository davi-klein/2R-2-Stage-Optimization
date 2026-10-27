% ==============================================================================
% File: statistical_optimizer_selection.m
% Description: Consolidated script for the "Statistical Selection of Most 
%              Suitable Optimizer" methodology. Merges dispersion boxplots, 
%              computational effort bar charts, descriptive statistics, 
%              and Wilcoxon rank-sum tests with Bonferroni-Holm corrections.
% ==============================================================================
clear; close all; clc;

dir_bench = fullfile('results', 'statistical_benchmark');
dir_fig = fullfile('results', 'figures');
% ------------------------------------------------------------------------------
% LOAD DATA
% ------------------------------------------------------------------------------
data_fp = load(fullfile(dir_bench, 'FMA_Pos_30runs.mat'));
data_dp = load(fullfile(dir_bench, 'DOA_Pos_30runs.mat'));
data_pp = load(fullfile(dir_bench, 'PSO_Pos_30runs.mat'));

data_fd = load(fullfile(dir_bench, 'FMA_DRT_30runs.mat'));
data_dd = load(fullfile(dir_bench, 'DOA_DRT_30runs.mat'));
data_pd = load(fullfile(dir_bench, 'PSO_DRT_30runs.mat'));

% Extract objective costs
pos_fval = [data_fp.history.fval, data_dp.history.fval, data_pp.history.fval];
drt_fval = [data_fd.history.fval, data_dd.history.fval, data_pd.history.fval];

% Extract Best X Arrays (30 runs x 6 variables)
fp_best_x = data_fp.history.best_x;
dp_best_x = data_dp.history.best_x;
pp_best_x = data_pp.history.best_x;
fd_best_x = data_fd.history.best_x;
dd_best_x = data_dd.history.best_x;
pd_best_x = data_pd.history.best_x;

% Extract computational effort (num_eval)
num_eval_pos = [data_fp.history.num_eval, data_dp.history.num_eval, data_pp.history.num_eval];
num_eval_drt = [data_fd.history.num_eval, data_dd.history.num_eval, data_pd.history.num_eval];

% Find the indices of the absolute best runs (minimum cost)
[~, idx_fp] = min(data_fp.history.fval);
[~, idx_dp] = min(data_dp.history.fval);
[~, idx_pp] = min(data_pp.history.fval);
[~, idx_fd] = min(data_fd.history.fval);
[~, idx_dd] = min(data_dd.history.fval);
[~, idx_pd] = min(data_pd.history.fval);

% --- CUSTOM IEEE COLOR PALETTE ---
col_fma = [0.49, 0.18, 0.56];
col_doa = [0.00, 0.45, 0.74];
col_pso = [0.85, 0.33, 0.10]; 
colors = [col_fma; col_doa; col_pso];

% Formatting setup
set(0, 'DefaultAxesFontName', 'Times New Roman', 'DefaultTextFontName', 'Times New Roman');
set(0, 'DefaultAxesFontSize', 9, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultAxesXColor', 'k', 'DefaultAxesYColor', 'k', 'DefaultTextColor', 'k');

% ------------------------------------------------------------------------------
% VISUALIZATION: ALGORITHM DISPERSION
% ------------------------------------------------------------------------------
f1 = figure('Units', 'centimeters', 'Position', [5, 5, 8.89, 11.0], 'Color', 'w');

% Subplot 1: Position Error Formulation
subplot(2, 1, 1);
boxplot(pos_fval, 'Labels', {'FMA', 'DOA', 'PSO'}, 'Colors', colors, 'Symbol', '+');
hold on; 
set(findobj(gca,'type','line'), 'LineWidth', 1.5);
set(findobj(gca,'Tag','Median'), 'Color', 'k');
outliers = findobj(gca, 'Tag', 'Outliers');
if length(outliers) == 3
    set(outliers(1), 'MarkerEdgeColor', col_pso); 
    set(outliers(2), 'MarkerEdgeColor', col_doa);
    set(outliers(3), 'MarkerEdgeColor', col_fma);
end
ylim([1.45e-6, 1.48e-6]);
ylabel('Cost Value ($F_{obj,p}$)', 'Interpreter', 'latex', 'Color', 'k');
title('Position Error Formulation', 'Interpreter', 'latex', 'FontSize', 9, 'Color', 'k');
grid on; box on;
set(gca, 'Color', 'w', 'TickLabelInterpreter', 'latex', 'GridColor', 'k', 'GridAlpha', 0.2);

h1 = plot(NaN,NaN,'Color',col_fma,'LineWidth',1.5);
h2 = plot(NaN,NaN,'Color',col_doa,'LineWidth',1.5);
h3 = plot(NaN,NaN,'Color',col_pso,'LineWidth',1.5);
lgd = legend([h1, h2, h3], {'FMA', 'DOA', 'PSO'}, 'Location', 'northeast', 'Interpreter', 'latex', 'FontSize', 8, 'TextColor', 'k');
set(lgd, 'Color', 'w', 'EdgeColor', 'k');

% Subplot 2: DRT Formulation
subplot(2, 1, 2);
boxplot(drt_fval, 'Labels', {'FMA', 'DOA', 'PSO'}, 'Colors', colors, 'Symbol', '+');
hold on;
set(findobj(gca,'type','line'), 'LineWidth', 1.5);
set(findobj(gca,'Tag','Median'), 'Color', 'k');
outliers = findobj(gca, 'Tag', 'Outliers');
if length(outliers) == 3
    set(outliers(1), 'MarkerEdgeColor', col_pso);
    set(outliers(2), 'MarkerEdgeColor', col_doa);
    set(outliers(3), 'MarkerEdgeColor', col_fma);
end
ylim([0.0165, 0.01725]);
ylabel('Cost Value ($F_{obj,t}$)', 'Interpreter', 'latex', 'Color', 'k');
title('Dynamic Residual Torque Formulation', 'Interpreter', 'latex', 'FontSize', 9, 'Color', 'k');
grid on; box on;
set(gca, 'Color', 'w', 'TickLabelInterpreter', 'latex', 'GridColor', 'k', 'GridAlpha', 0.2);

exportgraphics(f1, fullfile(dir_fig, 'algorithm_dispersion_combined.pdf'), 'ContentType', 'vector', 'BackgroundColor', 'white');
fprintf('\n[STYLED COMBINED BOXPLOT EXPORTED SUCCESSFULLY]\n');

% ------------------------------------------------------------------------------
% VISUALIZATION: COMPUTATIONAL EFFORT
% ------------------------------------------------------------------------------
f2 = figure('Color', 'w', 'Units', 'centimeters', 'Position', [15, 5, 8.89, 7.5]);
bar_data = [mean(num_eval_pos(:,1)), mean(num_eval_drt(:,1)); 
            mean(num_eval_pos(:,2)), mean(num_eval_drt(:,2)); 
            mean(num_eval_pos(:,3)), mean(num_eval_drt(:,3))];
b = bar(bar_data, 'grouped');

b(1).FaceColor = [0.40, 0.40, 0.40]; % Slate Grey (Position Formulation)
b(2).FaceColor = [0.93, 0.69, 0.13]; % Gold (DRT Formulation)

set(gca, 'XTickLabel', {'FMA', 'DOA', 'PSO'}, 'TickLabelInterpreter', 'latex', ...
         'FontSize', 9, 'Color', 'w', 'XColor', 'k', 'YColor', 'k');
ylabel('Objective Evaluations ($\mathrm{NumEval}$)', 'Interpreter', 'latex', 'FontSize', 10, 'Color', 'k');
title('Computational Effort by Algorithm', 'Interpreter', 'latex', 'FontSize', 10, 'Color', 'k');

lgd2 = legend({'Cost: Position', 'Cost: DRT'}, 'Interpreter', 'latex', 'Location', 'northeast', 'FontSize', 8, 'TextColor', 'k');
set(lgd2, 'Color', 'w', 'EdgeColor', 'k');
grid on;
set(gca, 'Position', [0.15, 0.13, 0.80, 0.78]);

exportgraphics(f2, fullfile(dir_fig, 'computational_effort_numeval.pdf'), 'ContentType', 'vector', 'BackgroundColor', 'white');
fprintf('[BAR CHART REFACTORED AND SAVED SUCCESSFULLY]\n');

% ------------------------------------------------------------------------------
% DESCRIPTIVE STATISTICS
% ------------------------------------------------------------------------------
alg_names = {'FMA', 'DOA', 'PSO'};
fprintf('\n========================================================================\n');
fprintf(' DESCRIPTIVE STATISTICS (30 RUNS)\n');
fprintf('========================================================================\n');
fprintf('\n--- Position Error Formulation (F_obj,p) ---\n');
fprintf('%-10s %-15s %-15s %-15s %-15s %-15s\n', 'Algorithm', 'Best (Min)', 'Worst (Max)', 'Mean', 'Median', 'Std. Dev.');
for i = 1:3
    fprintf('%-10s %-15.6e %-15.6e %-15.6e %-15.6e %-15.6e\n', alg_names{i}, ...
        min(pos_fval(:,i)), max(pos_fval(:,i)), mean(pos_fval(:,i)), median(pos_fval(:,i)), std(pos_fval(:,i)));
end

fprintf('\n--- Dynamic Residual Torque Formulation (F_obj,t) ---\n');
fprintf('%-10s %-15s %-15s %-15s %-15s %-15s\n', 'Algorithm', 'Best (Min)', 'Worst (Max)', 'Mean', 'Median', 'Std. Dev.');
for i = 1:3
    fprintf('%-10s %-15.6e %-15.6e %-15.6e %-15.6e %-15.6e\n', alg_names{i}, ...
        min(drt_fval(:,i)), max(drt_fval(:,i)), mean(drt_fval(:,i)), median(drt_fval(:,i)), std(drt_fval(:,i)));
end

fprintf('\n--- Computational Effort: Position Formulation (NumEval) ---\n');
fprintf('%-10s %-15s %-15s\n', 'Algorithm', 'Mean', 'Std. Dev.');
for i = 1:3
    fprintf('%-10s %-15.2f %-15.2f\n', alg_names{i}, mean(num_eval_pos(:,i)), std(num_eval_pos(:,i)));
end

fprintf('\n--- Computational Effort: DRT Formulation (NumEval) ---\n');
fprintf('%-10s %-15s %-15s\n', 'Algorithm', 'Mean', 'Std. Dev.');
for i = 1:3
    fprintf('%-10s %-15.2f %-15.2f\n', alg_names{i}, mean(num_eval_drt(:,i)), std(num_eval_drt(:,i)));
end


% ------------------------------------------------------------------------------
% WILCOXON RANK-SUM TESTS (MANN-WHITNEY U)
% ------------------------------------------------------------------------------
alpha = 0.05;
pairs = {
    3, 1, 'PSO vs. FMA';
    3, 2, 'PSO vs. DOA';
    2, 1, 'DOA vs. FMA'
};

fprintf('\n=================================================================================\n');
fprintf(' WILCOXON RANK-SUM TEST RESULTS WITH BONFERRONI-HOLM CORRECTION (alpha = %.2f)\n', alpha);
fprintf('=================================================================================\n');

% Cost (F_obj,p)
fprintf('\n--- Position Formulation: Cost (F_obj,p) ---\n');
fprintf('%-15s %-15s %-15s %-15s %-10s\n', 'Comparison', 'Raw p-value', 'Adj p-value', 'Decision (h)', 'Result');
p_raw = zeros(size(pairs, 1), 1);
for k = 1:size(pairs, 1)
    p_raw(k) = ranksum(pos_fval(:,pairs{k, 1}), pos_fval(:,pairs{k, 2}), 'Alpha', alpha);
end
[p_adj, h_adj] = apply_holm_bonferroni(p_raw, alpha);
for k = 1:size(pairs, 1)
    res = 'Significant'; if h_adj(k) == 0, res = 'Not Sig.'; end
    fprintf('%-15s %-15.4e %-15.4e %-15d %-10s\n', pairs{k, 3}, p_raw(k), p_adj(k), h_adj(k), res);
end

% Cost (F_obj,t)
fprintf('\n--- DRT Formulation: Cost (F_obj,t) ---\n');
fprintf('%-15s %-15s %-15s %-15s %-10s\n', 'Comparison', 'Raw p-value', 'Adj p-value', 'Decision (h)', 'Result');
p_raw = zeros(size(pairs, 1), 1);
for k = 1:size(pairs, 1)
    p_raw(k) = ranksum(drt_fval(:,pairs{k, 1}), drt_fval(:,pairs{k, 2}), 'Alpha', alpha);
end
[p_adj, h_adj] = apply_holm_bonferroni(p_raw, alpha);
for k = 1:size(pairs, 1)
    res = 'Significant'; if h_adj(k) == 0, res = 'Not Sig.'; end
    fprintf('%-15s %-15.4e %-15.4e %-15d %-10s\n', pairs{k, 3}, p_raw(k), p_adj(k), h_adj(k), res);
end

% num_eval (Position)
fprintf('\n--- Position Formulation: num_eval ---\n');
fprintf('%-15s %-15s %-15s %-15s %-10s\n', 'Comparison', 'Raw p-value', 'Adj p-value', 'Decision (h)', 'Result');
p_raw = zeros(size(pairs, 1), 1);
for k = 1:size(pairs, 1)
    p_raw(k) = ranksum(num_eval_pos(:,pairs{k, 1}), num_eval_pos(:,pairs{k, 2}), 'Alpha', alpha);
end
[p_adj, h_adj] = apply_holm_bonferroni(p_raw, alpha);
for k = 1:size(pairs, 1)
    res = 'Significant'; if h_adj(k) == 0, res = 'Not Sig.'; end
    fprintf('%-15s %-15.4e %-15.4e %-15d %-10s\n', pairs{k, 3}, p_raw(k), p_adj(k), h_adj(k), res);
end

% num_eval (DRT)
fprintf('\n--- DRT Formulation: num_eval ---\n');
fprintf('%-15s %-15s %-15s %-15s %-10s\n', 'Comparison', 'Raw p-value', 'Adj p-value', 'Decision (h)', 'Result');
p_raw = zeros(size(pairs, 1), 1);
for k = 1:size(pairs, 1)
    p_raw(k) = ranksum(num_eval_drt(:,pairs{k, 1}), num_eval_drt(:,pairs{k, 2}), 'Alpha', alpha);
end
[p_adj, h_adj] = apply_holm_bonferroni(p_raw, alpha);
for k = 1:size(pairs, 1)
    res = 'Significant'; if h_adj(k) == 0, res = 'Not Sig.'; end
    fprintf('%-15s %-15.4e %-15.4e %-15d %-10s\n', pairs{k, 3}, p_raw(k), p_adj(k), h_adj(k), res);
end
fprintf('=================================================================================\n\n');

% ==============================================================================
% HELPER FUNCTIONS
% ==============================================================================

function [p_adj, h_dec] = apply_holm_bonferroni(p_raw, alpha)
    m = length(p_raw);
    [p_sorted, sort_idx] = sort(p_raw);
    p_adj_sorted = zeros(m, 1);
    h_dec_sorted = zeros(m, 1);
    
    current_max = 0;
    reject_flag = true;
    
    for k = 1:m
        adj_val = min(1, p_sorted(k) * (m - k + 1));
        current_max = max(current_max, adj_val);
        p_adj_sorted(k) = current_max;
        
        if reject_flag && (current_max <= alpha)
            h_dec_sorted(k) = 1;
        else
            reject_flag = false;
            h_dec_sorted(k) = 0;
        end
    end
    
    p_adj = zeros(m, 1);
    h_dec = zeros(m, 1);
    p_adj(sort_idx) = p_adj_sorted;
    h_dec(sort_idx) = h_dec_sorted;
end