% ==============================================================================
% File: main.m
% Description: Main orchestration script for 2R robot PID tuning, trajectory 
%              optimization, statistical benchmarking, and analytics.
% ==============================================================================

clear; 
close all; 
clc;

% Add required subdirectories
addpath('src', 'models', 'data'); 
if ~exist('results', 'dir'), mkdir('results'); end

%Load trajectory data into client RAM
try
    traj = load('opt_traj_2R.mat');
catch
    error('Trajectory file not found. Ensure it is inside the "data" folder.');
end

% Display interactive menu
fprintf('====================================================\n');
fprintf('  2R ROBOT PID OPTIMIZATION & BENCHMARKING\n');
fprintf('====================================================\n');
fprintf('1. Run FMA - Position Error Minimization (Single Run)\n');
fprintf('2. Run FMA - Dynamic Residual Torque (DRT) (Single Run)\n');
fprintf('3. Run DOA - Position Error Minimization (Single Run)\n');
fprintf('4. Run DOA - DRT Minimization (Single Run)\n');
fprintf('5. Run PSO - Position Error Minimization (Single Run)\n');
fprintf('6. Run PSO - DRT Minimization (Single Run)\n');
fprintf('7. Run Full 30-Run Statistical Benchmark Suite\n');
fprintf('8. Publication Analytics & Plotting Sub-Menu\n');
fprintf('====================================================\n');
user_choice = input('Select an execution pathway (1-8): ');

% ------------------------------------------------------------------------------
% Algorithm Parameters and Bounds
% ------------------------------------------------------------------------------
lower_bounds = [0, 0, 0, 0, 0, 0];
upper_bounds = [33443.7632, 1065.088, 16721.8816, 3233.9488, 103, 1616.9744];
params = [25, 300, 0.5, 0.2, 8.94e-10]; % [SwarmSize, MaxIter, Alpha, BetaMin, Gamma] for FMA/PSO

% ------------------------------------------------------------------------------
% Worker and memory Sanitation
% ------------------------------------------------------------------------------
if user_choice >= 1 && user_choice <= 7
    % Purge any existing corrupted caches
    if exist('slprj', 'dir'), rmdir('slprj', 's'); end
    delete('*.slxc');

    % Reset parallel pool to guarantee a fresh memory space
    if ~isempty(gcp('nocreate'))
        delete(gcp('nocreate'));
    end
    
    fprintf('Initializing clean parallel pool (2 workers)...\n');
    parpool(2);
    
    % Disable SDI logging and isolate worker caches to prevent read/write collisions
    Simulink.sdi.setAutoArchiveMode(false);
    Simulink.sdi.clear;
    parfevalOnAll(@Simulink.sdi.setAutoArchiveMode, 0, false);
    parfevalOnAll(@Simulink.sdi.clear, 0);
    parfevalOnAll(@Simulink.fileGenControl, 0, 'set', 'CacheFolder', tempdir, 'CodeGenFolder', tempdir);
    
    % Pre-load model on workers
    fprintf('Pre-loading Simulink model on workers...\n');
    parfevalOnAll(@load_system, 0, 'Robot_2R_Dynamics_and_Control');
end

% Anonymous cost wrappers
cost_pos = @(x) cost_function_err(x, traj);
cost_drt = @(x) cost_function_drt(x, traj);

% ------------------------------------------------------------------------------
% Execution Routing
% ------------------------------------------------------------------------------
switch user_choice
    case 1
        fprintf('\n[INFO] Starting FMA Position Optimization...\n');
        [x_opt_pos, fval_pos, NumEval, gen_conv, total_time] = fma(cost_pos, lower_bounds, upper_bounds, params);
        save('results/gain_fma_pos_final.mat', 'x_opt_pos', 'fval_pos', 'NumEval', 'gen_conv', 'total_time');
        fprintf('[SUCCESS] FMA Position run saved.\n');
        
    case 2
        fprintf('\n[INFO] Starting FMA DRT Optimization...\n');
        [x_opt_drt, fval_drt, NumEval, gen_conv, total_time] = fma(cost_drt, lower_bounds, upper_bounds, params);
        save('results/gain_fma_drt_final.mat', 'x_opt_drt', 'fval_drt', 'NumEval', 'gen_conv', 'total_time');
        fprintf('[SUCCESS] FMA DRT run saved.\n');
        
    case 3
        fprintf('\n[INFO] Starting DOA Position Optimization...\n');
        [x_opt_doa_pos, fval_doa_pos, conv_doa_pos, NumEval, gen_conv, total_time] = doa(cost_pos, params(1), params(2), lower_bounds, upper_bounds, 6);
        save('results/gain_doa_pos_final.mat', 'x_opt_doa_pos', 'fval_doa_pos', 'conv_doa_pos', 'NumEval', 'gen_conv', 'total_time');
        fprintf('[SUCCESS] DOA Position run saved.\n');
        
    case 4
        fprintf('\n[INFO] Starting DOA DRT Optimization...\n');
        [x_opt_doa_drt, fval_doa_drt, conv_doa_drt, NumEval, gen_conv, total_time] = doa(cost_drt, params(1), params(2), lower_bounds, upper_bounds, 6);
        save('results/gain_doa_drt_final.mat', 'x_opt_doa_drt', 'fval_doa_drt', 'conv_doa_drt', 'NumEval', 'gen_conv', 'total_time');
        fprintf('[SUCCESS] DOA DRT run saved.\n');
        
    case 5
        fprintf('\n[INFO] Starting PSO Position Optimization...\n');
        [x_opt_pso_pos, fval_pso_pos, NumEval, gen_conv, total_time] = pso(cost_pos, lower_bounds, upper_bounds, params);
        save('results/gain_pso_pos_final.mat', 'x_opt_pso_pos', 'fval_pso_pos', 'NumEval', 'gen_conv', 'total_time');
        fprintf('[SUCCESS] PSO Position run saved.\n');
        
    case 6
        fprintf('\n[INFO] Starting PSO DRT Optimization...\n');
        [x_opt_pso_drt, fval_pso_drt, NumEval, gen_conv, total_time] = pso(cost_drt, lower_bounds, upper_bounds, params);
        save('results/gain_pso_drt_final.mat', 'x_opt_pso_drt', 'fval_pso_drt', 'NumEval', 'gen_conv', 'total_time');
        fprintf('[SUCCESS] PSO DRT run saved.\n');
        
    case 7
        fprintf('\n[INFO] Launching full 30-run statistical benchmark suite...\n');
        run_statistical_benchmark;
        
    case 8
        fprintf('\n====================================================\n');
        fprintf('  PUBLICATION ANALYTICS & PLOTTING SUB-MENU\n');
        fprintf('====================================================\n');
        fprintf('1. Stage 1 Optimal Trajectory Profile\n');
        fprintf('2. Ablation Study: Unconstrained vs Constrained\n');
        fprintf('3. Statistical Optimizer Selection (Boxplots & Wilcoxon)\n');
        fprintf('4. Physical Metrics & Trade-off Evaluations (Pareto)\n');
        fprintf('====================================================\n');
        plot_choice = input('Select an analytical module (1-4): ');
        
        switch plot_choice
            case 1
                plot_stage1_traj;
            case 2
                ablation_study_stage1;
            case 3
                statistical_optimizer_selection;
            case 4
                physical_metrics_and_tradeoff;
            otherwise
                fprintf('\n[ERROR] Invalid plotting module selected.\n');
        end
        
    otherwise
        fprintf('\n[ERROR] Invalid execution pathway selected. Terminating script.\n');
end