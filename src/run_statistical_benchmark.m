% ==============================================================================
% File: run_statistical_benchmark.m
% Description: Master batch script running 30 independent trials across 
%              all 4 experimental conditions.
% ==============================================================================

clear; close all; clc;

addpath('src', 'models', 'data');

if exist('slprj', 'dir'), rmdir('slprj', 's'); end
delete('*.slxc');

%Initialize parpool
if ~isempty(gcp('nocreate'))
    delete(gcp('nocreate'));
end
parpool(2);

% Configure SDI on the MASTER client ONLY to avoid connector errors
Simulink.sdi.setAutoArchiveMode(false);
Simulink.sdi.clear;

% Configure isolated worker caches without touching SDI on the workers
spmd
    t = getCurrentTask();
    worker_id = t.ID;
    worker_dir = fullfile(pwd, sprintf('slprj_worker_%d', worker_id));
    if ~exist(worker_dir, 'dir')
        mkdir(worker_dir);
    end
    Simulink.fileGenControl('set', 'CacheFolder', worker_dir, ...
        'CodeGenFolder', worker_dir, 'createDir', true);
end

%Load trajectory
try
    traj = load('opt_traj_2R.mat');
catch
    error('Trajectory file not found in "data" folder.');
end

% Anonymous cost functions
cost_pos = @(x) cost_function_err(x, traj);
cost_drt = @(x) cost_function_drt(x, traj);

% Bounds and Parameters
lower_bounds = [0, 0, 0, 0, 0, 0];
upper_bounds = [33443.7632, 1065.088, 16721.8816, 3233.9488, 103, 1616.9744];
params = [25, 300, 0.5, 0.2, 8.94e-10];
num_runs = 30;

% sequential Dry-Run to Build the .slxc Cache Safely
fprintf('\nExecuting sequential dry-run to compile Simulink cache...\n');
cost_pos(lower_bounds); 

% Pre-load the now-compiled model onto the workers
parfevalOnAll(@load_system, 0, 'Robot_2R_Dynamics_and_Control');

%Execution Routing
fprintf('\n>>> STEP 1/6: FMA Position Error (30 runs) <<<\n');
run_batch_fma(cost_pos, lower_bounds, upper_bounds, params, num_runs, 'FMA_Pos');

fprintf('\n>>> STEP 2/6: FMA DRT Minimization (30 runs) <<<\n');
run_batch_fma(cost_drt, lower_bounds, upper_bounds, params, num_runs, 'FMA_DRT');

fprintf('\n>>> STEP 3/6: PSO Position Error (30 runs) <<<\n');
run_batch_pso(cost_pos, lower_bounds, upper_bounds, params, num_runs, 'PSO_Pos');

fprintf('\n>>> STEP 4/6: PSO DRT Minimization (30 runs) <<<\n');
run_batch_pso(cost_drt, lower_bounds, upper_bounds, params, num_runs, 'PSO_DRT');

fprintf('\n>>> STEP 5/4: DOA Position Error (30 runs) <<<\n');
run_batch_doa(cost_pos, lower_bounds, upper_bounds, params, num_runs, 'DOA_Pos');

fprintf('\n>>> STEP 6/4: DOA DRT Minimization (30 runs) <<<\n');
run_batch_doa(cost_drt, lower_bounds, upper_bounds, params, num_runs, 'DOA_DRT');

fprintf('\n[COMPLETED] All 180 statistical benchmark runs finished successfully.\n');