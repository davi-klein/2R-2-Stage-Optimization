function run_batch_pso(cost_fcn, lb, ub, params, num_runs, name)
% RUN_BATCH_PSO Robust sequential execution of PSO benchmark.

    save_dir = fullfile('results', 'statistical_benchmark');
    if ~exist(save_dir, 'dir')
        mkdir(save_dir);
    end
    save_file = fullfile(save_dir, [name, '_30runs.mat']);
    
    if exist(save_file, 'file')
        load(save_file, 'history');
        start_run = find(history.fval == 0, 1);
        if isempty(start_run) && length(history.fval) >= num_runs
            fprintf('[INFO] %s already has %d completed runs. Skipping.\n', name, num_runs);
            return;
        elseif isempty(start_run)
            start_run = length(history.fval) + 1;
        end
    else
        history = struct('best_x', zeros(num_runs, 6), ...
                         'fval', zeros(num_runs, 1), ...
                         'num_eval', zeros(num_runs, 1), ...
                         'gen_conv', zeros(num_runs, 1), ...
                         'time', zeros(num_runs, 1));
        start_run = 1;
    end
    
    % Ensure clean slate and load model in base session
    bdclose('all');
    Simulink.sdi.setAutoArchiveMode(false);
    Simulink.sdi.clear;
    
    load_system('Robot_2R_Dynamics_and_Control');
    set_param('Robot_2R_Dynamics_and_Control', 'FastRestart', 'off');
    
    for r = start_run:num_runs
        fprintf('\n==============================================\n');
        fprintf(' %s | Starting Run %d of %d\n', name, r, num_runs);
        fprintf('==============================================\n');
        
        rng(3000 + r);
        
        [best_x, best_fval, NumEval, gen_conv, total_time] = pso(cost_fcn, lb, ub, params);
        
        history.best_x(r, :) = best_x;
        history.fval(r) = best_fval;
        history.num_eval(r) = NumEval;
        history.gen_conv(r) = gen_conv;
        history.time(r) = total_time;
        
        save(save_file, 'history');
        fprintf('[SAVED] %s Run %d completed. Cost: %.8e, Time: %.1fs\n', ...
                name, r, best_fval, total_time);
            
        Simulink.sdi.clear;
        parfevalOnAll(@Simulink.sdi.clear, 0);
    end
    
    set_param('Robot_2R_Dynamics_and_Control', 'FastRestart', 'on');
    close_system('Robot_2R_Dynamics_and_Control', 0);
end