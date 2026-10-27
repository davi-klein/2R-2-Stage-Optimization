function run_batch_doa(cost_fcn, lb, ub, params, num_runs, name)
% RUN_BATCH_DOA Robust sequential execution of DOA benchmark.

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
        
        rng(2000 + r);
        
        [best_pos, best_score, ~, NumEval, gen_conv, total_time] = ...
            doa(cost_fcn, params(1), params(2), lb, ub, 6);
        
        history.best_x(r, :) = best_pos;
        history.fval(r) = best_score;
        history.num_eval(r) = NumEval;
        history.gen_conv(r) = gen_conv;
        history.time(r) = total_time;
        
        save(save_file, 'history');
        fprintf('[SAVED] %s Run %d completed. Cost: %.8e, Time: %.1fs\n', ...
                name, r, best_score, total_time);
            
        Simulink.sdi.clear;
        parfevalOnAll(@Simulink.sdi.clear, 0);
    end
    
    set_param('Robot_2R_Dynamics_and_Control', 'FastRestart', 'on');
    close_system('Robot_2R_Dynamics_and_Control', 0);
end