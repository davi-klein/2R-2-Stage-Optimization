% ==============================================================================
% File: pso.m
% Description: Particle Swarm Optimization wrapper using MATLAB's built-in 
%              particleswarm algorithm with custom dynamic stagnation limits.
% ==============================================================================
function [best_x, best_fval, NumEval, gen_conv, total_time] = pso(cost_fcn, lb, ub, params)
    
    SwarmSize = params(1);
    MaxIterations = params(2);
    num_vars = length(lb);
    
    % Initialize tracking variables in parent scope
    stall_count = 0;
    stall_max = 30;
    fbest_old = inf;
    gen_conv = 1;
    
    %custom Output Function for Dynamic Stagnation and Cache Flushing
    function stop = pso_output(optimValues, state)
        stop = false;
        
        % Only evaluate stagnation after a complete iteration
        if strcmp(state, 'iter')
            current_best = optimValues.bestfval;
            
            % Dynamic tolerance logic strictly matching FMA and DOA
            dynamic_tolerance = max(1e-12, current_best * 1e-4);
            improvement = fbest_old - current_best;
            
            if improvement > dynamic_tolerance
                stall_count = 0;
                gen_conv = optimValues.iteration;
                fprintf('PSO Gen %d: Improvement detected! (Req: %.2e) Cost = %.8e\n', optimValues.iteration, dynamic_tolerance, current_best);
            else
                stall_count = stall_count + 1;
                fprintf('PSO Gen %d: [Stagnation %d/%d] Cost = %.8e\n', optimValues.iteration, stall_count, stall_max, current_best);
            end
            
            fbest_old = current_best;
            
            % Enforce the custom 30-stall hard stop
            if stall_count >= stall_max
                stop = true;
            end
            
            % SDI flush every 20 generations
            if mod(optimValues.iteration, 20) == 0
                Simulink.sdi.clear;
            end
        end
    end
    
    %Configure PSO options to bypass MATLAB's built-in limits
    options = optimoptions('particleswarm', ...
        'SwarmSize', SwarmSize, ...
        'MaxIterations', MaxIterations, ...
        'UseParallel', true, ...
        'Display', 'off', ...                
        'TolFun', 0, ...                  
        'StallIterLimit', MaxIterations, ... 
        'OutputFcn', @pso_output);           
    tic;

    %Execute built-in PSO
    [best_x, best_fval, ~, output] = particleswarm(cost_fcn, num_vars, lb, ub, options);
    total_time = toc;
    
    NumEval = output.funccount;
    
    fprintf('\n[PSO] Optimization completed in %.2f seconds.\n', total_time);
end