% ==============================================================================
% File: fma.m
% Description: Firefly Metaheuristic Algorithm (FMA) adapted for parallel 
%              execution. Credits to Xin-She Yang.
%              Equation references from: "Nature-Inspired Metaheuristic 
%              Algorithms" (2nd Edition) by Xin-She Yang.
% ==============================================================================
function [best_x, best_fval, NumEval, gen_convergencia, tempo_total] = fma(cost_func, Lb, Ub, para)
    
    n = para(1); MaxGeneration = para(2);
    alpha = para(3); betamin = para(4); gamma = para(5);
    d = length(Lb);
    
    % Initialization
    ns = repmat(Lb, n, 1) + repmat((Ub - Lb), n, 1) .* rand(n, d);
    Lightn = ones(n, 1) * 10^100;
    
    stall_count = 0; 
    stall_max = 30;
    fbest_old = inf;
    scale = abs(Ub - Lb);
    
    NumEval = 0;
    gen_convergencia = 1;
    
    tic;
    for k = 1:MaxGeneration
        % Alpha reduction (implements randomness reduction conceptually similar to Eq. 10.13)
        alpha = (1 - (1 - (10^(-4)/0.9)^(1/MaxGeneration))) * alpha;
        
        zn = zeros(n, 1);
        
        % PARALLEL EVALUATION
        parfor i = 1:n
            zn(i) = cost_func(ns(i, :));
        end
        Lightn = zn;
        NumEval = NumEval + n;
        
        % Ranking
        [Lightn, Index] = sort(zn);
        ns = ns(Index, :);
        
        nso = ns; 
        Lighto = Lightn;
        nbest = ns(1, :); 
        Lightbest = Lightn(1);
        
        dynamic_tolerance = max(1e-12, Lightbest * 1e-4);
        
        improvement = fbest_old - Lightbest;
        if improvement > dynamic_tolerance
            stall_count = 0;
            gen_convergencia = k; 
            fprintf('FMA Gen %d: Improvement detected! (Req: %.2e) Cost = %.8e\n', k, dynamic_tolerance, Lightbest);
        else
            stall_count = stall_count + 1;
            fprintf('FMA Gen %d: [Stagnation %d/%d] Cost = %.8e\n', k, stall_count, stall_max, Lightbest);
        end
        
        fbest_old = Lightbest;
        
        % Flush every 20 generations
        if mod(k, 20) == 0
            Simulink.sdi.clear;
        end
        
        % Stop condition
        if stall_count >= stall_max
            break;
        end
        
        % Movement
        for i = 1:n
            for j = 1:n
                % Eq. (10.9): Cartesian distance between firefly i and firefly j
                r = sqrt(sum((ns(i,:) - ns(j,:)).^2));
                
                if Lightn(i) > Lighto(j)
                    % Eq. (10.4): Attractiveness parameter beta with exponential decay 
                    % (Modified to include 'betamin' as a minimum threshold)
                    beta = (1 - betamin) * exp(-gamma * r.^2) + betamin;
                    
                    % Eq. (10.11): Movement of firefly i attracted to brighter firefly j
                    % ( vectorized implementation of the random walk)
                    tmpf = alpha .* (rand(1, d) - 0.5) .* scale;
                    ns(i,:) = ns(i,:) .* (1 - beta) + nso(j,:) .* beta + tmpf;
                end
            end
        end
        
        % Limits
        ns = max(ns, Lb); 
        ns = min(ns, Ub); 
    end
    
    tempo_total = toc;
    best_x = nbest;
    best_fval = Lightbest;
    fprintf('\n[FMA] Optimization completed in %.2f seconds.\n', tempo_total);
end