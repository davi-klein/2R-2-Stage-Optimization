% ==============================================================================
% File: doa.m
% Description: Dhole Optimization Algorithm (DOA) adapted for parallel 
%              execution. Credits to Bnyad O. Mohammed, Hadi S. Aghdasi 
%              and Pedram Salehpour (2025).
% ==============================================================================
function [Best_pos, Best_score, Convergence_curve, NumEval, gen_conv, total_time] = doa(fobj, N, Max_iter, lb, ub, dim, type)
    
    % 1. Initialization
    X = zeros(N, dim);
    for i = 1:dim
        X(:, i) = rand(N, 1) .* (ub(i) - lb(i)) + lb(i);
    end
    
    current_fitness = zeros(N, 1);
    Convergence_curve = zeros(1, Max_iter);
    
    tic;
    NumEval = 0;
    gen_conv = 1;
    
    stall_count = 0; 
    stall_max = 30;
    
    % 2. Initial Parallel Evaluation
    parfor i = 1:N
        current_fitness(i) = fobj(X(i, :));
    end
    NumEval = NumEval + N;
    
    [Best_score, idx_best] = min(current_fitness);
    Best_pos = X(idx_best, :);
    
    prey_global = Best_pos;
    localBest_position = Best_pos;
    localBest_fitness = Best_score;
    fitness_f = current_fitness;
    fbest_old = Best_score;
    
    Convergence_curve(1) = Best_score;
    fprintf('DOA Gen 1: Best Cost = %.6f\n', Best_score);
    
    % 3. Main Iteration Loop
    for t = 2:Max_iter
        C = 1 - (t / Max_iter);
        PWN = round(rand * 15 + 5);
        prey = (prey_global + localBest_position) / 2;
        prey_local = localBest_position;
        
        Xnew = zeros(N, dim);
        
        for i = 1:N
            if rand() < 0.5
                if PWN < 10
                    Xnew(i,:) = X(i,:) + C * rand .* (prey - X(i,:)); 
                else
                    for j = 1:dim
                        z = round(rand * (N - 1)) + 1;
                        while (i == z)
                            z = round(rand * (N - 1)) + 1;
                        end
                        Xnew(i,j) = X(i,j) - X(z,j) + prey(j);
                    end
                end
            else
                Q = 3 * rand * fitness_f(i) / (localBest_fitness + eps);
                if Q > 2
                    W_prey = exp(-1/Q) .* prey_local;
                    for j = 1:dim
                        Xnew(i,j) = X(i,j) + cos(2*pi*rand)*W_prey(j)*p_obj(PWN) - sin(2*pi*rand)*W_prey(j)*p_obj(PWN);
                    end
                else
                    Xnew(i,:) = (X(i,:) - prey_global) * p_obj(PWN) + p_obj(PWN) .* rand(1,dim) .* X(i,:);
                end
            end
        end
        
        for i = 1:N
            for j = 1:dim
                Xnew(i,j) = min(ub(j), max(lb(j), Xnew(i,j)));
            end
        end
        
        % 4. Parallel Evaluation of New Positions
        new_fitness = zeros(N, 1);
        parfor i = 1:N
            new_fitness(i) = fobj(Xnew(i, :));
        end
        NumEval = NumEval + N;
        
        % 5. Update Bests
        [localBest_fitness, idx_local] = min(new_fitness);
        localBest_position = Xnew(idx_local, :);
        
        for i = 1:N
            if new_fitness(i) < fitness_f(i)
                fitness_f(i) = new_fitness(i);
                X(i,:) = Xnew(i,:);
                
                if fitness_f(i) < Best_score
                    Best_score = fitness_f(i);
                    prey_global = X(i,:);
                    Best_pos = X(i,:);
                end
            end
        end
        
        Convergence_curve(t) = Best_score;
        
        dynamic_tolerance = max(1e-12, Best_score * 1e-4);
        
        improvement = fbest_old - Best_score;
        if improvement > dynamic_tolerance
            stall_count = 0;
            gen_conv = t;
            fprintf('DOA Gen %d: Improvement detected! (Req: %.2e) Cost = %.8e\n', t, dynamic_tolerance, Best_score);
        else
            stall_count = stall_count + 1;
            fprintf('DOA Gen %d: [Stagnation %d/%d] Cost = %.8e\n', t, stall_count, stall_max, Best_score);
        end
        
        fbest_old = Best_score;
        
        % Client-side flush every 20 generations
        if mod(t, 20) == 0
            Simulink.sdi.clear;
        end
        
        if stall_count >= stall_max
            break;
        end
    end
    
    total_time = toc;
    fprintf('\n[DOA] Optimization completed in %.2f seconds.\n', total_time);
end

function y = p_obj(x)   
    y = ((1 / (1 + exp(-0.5 * (x - 25))))^2) * rand;
end