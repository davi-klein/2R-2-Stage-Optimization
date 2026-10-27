function Z = cost_function_err(x, traj)
    try
        % 2. Create a thread-safe simulation input object
        simIn = Simulink.SimulationInput('Robot_2R_Dynamics_and_Control');
        
        % 3. Attach variables directly to the simulation object
        simIn = setVariable(simIn, 'x', x);
        simIn = setVariable(simIn, 'xx', traj.xx);
        simIn = setVariable(simIn, 'n', length(traj.xx));
        simIn = setVariable(simIn, 'stpt', traj.xx(end));
        simIn = setVariable(simIn, 'c', 15);
        simIn = setVariable(simIn, 'key_points', [0.0000 -0.2241 1.0001; 0 1.0311 0.9999]);
        simIn = setVariable(simIn, 'position', traj.position);
        simIn = setVariable(simIn, 'velocity', traj.velocity);
        simIn = setVariable(simIn, 'acceleration', traj.acceleration);

        % 4. Set model parameters (without TimeOut, as it is invalid for setModelParameter)
        simIn = setModelParameter(simIn, ...
                  'SaveOutput', 'on', ...
                  'SaveTime', 'on', ...
                  'SignalLogging', 'off', ...
                  'SolverType', 'Fixed-step', ...
                  'Solver', 'ode4', ...
                  'FixedStep', '0.001', ...
                  'FastRestart', 'on');
                  
        % 5. Run simulation
        out = sim(simIn);
        
        time_vector = out.tout;
        if isempty(time_vector) || time_vector(end) < 2.9
            error('Simulation terminated prematurely.');
        end
        
        error1 = get_sim_data(out.err1);
        error2 = get_sim_data(out.err2);
        
        dt = 0.001;
        ise_pos_1 = sum(error1.^2) * dt;
        ise_pos_2 = sum(error2.^2) * dt;
        z_cost = (1.0 * ise_pos_1) + (1.0 * ise_pos_2);
        
        if isnan(z_cost) || isinf(z_cost)
            error('Cost diverged to NaN/Inf.');
        end
        
        max_pos1 = max(abs(get_sim_data(out.pos_signal_1)));
        max_pos2 = max(abs(get_sim_data(out.pos_signal_2)));
        max_vel1 = max(abs(get_sim_data(out.vel_signal_1)));
        max_vel2 = max(abs(get_sim_data(out.vel_signal_2)));
        max_acc1 = max(abs(get_sim_data(out.acc_signal_1)));
        max_acc2 = max(abs(get_sim_data(out.acc_signal_2)));
        max_torque1 = max(abs(get_sim_data(out.torque_signal_1)));
        max_torque2 = max(abs(get_sim_data(out.torque_signal_2)));
        
        P1_max = 100 * (pi/180); P2_max = 160 * (pi/180);
        V1_max = 3; V2_max = 8;
        A1_max = 18; A2_max = 18;
        Q1_max = 25; Q2_max = 9;
        
        g = [max_pos1 - P1_max, max_pos2 - P2_max, ...
             max_vel1 - V1_max, max_vel2 - V2_max, ...
             max_acc1 - A1_max, max_acc2 - A2_max, ...
             max_torque1 - Q1_max, max_torque2 - Q2_max];
             
        if any(isnan(g)) || any(isinf(g))
            error('Constraints diverged to NaN/Inf.');
        end
             
        penalty = 0;
        lam = 10^15;
        for k = 1:length(g)
            if g(k) > 0
                penalty = penalty + lam * (g(k)^2);
            end
        end
        
        Z = z_cost + penalty;
        clear out;
        
    catch
        Z = 10^15;
    end
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