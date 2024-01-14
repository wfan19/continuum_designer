function [residuals, pressures] = evaluate_arm(arm_series, g_targets, Q)
    
    function [pressures_optim, res] = optimize_input_to_reach_target(g_target)

        % Pull the pressure limits from each rod, which will inform bounds
        % on fmincon's search
        pressures_0 = zeros(arm_series.N_rods, 1);
        p_bounds = zeros(arm_series.N_rods, 2);
        for i = 1 : length(arm_series.segments(1).rods)
            rod_i = arm_series.segments(1).rods(i);
            p_bounds(i, :) = rod_i.mechanics.a_bounds;
        end

        % Cost function: for a given set of pressures, how far is the arm
        % tip to the target pose?
        function residual = evaluate_input(pressures)
            g_circ_right_eq = arm_series.solve_equilibrium_gina(pressures, Q, "print", false);
            tip_pose = arm_series.get_tip_pose(g_circ_right_eq);
            
            % Question: Should this be in SE2 or se2?
            %v_error = Twist2.vee(logm(inv(tip_pose) * g_target));
            v_error = Pose2.vee(tip_pose) - Pose2.vee(g_target);
    
            K = diag([1, 1, 0]); % x-weight, y-weight, theta-cost
            residual = v_error' * K * v_error;
        end

        % Solve for the pressures which minimize the tip distance to target
        opts = optimoptions("fmincon", "DiffMinChange", 0.1, "display", "final");
        tic
        [pressures_optim, res] = fmincon(@evaluate_input, pressures_0, [], [], [], [], p_bounds(:, 1), p_bounds(:, 2), [], opts);
        toc
    end

    residuals = zeros(length(g_targets), 1);
    pressures = zeros(arm_series.N_rods, length(g_targets));
    for i_target = 1 : length(g_targets)
        % For each target, solve for the pressures which minimize the tip
        % distance to target.
        g_target_i = g_targets{i_target};
        [pressures_i, residual_i] = optimize_input_to_reach_target(g_target_i);
        pressures(:, i_target) = pressures_i;
        residuals(:, i_target) = residual_i;
    end
end