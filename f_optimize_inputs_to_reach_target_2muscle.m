function g_error = f_optimize_inputs_to_reach_target_2muscle(arm_series, Q, g_tip_goal)

    % Find the pressure that minimizes g_error
    tic
    options = optimoptions("fmincon", "DiffMinChange", 0.1);
    f_tip_error = @(pressures) tip_error(pressures, arm_series, Q, g_tip_goal);
    pressures_optim = fmincon(f_tip_error, [30; 0], [], [], [], [], [0; 0], [100; 100]);
    toc
    
    % Compute the minimal g_error and return it
    g_circ_optim = arm_series.solve_equilibrium_gina(pressures_optim, Q);
    g_tip_optim = arm_series.get_tip_pose(g_circ_optim);
    
    g_error = logm(inv(g_tip_optim) * g_tip_goal); % Question: should this be SE2 or se2?
    
    %%
    function error = tip_error(pressures, arm, Q, g_tip_goal)
        g_circ_right_eq = arm.solve_equilibrium_gina(pressures, Q);
        tip_pose = arm.get_tip_pose(g_circ_right_eq);
        v_error = Twist2.vee(logm(inv(tip_pose) * g_tip_goal));
        K = diag([1, 1, 1]);
        error = v_error' * K * v_error;
    end
end

