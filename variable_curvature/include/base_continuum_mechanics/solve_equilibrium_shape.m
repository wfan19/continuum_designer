function soln_mat_segment_twists = solve_equilibrium_shape(N_segments, struct_design, p, Q_tip)
    default_segment_twists = zeros(3, N_segments);

    N_starts = 5;
    curvatures = linspace(-1, 1, N_starts);

    solns = zeros(3, N_segments, N_starts);
    residuals = zeros(1, N_starts);
    for i = 1 : N_starts
        l_0 = 0.5;
        default_segment_twists(1, :) = l_0;
        default_segment_twists(3, :) = curvatures(i);
    
        opts = optimoptions("fsolve", MaxFunctionEvaluations=4e5, MaxIterations=5e3, StepTolerance=1e-8, FunctionTolerance=1e-8, Algorithm="levenberg-marquardt", Display="off");
        f = @(mat_in) check_equilibrium(mat_in, Q_tip, p, struct_design);
        [solns(:, :, i), res_i] = fsolve(f, default_segment_twists, opts);
        residuals(i) = sum(vecnorm(res_i));
    end
    
    [~, i_min] = min(residuals);
    soln_mat_segment_twists = solns(:, :, i_min);

end

