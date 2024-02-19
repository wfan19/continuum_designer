function residual = check_equilibrium(mat_segment_twists, Q_tip, p, struct_design)
    % Compute the externally applied wrenches
    external_wrenches = calc_external_wrench(mat_segment_twists, Q_tip, struct_design);

    % Compute the internal reaction wrenches
    reaction_wrenches = calc_reaction_wrench(mat_segment_twists, p, struct_design);

    residual = external_wrenches + reaction_wrenches;
    residual(2, :) = 1e5 * mat_segment_twists(2, :);
end