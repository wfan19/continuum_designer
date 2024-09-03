function metric_out = calc_abs_attainability_metric(segment_twists, w_tip, struct_design)
    N_nodes = size(segment_twists, 2);
    
    [bndry_af, bndry_am] = calc_attainable_wrench_hull(segment_twists, struct_design);
    i_bndry = boundary(bndry_af(1, :)', bndry_am(1, :)', 0);
    bndry_af = bndry_af(:, i_bndry);
    bndry_am = bndry_am(:, i_bndry);

    a_requirements = -calc_external_wrench(segment_twists, w_tip, struct_design.g_0);

    metric_out = 0;
    for i = 1 : N_nodes
        % Compute the minimum distance from each requirement point to
        % absolute attainable wrench space sets
        requirement_i = a_requirements([1, 3], i);
        bndry_verts_i = [bndry_af(i, :); bndry_am(i, :)];
        metric_out = metric_out + distance_to_convex_set(requirement_i, bndry_verts_i);
    end
end

