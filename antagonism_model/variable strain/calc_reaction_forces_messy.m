function mat_reactions = calc_reaction_forces_messy(arm_obj, Q)
    g_tip = arm_obj.get_tip_pose();

    mat_reactions = zeros(3, length(arm_obj.arms));

    % For each point, solve the residual
    N_segments = length(arm_obj.arms);
    for i = 1 : N_segments
        reactions_i = [0; 0; 0];

        % Compute current position, and distance vector to end point
        arm_i = arm_obj.arms{i};
        g_i = arm_i.g_o;
        delta_g_i =  g_tip * inv(g_i);
        r_i = SE2.translation(delta_g_i);
        r_i = [r_i(1); r_i(2); 0];

        reactions_i(1) = dot(Q(1:2), g_i(1:2, 1));

        moment_from_Qf = norm(cross(diag([1, 1, 0]) * Q, r_i));
        reactions_i(3) = moment_from_Qf + Q(3);

        mat_reactions(:, i) = reactions_i;
    end
end