function mat_reactions = calc_reaction_forces(arm_obj, Q)
    g_tip = arm_obj.get_tip_pose();

    mat_reactions = zeros(3, length(arm_obj.arms));
    for i = 1 : length(arm_obj.arms)
        % For each segment, calculate the reaction forces at its base
        segment = arm_obj.arms{i};
        g_i = segment.g_o;
        g_i_tip = inv(g_i) * g_tip;        

        Q_right_circ_tip = SE2.left_lifted_action(g_i)' * Q;
        Q_right_circ_i = SE2.adjoint(g_i_tip)' * Q_right_circ_tip;

        mat_reactions(:, i) = Q_right_circ_i;
    end
end