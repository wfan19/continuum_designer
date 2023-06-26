%% Jacobian function implementation
function mat_jacobian = pcc_jacobian(arm, q, link_tip)
    arguments
        arm
        q
        link_tip=0
    end

    mat_jacobian = zeros(3, length(q));

    if link_tip == 0
        link_tip = length(arm.rods);
    end
    g_end_d = SE2.hat(arm.rods(link_tip).calc_posns());
    
    for i = 1 : length(arm.rods)
        l_i = q(2*i-1);
        k_i = q(2*i);

        J_i = pcc_segment_jacobian(l_i, k_i);
        if i <= link_tip
            % For motion in joint i:
            % TeLg_i * J_i = g_dot_i in the world frame
            % inv(TeRg_i_p) * g_dot_i = g_right_circ_i = g_right_circ_end
            % inv(adjoint(g_end_d) * g_right_circ_end = g_left_circ_end
            % TeLg_end * g_left_circ_end = g_dot_end
            % TODO: We can combine the two lifted actions on the right into a
            % single Adjoint and then merge the two adjoints to be the
            % adjoint of the delta transformation. I need to write out the math.

            g_base_tform = arm.rods(i).g_0;
            g_i_p = SE2.hat(arm.rods(i).calc_posns());
            delta_h = inv(g_base_tform) * g_end_d;
            TeLg_i = SE2.left_lifted_action(g_base_tform);
            TeLg_tip = SE2.left_lifted_action(g_end_d);

            mat_jacobian(:, 2*i-1 : 2*i) = TeLg_tip * inv(SE2.adjoint(g_end_d)) * inv(SE2.right_lifted_action(g_i_p)) * TeLg_i * J_i;
            %mat_jacobian(:, 2*i-1 : 2*i) = J_i;
        else 
            mat_jacobian(:, 2*i-1 : 2*i) = zeros(size(J_i));
            continue
        end        
    end
end