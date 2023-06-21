%% Jacobian function implementation
function mat_jacobian = pcc_jacobian(arm, q, link_tip)
    arguments
        arm
        q
        link_tip=0
    end

    mat_jacobian = zeros(3, length(q));

    g_base_tform = arm.rods(1).g_0;
    if link_tip == 0
        link_tip = length(arm.rods);
    end
    g_end_d = SE2.hat(arm.rods(link_tip).calc_posns());
    
    for i = 1 : length(arm.rods)
        l_i = q(2*i-1);
        k_i = q(2*i);

        J_i = pcc_segment_jacobian(l_i, k_i);
        if i <= link_tip
            temp = arm.rods(i).g_0;
            g_i_p = SE2.hat(arm.rods(i).calc_posns());
            delta_h = inv(temp) * g_end_d;

            TeLg = SE2.left_lifted_action(SE2.hat(arm.rods(i).calc_posns()));

            mat_jacobian(:, 2*i-1 : 2*i) = TeLg * inv(SE2.adjoint(g_end_d)) * inv(SE2.right_lifted_action(g_i_p)) * J_i;
        else 
            mat_jacobian(:, 2*i-1 : 2*i) = zeros(size(J_i));
            continue
        end        
    end
end