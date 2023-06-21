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
    h_end_d = SE2.hat(arm.rods(link_tip).calc_posns());
    
    for i = 1 : length(arm.rods)
        l_i = q(2*i-1);
        k_i = q(2*i);

        J_i = pcc_segment_jacobian(l_i, k_i);
        if i <= link_tip
            h_i_p = SE2.hat(arm.rods(i).calc_posns());
            delta_h = inv(h_i_p) * h_end_d;

            mat_jacobian(:, 2*i-1 : 2*i) = inv(SE2.adjoint(h_end_d)) * inv(SE2.right_lifted_action(h_i_p)) * J_i;
        else 
            mat_jacobian(:, 2*i-1 : 2*i) = zeros(size(J_i));
            continue
        end        
    end
end