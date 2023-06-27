function g_s = integrate_variable_strain(s, g_circ_right, g_0)
    % Cumulative exponentiation / On-manifold Euler integration
    % Currently on SE(2)
    g_s = cell(1, length(s));
    
    g_s{1} = g_0;

    for i = 1 : length(s) - 1
        delta_s = s(i+1) - s(i);
        g_circ_right_i = g_circ_right(:, i);
        delta_g_right = se2.expm(g_circ_right_i * delta_s);

        g_s{i+1} = g_s{i} * delta_g_right;
    end
end

