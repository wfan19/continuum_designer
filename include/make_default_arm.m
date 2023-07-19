function arm = make_default_arm(rho, N_segments, l_0)
%MAKE_DEFAULT_ARM Create a default antagonistic arm
% Actuators: | X | A | B | Y |
% where X, Y are bellows and A, B are muscles

    arguments
        rho = 1 * 0.0254;   % Radius of arm
        N_segments = 10;    % Number of discretization segments
        l_0 = 0.75;         % Neutral length of manipulator
    end

    % Define inter-muscle geometry
    g_o_X = SE2.hat([0, rho, 0]);
    g_o_A = SE2.hat([0, rho * 1/3, 0]);
    g_o_B = SE2.hat([0, -rho * 1/3, 0]);
    g_o_Y = SE2.hat([0, -rho, 0]);
    g_o_muscles = {g_o_X; g_o_A; g_o_B; g_o_Y};
    
    g_0_o = SE2.hat([0, 0, -pi/2]);
    
    fitresult = make_bellow_force_func();
    force_funcs = {fitresult, @actuatorForce_key, @actuatorForce_key, fitresult};
    
    base_segment = Arm2D(g_0_o, g_o_muscles, l_0, 'plot_unstrained', false);
    base_segment.rho = rho;
    base_segment.n_spacers = 2;
    
    % Create the discretized variable strain arm
    arm = variable_strain_segment(N_segments, base_segment, force_funcs);

end

