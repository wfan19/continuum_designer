function rxn = f_reactions_4muscle(rho_out, l_0, g_circ_right, p)
    f_forces = {
        @JacobBellowMechanics.actuator_force,
        @GinaMuscleMechanics.actuatorForce_key,
        @GinaMuscleMechanics.actuatorForce_key,
        @JacobBellowMechanics.actuator_force
    };

    rho_inner = 0.02;
    rho_outer = rho_out;
    l_0_i = l_0;

    g_o_X = Pose2.hat([0, -rho_outer, 0]);
    g_o_A = Pose2.hat([0, -rho_inner, 0]);
    g_o_B = Pose2.hat([0, rho_inner, 0]);
    g_o_Y = Pose2.hat([0, rho_outer, 0]);
    
    g_o_actuators = {g_o_X, g_o_A, g_o_B, g_o_Y};
    strains = zeros(4, 1);
    forces = zeros(4, 1);
    for i_actuator = 1 : length(g_o_actuators)
        g_o_i = g_o_actuators{i_actuator};
        ad_o_i = Pose2.adjoint(inv(g_o_i));
        g_circ_i = ad_o_i * g_circ_right;
        strain_i = (g_circ_i(1) - l_0_i)/l_0_i;
        strains(i_actuator) = strain_i;
        forces(i_actuator) = f_forces{i_actuator}(strain_i, p(i_actuator));
    end

    mat_A = [
        1, 1, 1, 1;
        0, 0, 0, 0;
        rho_outer, rho_inner, -rho_inner, -rho_outer
    ];

    rxn = mat_A * forces;
end