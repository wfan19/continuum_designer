function tests = test_jacobian
tests = functiontests(localfunctions);
end

function test_reaction_jacobian(testCase)
    %%% Create a set of random linearization points
    % Create the random base-curve curvatures to linearize around
    N_segs = 10;
    N_test_pts = 10;
    rng(0);
    test_segment_twists = rand([3, N_segs, N_test_pts]);
    for i = 1 : N_test_pts
        % Go through each twist matrix and scale the random values to
        % appropriate ranges
        length_min = 0.4;
        length_max = 0.6;
        length_range = length_max - length_min;

        shear = 0;
        
        curvature_min = -1;
        curvature_max = 1;
        curvature_range = curvature_max - curvature_min;
        
        K = diag([length_range, shear, curvature_range]);
        c = [length_min; shear; curvature_min];
        test_segment_twists(:, :, i) = K * test_segment_twists(:, :, i) + c;
    end
    
    % Test arm and pressures
    p = [10; 20];
    struct_design = struct();
    struct_design.g_0 = Pose2.hat([0, 0, -pi/2]);
    struct_design.rhos = [-0.05, 0.05];
    struct_design.l_0 = 0.5;
    struct_design.fs = {
        @JacobBellowMechanics.actuator_force,
        @JacobBellowMechanics.actuator_force,
    };

    struct_design.mat_A = [
        1, 1;
        0, 0;
        struct_design.rhos;
    ];
    
    for i = 1 : N_test_pts
        %%% For each linearization point, take a random small step in x, and track
        % how that changes the output
        test_twists_i = test_segment_twists(:, :, i);
        
        [rxn_wrench_g_circ, J] = calc_reaction_wrench(test_twists_i, p, struct_design);
       
        % Generate a random small change in base curve shape
        d_g_circ_max = 1e-3;
        d_g_circ_min = -1e-3;
        range = d_g_circ_max - d_g_circ_min;
        d_g_circ = range*rand([3, N_segs]) + d_g_circ_min;
        test_twists_i_perturbed = test_twists_i + d_g_circ; % Slightly modified base-curve

        % Compute the reaction at the slightly different base-curve
        rxn_wrench_perturbed = calc_reaction_wrench(test_twists_i_perturbed, p, struct_design);
        
        d_a_numeric = rxn_wrench_perturbed - rxn_wrench_g_circ;
        fprintf("Test %d", i);

        %%% Compare to the output from J*dx using the analytic implementation
        % First we need to construct the analytic Jacobian
        d_a_analytic = J * d_g_circ(:);
        disp([d_a_numeric(:), d_a_analytic]);
    end
end

function test_external_jacobian(testCase)

end