function tests = test_jacobian
tests = functiontests(localfunctions);
end

function test_segment_twists = create_test_twists(N_segs, N_test_pts)
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
end

function test_reaction_jacobian(testCase)   
    %%% Declare baseline test arm design and muscle pressures
    % TODO: test for different pressures?
    p = [60; 50];
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

    %%% Create a set of random linearization points
    % Create the random base-curve curvatures to linearize around
    N_segs = 10;
    N_test_pts = 10;
    rng(0);
    test_segment_twists = create_test_twists(N_segs, N_test_pts);
    
    f_reaction = @(test_twists) calc_reaction_wrench(test_twists, p, struct_design);
    for i = 1 : N_test_pts
        %%% For each linearization point, take a random small step in x, and track
        % how that changes the output
        test_twists_i = test_segment_twists(:, :, i);
        [e, d_a_numeric, d_g_circ, J_x] = test_numeric_jacobian(f_reaction, test_twists_i, -1e-5, 1e-5);
        
        %%% Compare to the output from J*dx using the analytic implementation
        % First we need to construct the analytic Jacobian
        fprintf("Test %d", i);
        disp([d_a_numeric(:), J_x*d_g_circ(:)]);
        verifyLessThan(testCase, e, 1e-3);
    end
end

function test_external_jacobian(testCase)
    %%% Define baseline test arm design and external load
    theta_0 = -pi/2;
    g_0 = Pose2.hat([0, 0, theta_0]);
    Q = [3; 0; -10];

    %%% Create a set of random linearization points
    % Create the random base-curve curvatures to linearize around
    N_segs = 5;
    N_test_pts = 10;
    rng(0);
    test_segment_twists = create_test_twists(N_segs, N_test_pts);
    
    % f_reaction = @(test_twists) deal(...
    %     calc_external_wrench(test_twists, Q, g_0), ...
    %     J_external_5seg(theta_0, Q, test_twists) ...
    % );
    f_reaction = @(test_twists) calc_external_wrench(test_twists, Q, g_0);
    for i = 1 : N_test_pts
        %%% For each linearization point, take a random small step in x, and track
        % how that changes the output
        test_twists_i = test_segment_twists(:, :, i);
        [e, d_a_numeric, d_g_circ, J_x] = test_numeric_jacobian(f_reaction, test_twists_i, -1e-5, 1e-5);
        
        %%% Compare to the output from J*dx using the analytic implementation
        % First we need to construct the analytic Jacobian
        fprintf("Test %d", i);
        disp([d_a_numeric(:), J_x * d_g_circ(:)]);
        verifyLessThan(testCase, e, 1e-3);
    end
end