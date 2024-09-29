function tests = test_mechanics
    tests = functiontests(localfunctions);
end

function struct_antagonist = make_antagonist_arm(r_outer, r_inner)
    %%% Define arm design
    % Actuator position radii and the associated moment-arm matrix
    rs = [-r_outer, -r_inner, r_inner, r_outer];
    mat_A = [
        1, 1, 1, 1;
        0, 0, 0, 0;
        -rs;
        ];
    
    l_0 = 0.5;
    
    g_0 = Pose2.hat([0, 0, -pi/2]);
    
    % Assemble it all into structs
    struct_design_base = struct();
    struct_design_base.mat_A = mat_A;
    struct_design_base.rhos = rs;
    struct_design_base.l_0 = l_0;
    struct_design_base.g_0 = g_0;
    
    % Muscle force functions
    f_bellow = @JacobBellowMechanics.actuator_force;
    f_muscle = @GinaMuscleMechanics.actuatorForce_key;
    
    struct_antagonist = struct_design_base;
    struct_antagonist.fs = {f_bellow, f_muscle, f_muscle, f_bellow};
    struct_antagonist.p_bounds = [50; 100; 100; 50];

end

function test_antagonism1(testCase)
    %%% Test scenario
    % Make the arm design struct
    r_outer = 0.08;
    r_inner = 0.015;
    struct_design = make_antagonist_arm(r_outer, r_inner);

    % Pressure and loading
    q_tip = [3.1525; 5.5658; 0.5592];
    p = [0.99; 0.99; 0.99; 0.99];

    N_nodes = 10;
    eq_twists = solve_equilibrium_shape(N_nodes, struct_design, p, q_tip);
    eq_poses = calc_poses(struct_design.g_0, eq_twists);
    
    plot_poses(eq_poses, axes(figure()));

    tip_pose = Pose2.vee(eq_poses(:, :, end));
    color_yellow = [255 176 0] / 255;
    uv = q_tip(1:2) * 0.01;
    quiver(tip_pose(1), tip_pose(2), uv(1), uv(2), "color", color_yellow);
end

