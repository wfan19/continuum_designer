%% Initialize the script with an arm design
% Given an arm design, actuator descriptions, and a loading scenario, find
% the actuations that will get the arm closest to reaching the target pose.

% Build a simple 4-muscle 2D arm
l_0 = 0.5;
rho = 0.0254;

arm_series = ArmSeriesFactory.constant_2d_muscle_arm(N_segments, rho, l_0);
arm_series.set_mechanics(GinaMuscleMechanics(l_0));

Q = [0; -1; 0];
g_tip_goal = Pose2.hat([0.5; -0.5; 0]);

%% Manually scope out the optimization landscape
disp("Preforming naive sweep of pressure space")

% Naive sweep over the pressure input space
resolution = 20;
p1 = linspace(0, 70, resolution);
p2 = linspace(0, 70, resolution);

[P1, P2] = meshgrid(p1, p2);

E = zeros(resolution, resolution);
f = waitbar(0, "Performing naive sweep of pressure space");
tic
for i = 1 : resolution
    for j = 1 : resolution
        pressures_ij = [P1(i, j); P2(i, j)];
        E(i, j) = tip_error(pressures_ij, arm_series, Q, g_tip_goal);
    end
    waitbar((resolution * (i-1) + j) / resolution^2, f, "Preforming naive sweep of pressure space");
end
toc
close(f)

%% Now perform optimization over the input pressure space
[pressures_optim, g_tip_optim, residual] = f_optimize_inputs_to_reach_target_2muscle(arm_series, Q, g_tip_goal);
disp(pressures_optim)
%% 
figure()
hold on
mesh(P1, P2, E);
scatter3(pressures_optim(1), pressures_optim(2), residual, 'ro')
view(30, 30)
grid on
%% 
ax = axes(figure());
arm_series.solve_equilibrium_gina(pressures_optim, Q);
Plotter2D.plot_arm_series(arm_series, ax);
scatter(0.5, -0.5, 'ro')

%% Error function for the naive sweep
function error = tip_error(pressures, arm, Q, g_tip_goal)
    g_circ_right_eq = arm.solve_equilibrium_gina(pressures, Q, "print", false);
    tip_pose = arm.get_tip_pose(g_circ_right_eq);
    
    % Question: Should this be in SE2 or se2?
    v_error = Twist2.vee(logm(inv(tip_pose) * g_tip_goal));

    K = diag([1, 1, 0]);
    error = v_error' * K * v_error;
end