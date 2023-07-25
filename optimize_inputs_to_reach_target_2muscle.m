%% Initialize the script with an arm design
% Given an arm design, actuator descriptions, and a loading scenario, find
% the actuations that will get the arm closest to reaching the target pose.

% Build a simple 4-muscle 2D arm
l_0 = 0.5;
rho = 0.0254;

g_o_A = Pose2.hat([0, rho, 0]);
g_o_B = Pose2.hat([0, -rho, 0]);
g_o_rods = {g_o_A; g_o_B};

g_0_o = Pose2.hat([0, 0, -pi/2]);

arm_segment = ArmSegment(Pose2, g_0_o, g_o_rods, l_0);

arm_segment.rod_o.mechanics.l_0 = l_0; % Default length of the whole segment

arm_segment.rods(1).mechanics = GinaMuscleMechanics(l_0);
arm_segment.rods(2).mechanics = GinaMuscleMechanics(l_0);

% TODO: Better way to create a series of segments.
N_segments = 3;
arm_segments = ArmSegment.empty(0, N_segments);
for i = 1 : N_segments
    arm_segments(i) = copy(arm_segment);
end
arm_series= ArmSeries(arm_segments);

%% Manually scope out the optimization landscape
disp("Preforming naive sweep of pressure space")
Q = [0; 0; 0];
g_tip_goal = Pose2.hat([0.5; -0.5; 0]);

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

mesh(P1, P2, E);

%% Now perform optimization over the input pressure space
tic
options = optimoptions("fmincon", "DiffMinChange", 0.1);
f_tip_error = @(pressures) tip_error(pressures, arm_series, Q, g_tip_goal);
pressures_optim = fmincon(f_tip_error, [30; 0], [], [], [], [], [0; 0], [100; 100]);
toc

%% Verify the optimization solution
g_circ_optim = arm_series.solve_equilibrium_gina(pressures_optim, Q);
g_tip_optim = arm_series.get_tip_pose(g_circ_optim);

delta_g = inv(g_tip_optim) * g_tip_goal;
disp("Final tip pose error:")
disp(Pose2.vee(delta_g))

%%
function error = tip_error(pressures, arm, Q, g_tip_goal)
    g_circ_right_eq = arm.solve_equilibrium_gina(pressures, Q);
    tip_pose = arm.get_tip_pose(g_circ_right_eq);
    v_error = Twist2.vee(logm(inv(tip_pose) * g_tip_goal));
    K = diag([1, 1, 1]);
    error = v_error' * K * v_error;
end
