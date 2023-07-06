%% Solve the statics of a 3-muscle antagonistic arm under variable strain.
%%% Define the geometry of an individual segment
% Create the base segment
N_segments = 10;
l_0 = 0.75; % Default length

rho = 1 * 0.0254; % Define inter-muscle geometry
g_o_X = SE2.hat([0, rho, 0]);
g_o_A = SE2.hat([0, rho * 1/3, 0]);
g_o_B = SE2.hat([0, -rho * 1/3, 0]);
g_o_Y = SE2.hat([0, -rho, 0]);
g_o_muscles = {g_o_X; g_o_A; g_o_B; g_o_Y};

g_0_o = SE2.hat([0, 0, -pi/2]);
g_0_muscles = lmatmul_cell(g_0_o, g_o_muscles); % g_i(0) = g_o(0) * g_oi

f_force_outer = @(strain, pressure) actuatorForce_key(strain, pressure);
f_force_inner = @(strain, pressure) actuatorForce_key(strain, pressure);
force_funcs = {fitresult, @actuatorForce_key, @actuatorForce_key, fitresult};

base_segment = Arm2D(g_0_o, g_o_muscles, l_0, 'plot_unstrained', false);
base_segment.rho = rho;
base_segment.n_spacers = 2;

% Create the discretized variable strain arm
arm = variable_strain_segment(N_segments, base_segment, force_funcs);

%% Find a configuration where the tip is at our desired position when unloaded
pressures = [0, 40, 0, 0];
Q = [0; 0; 0];

% Initialize the problem
equilibrium_soln = arm.solve_for_base_curve(pressures, Q);

%%% Plot the arm
ax = axes(figure());
border_length_cm = 45;
axis equal
hold on
arm.plot(ax);

% Create the arrow for the tip force
xy = SE2.translation(arm.get_tip_pose());
uv = Q(1:2) * l_0/10;
quiver(xy(1), xy(2), uv(1), uv(2), "off", "linewidth", 3, "MaxHeadSize", 1);

%% What actuator forces would produce the internal forces needed to maintain
% this shape with a 40N load downward?
g_circ_right_unloaded = equilibrium_soln;
Q = [0; -40; 0];
loaded_reaction_forces = arm.calc_reaction_forces(Q);

%%% Utilize the fact the linearity of internal forces:
% A * muscle_forces = internal_forces
% Since we know A and internal_forces, this is a least squares problems
% However, for a planar arm rank(A) = 2, but dim(muscle_forces) = N_muscles
% so for N_muscles > 2 the system is underdetermined. Thus there's an
% infinite family of solutions

% Here we solve for a couple variants:

% Least squared error solution
muscle_forces_lstsq = arm.A \ loaded_reaction_forces;
disp("Least squared error solution:")
disp(muscle_forces_lstsq);

% Minimum 2-norm solution
muscle_forces_minnorm = lsqminnorm(arm.A, loaded_reaction_forces);
disp("Minimum L2 norm solution:")
disp(muscle_forces_minnorm);

% Solve a constrained least squares such all muscle forces are within their
% force bounds
f_muscle_force = @(x) actuatorForce_key(x(1), x(2));
muscle_force_min_config = fmincon(@(x) actuatorForce_key(x(1), x(2)), [0; 0], [], [], [], [], [-0.5; 0], [0.1; 100]);
muscle_force_max_config = fmincon(@(x) -actuatorForce_key(x(1), x(2)), [0; 0], [], [], [], [], [-0.5; 0], [0.1; 100]);
%muscle_force_min = f_muscle_force(muscle_force_min_config);
%muscle_force_max = f_muscle_force(muscle_force_max_config);
muscle_force_min = -40;
muscle_force_max = 10;

bellow_force_min = -10;
bellow_force_max = 40;

forces_min = [bellow_force_min; muscle_force_min; muscle_force_min; bellow_force_min];
forces_max = [bellow_force_max; muscle_force_max; muscle_force_max; bellow_force_max];

muscle_forces_lsqlin = lsqlin(arm.A, loaded_reaction_forces(:, 1), [], [], [], [], forces_min, forces_max);

%% Ignore later code for now.
%{
%% Now apply a downward load to the arm
arm_loaded = copy(arm);
pressures = [0, 40, 0, 0];
Q = [0; -40; 0];

% Initialize the problem
g_circ_right_initial = arm_loaded.get_neutral_base_curve();
f_equilibrium = @(v_g_circ_right) arm_loaded.check_equilibrium(v_g_circ_right, Q, pressures, force_funcs);
options = optimoptions('fsolve',"MaxFunctionEvaluations", 1e5);

profile("on")
equilibrium_soln = fsolve(f_equilibrium, g_circ_right_initial, options);
arm_loaded.set_base_curve(equilibrium_soln);
profile("off")

disp("Final residuals: ")
disp(f_equilibrium(equilibrium_soln));

%%% Plot the arm
ax = axes(figure());
border_length_cm = 45;
axis equal
hold on
arm_loaded.plot(ax);

% Create the arrow for the tip force
xy = SE2.translation(arm_loaded.get_tip_pose());
uv = Q(1:2) * l_0/10;
quiver(ax, xy(1), xy(2), uv(1), uv(2), "off", "linewidth", 3, "MaxHeadSize", 1);

%% Now activate antagonism
arm_antagonism = copy(arm_loaded);
pressures = [0, 40, 0, 60];
Q = [0; -40; 0];

% Initialize the problem
g_circ_right_initial = arm_antagonism.get_base_curve();
f_equilibrium = @(v_g_circ_right) arm_antagonism.check_equilibrium(v_g_circ_right, Q, pressures, force_funcs);
options = optimoptions('fsolve',"MaxFunctionEvaluations", 1e5);

profile("on")
equilibrium_soln = fsolve(f_equilibrium, g_circ_right_initial, options);
arm_antagonism.set_base_curve(equilibrium_soln);
profile("off")

disp("Final residuals: ")
disp(f_equilibrium(equilibrium_soln));

%%% Plot the arm
ax = axes(figure());
border_length_cm = 45;
axis equal
hold on
arm_antagonism.plot(ax);

% Create the arrow for the tip force
xy = SE2.translation(arm_antagonism.get_tip_pose());
uv = Q(1:2) * l_0/10;
quiver(ax, xy(1), xy(2), uv(1), uv(2), "off", "linewidth", 3, "MaxHeadSize", 1);
%}