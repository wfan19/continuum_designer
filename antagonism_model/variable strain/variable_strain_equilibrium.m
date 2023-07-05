%% Solve the statics of a 3-muscle antagonistic arm under variable strain.
%%% Define the geometry of an individual segment
% Create the base segment
N_segments = 10;
l_0 = 0.5; % Default length

rho = 1 * 0.0254; % Define inter-muscle geometry
g_o_X = SE2.hat([0, rho, 0]);
g_o_A = SE2.hat([0, rho * 1/3, 0]);
g_o_B = SE2.hat([0, -rho * 1/3, 0]);
g_o_Y = SE2.hat([0, -rho, 0]);
g_o_muscles = {g_o_X; g_o_A; g_o_B; g_o_Y};

g_0_o = SE2.hat([0, 0, -pi/2]);
g_0_muscles = lmatmul_cell(g_0_o, g_o_muscles); % g_i(0) = g_o(0) * g_oi

base_segment = Arm2D(g_0_o, g_o_muscles, l_0, 'plot_unstrained', false);
base_segment.rho = rho;
base_segment.n_spacers = 2;

% Create the discretized variable strain arm
arm = variable_strain_segment(N_segments, base_segment);

%% Solve the statics of the 3-muscle arm
% Define the loading functions
% TODO: These should be a part of the variable_strain_segment, or part of
% the arm?
f_force_outer = @(strain, pressure) actuatorForce_key(strain, pressure);
f_force_inner = @(strain, pressure) actuatorForce_key(strain, pressure);
force_funcs = {fitresult, @actuatorForce_key, @actuatorForce_key, fitresult};

% Define the scenario
pressures = [0, 40, 0, 0];
Q = [0; -1; 0];

% Initialize the problem
g_circ_right_initial = arm.get_neutral_base_curve();
f_equilibrium = @(v_g_circ_right) arm.check_equilibrium(v_g_circ_right, Q, pressures, force_funcs);
options = optimoptions('fsolve',"MaxFunctionEvaluations", 1e5);

profile("on")
equilibrium_soln = fsolve(f_equilibrium, g_circ_right_initial, options);
arm.set_base_curve(equilibrium_soln);
profile("off")
profile("view")

disp("Final residuals: ")
disp(f_equilibrium(equilibrium_soln));

%% Plot the arm
ax = axes(figure());
border_length_cm = 45;
axis equal
hold on
arm.plot(ax);

% Create the arrow for the tip force
xy = SE2.translation(arm.get_tip_pose());
uv = Q(1:2) * l_0/10;
quiver(xy(1), xy(2), uv(1), uv(2), "off", "linewidth", 3, "MaxHeadSize", 1);