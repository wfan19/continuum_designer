%% Solve the statics of a 3-muscle antagonistic arm under variable strain.
%%% Define the geometry of an individual segment
% Create the base segment
N_segments = 5;
l_0 = 0.5; % Default length

rho = 1 * 0.0254; % Define inter-muscle geometry
g_o_A = SE2.hat([0, rho, 0]);
g_o_B = SE2.hat([0, -rho, 0]);
g_o_muscles = {g_o_A; g_o_B};

g_0_o = SE2.hat([0, 0, -pi/2]);

base_segment = Arm2D(g_0_o, g_o_muscles, l_0, 'plot_unstrained', false);
base_segment.rho = rho;
base_segment.n_spacers = 2;

force_funcs = {@actuatorForce_key, @actuatorForce_key};

% Create the discretized variable strain arm
arm = variable_strain_segment(N_segments, base_segment, force_funcs);

%% Solve the statics of the 3-muscle arm

% Define the scenario
pressures = [20, 0];
Q = [-1; 0; 0];

% Initialize the problem
arm.solve_for_base_curve(pressures, Q)

%% Plot the arm
ax = axes(figure());
border_length_cm = 45;
axis equal
hold on
arm.plot(ax, Q);