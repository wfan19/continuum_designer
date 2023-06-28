%% Solve the statics of a 3-muscle antagonistic arm under variable strain.

%%% Define the geometry of an individual segment
% Create the base segment
N_segments = 3;
l_0_full = 0.443; % Default length
l_0_seg = l_0_full / N_segments;

rho = 1 * 0.0254; % Define inter-muscle geometry
g_o_A = SE2.hat([0, rho, 0]);
g_o_o = eye(3);
g_o_B = SE2.hat([0, -rho, 0]);
g_o_muscles = {g_o_A; g_o_o; g_o_B};

g_0_o = SE2.hat([0, 0, pi/2]);
g_0_muscles = lmatmul_cell(g_0_o, g_o_muscles); % g_i(0) = g_o(0) * g_oi

base_segment = Arm2D(g_0_o, g_o_muscles, l_0_seg, 'plot_unstrained', false);
base_segment.rho = rho;
base_segment.n_spacers = 2;

% Create the discretized variable strain arm
s = linspace(0, 1, N_segments+1);
arm = variable_strain_segment(N_segments, base_segment);

% Test setting the twist vectors along the rod
g_circ_right = zeros(3, N_segments);
g_circ_right(1, :) = [0.4, 0.45, 0.5]/ 3;
g_circ_right(3, :) = [0.1, 0.2, 0.3];
arm.set_base_curve(g_circ_right)

%%% Plot the arm
ax = axes(figure());
border_length_cm = 45;
ylim(ax, [0, 1] * border_length_cm / 100);
xlim(ax, [-0.5, 0.5] * border_length_cm / 100);
arm.plot(ax)

%%% Solve the static equilibrium at each point
disp("TODO")

%%% Plot
