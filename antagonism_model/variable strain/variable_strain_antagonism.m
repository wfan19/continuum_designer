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

% Initialize plotting for the first segment
ax = axes(figure());
line_options_muscles = struct("LineWidth", 3);
line_options_spacers = struct("Linewidth", 2.5);
line_options_base_curve = struct("Linestyle", ":");

% Create the discretized variable strain arm
s = linspace(0, 1, N_segments+1);
arm = variable_strain_segment(N_segments, base_segment);

% Plotting
border_length_cm = 45;
ylim(ax, [0, 1] * border_length_cm / 100);
xlim(ax, [-0.5, 0.5] * border_length_cm / 100);

arm.plot(ax)

% Test setting the twist vectors along the rod
g_circ_right = zeros(3, N_segments);
g_circ_right(1, :) = linspace(l_0_seg, l_0_seg * 1.2, N_segments);
g_circ_right(3, :) = linspace(0, -0.3, N_segments);
for i = 1 : length(arm.arms) - 1
    arm.arms{i}.muscle_o.h_tilde = g_circ_right(:, i);
    arm.arms{i+1}.g_o = SE2.hat(arm.arms{i}.muscle_o.calc_posns());
    % Now transform the other muscles...
end

arm.plot(ax)

%%% Solve the static equilibrium at each point
disp("TODO")

%%% Plot
