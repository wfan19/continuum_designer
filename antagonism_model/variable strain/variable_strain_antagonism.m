%% Solve the statics of a 3-muscle antagonistic arm under variable strain.

%%% Discretize the arm into segments
N_segments = 6;
s = linspace(0, 1, N_segments+1);

%%% Define the geometry of the arm
arms = cell(1, N_segments);
% Create the base segment
rho = 1 * 0.0254; % Define inter-muscle geometry
g_o_A = SE2.hat([0, rho, 0]);
g_o_o = eye(3);
g_o_B = SE2.hat([0, -rho, 0]);
g_o_i_muscles = {g_o_A; g_o_o; g_o_B};

g_0_o = SE2.hat([0, 0, pi/2]);
g_0_muscles = lmatmul_cell(g_0_o, g_o_i_muscles); % g_i(0) = g_o(0) * g_oi

l_0_full = 0.443; % Default length
l_0_seg = l_0_full / N_segments;

arms{1} = Arm2D(g_0_o, g_0_muscles, l_0_seg, 'plot_unstrained', false);
arms{1}.rho = rho;
arms{1}.n_spacers = 2;

% Initialize plotting for the first segment
ax = axes(figure());
line_options_muscles = struct("LineWidth", 3);
line_options_spacers = struct("Linewidth", 2.5);
line_options_base_curve = struct("Linestyle", ":");
arms{1}.initialize_plotting(ax, "line_options_muscles", line_options_muscles, ...
"line_options_spacers", line_options_spacers, "line_options_base_curve", line_options_base_curve);
arms{1}.plot_arm();

% Now create N-segments based on this initial segment
for i = 1 : N_segments - 1
    g_tip_i = SE2.hat(arms{i}.muscle_o.calc_posns());
    g_o_i = arms{i}.g_o;
    tform_along_i = inv(g_o_i) * g_tip_i; % TODO FIXME
    g_0_muscles_i = {arms{i}.muscles.g_0};
    
    g_o_next = arms{i}.g_o * tform_along_i;
    g_0_muscles_next = rmatmul_cell(g_0_muscles_i, tform_along_i);

    arms{i+1} = Arm2D(g_o_next, g_0_muscles_next, l_0_seg, 'plot_unstrained', false);
    arms{i+1}.rho = rho;
    arms{i+1}.n_spacers = 2;

    arms{i+1}.initialize_plotting(ax, "line_options_muscles", line_options_muscles, ...
    "line_options_spacers", line_options_spacers, "line_options_base_curve", line_options_base_curve);

    arms{i+1}.plot_arm();
end

border_length_cm = 45;
ylim(ax, [0, 1] * border_length_cm / 100);
xlim(ax, [-0.5, 0.5] * border_length_cm / 100);

%%% Solve the static equilibrium at each point
disp("TODO")

%%% Plot
