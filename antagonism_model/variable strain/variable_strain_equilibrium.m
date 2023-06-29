%% Solve the statics of a 3-muscle antagonistic arm under variable strain.
%%% Define the geometry of an individual segment
% Create the base segment
N_segments = 10;
l_0 = 0.5; % Default length

rho = 1 * 0.0254; % Define inter-muscle geometry
g_o_A = SE2.hat([0, rho, 0]);
g_o_o = eye(3);
g_o_B = SE2.hat([0, -rho, 0]);
g_o_muscles = {g_o_A; g_o_o; g_o_B};

g_0_o = SE2.hat([0, 0, -pi/2]);
g_0_muscles = lmatmul_cell(g_0_o, g_o_muscles); % g_i(0) = g_o(0) * g_oi

base_segment = Arm2D(g_0_o, g_o_muscles, l_0, 'plot_unstrained', false);
base_segment.rho = rho;
base_segment.n_spacers = 2;

% Create the discretized variable strain arm
arm = variable_strain_segment(N_segments, base_segment);

% Test setting the twist vectors along the rod
%% Solve the statics of the 3-muscle arm
f_force_outer = @(strain, pressure) actuatorForce_key(strain, pressure);
f_force_inner = @(strain, pressure) actuatorForce_key(strain, pressure);
force_funcs = {f_force_outer, f_force_inner, f_force_outer};

pressures = [0, 0, 0];
A = [
    1, 1, 1;
    0, 0, 0;
    rho, 0, -rho;
];

Q = [1; 0; 0];

g_circ_right_initial = zeros(3, N_segments);
g_circ_right_initial(1, :) = l_0;
f_equilibrium = @(v_g_circ_right) check_equilibrium_with_shear(v_g_circ_right, Q, arm, force_funcs, pressures, l_0, A);
equilibrium_soln = fsolve(f_equilibrium, g_circ_right_initial);

arm.set_base_curve(equilibrium_soln);

%%% Plot the arm
ax = axes(figure());
border_length_cm = 45;
axis equal
arm.plot(ax);

function v_residuals = check_equilibrium_with_shear(v_g_circ_right, Q, arm_obj, force_funcs, pressures, l_0, A)
    N_segments = length(arm_obj.arms);
    g_circ_right = reshape(v_g_circ_right, [3, N_segments]);
    arm_obj.set_base_curve(g_circ_right);

    reaction_forces = calc_reaction_forces(arm_obj, Q);

    internal_forces = zeros(3, N_segments);
    for i = 1 : N_segments
        segment = arm_obj.arms{i};
        forces_i = zeros(length(segment.muscles), 1);
        for j = 1 : length(segment.muscles)
            muscle_j = segment.muscles(j);
            epsilon_j = (muscle_j.l - l_0) / l_0;
            forces_i(j) = force_funcs{j}(epsilon_j, pressures(j));
        end
        internal_forces(:, i) = A * forces_i;
    end

    mat_residuals = internal_forces + reaction_forces;
    mat_residuals(2, :) = g_circ_right(2, :);
    v_residuals = mat_residuals(:);
end