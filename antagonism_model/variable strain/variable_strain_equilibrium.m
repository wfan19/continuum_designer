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
pressures = [0, 40, 0, 20];
Q = [0; -1; 0];

% Initialize the problem
g_circ_right_initial = zeros(3, N_segments);
g_circ_right_initial(1, :) = l_0;
f_equilibrium = @(v_g_circ_right) check_equilibrium_with_shear(v_g_circ_right, Q, arm, force_funcs, pressures, l_0);
options = optimoptions('fsolve',"MaxFunctionEvaluations", 1e5);

tic
equilibrium_soln = fsolve(f_equilibrium, g_circ_right_initial, options);
toc

disp("Final residuals: ")
disp(f_equilibrium(equilibrium_soln));

arm.set_base_curve(equilibrium_soln);

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

function v_residuals = check_equilibrium_with_shear(v_g_circ_right, Q, arm_obj, force_funcs, pressures, l_0)
    N_segments = length(arm_obj.arms);
    g_circ_right = reshape(v_g_circ_right, [3, N_segments]);
    arm_obj.set_base_curve(g_circ_right);

    reaction_forces = calc_reaction_forces(arm_obj, Q);

    internal_forces = zeros(3, N_segments);
    for i = 1 : N_segments
        segment = arm_obj.arms{i};
        N_muscles = length(segment.muscles);
        forces_i = zeros(N_muscles, 1);

        % Find the strain and thus corresponding force in each muscle
        for j = 1 : N_muscles
            muscle_j = segment.muscles(j);
            epsilon_j = (muscle_j.l - l_0) / l_0;
            forces_i(j) = force_funcs{j}(epsilon_j, pressures(j));
        end

        % Use the segment geometry to build the equilibrium matrix
        % TODO: this should be part of an arm or segment?
        g_o_muscles = {segment.muscles.g_o_i};
        t_o_muscles = cell2mat(cellfun(@(mat) SE2.translation(mat), g_o_muscles, "uniformoutput", false));
        dy_o_muscles = t_o_muscles(2, :);
        A = zeros(3, N_muscles);
        A(1, :) = 1;
        A(3, :) = dy_o_muscles;

        % Calculate the total internal force vector
        internal_forces(:, i) = A * forces_i;
    end

    mat_residuals = internal_forces + reaction_forces;
    mat_residuals(2, :) = g_circ_right(2, :);
    v_residuals = mat_residuals(:);
end