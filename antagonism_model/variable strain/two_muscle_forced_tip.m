%% Solve the statics of a 3-muscle antagonistic arm under variable strain.
%%% Define the geometry of an individual segment
% Create the base segment
N_segments = 5;
l_0 = 0.5; % Default length

rho = 1 * 0.0254; % Define inter-muscle geometry
g_o_A = SE2.hat([0, rho, 0]);
g_o_o = eye(3);
g_o_B = SE2.hat([0, -rho, 0]);
g_o_muscles = {g_o_A; g_o_B};

g_0_o = SE2.hat([0, 0, pi/2]);
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
force_funcs = {f_force_outer, f_force_outer};

pressures = [30, 0, 0];
A = [
    1, 1;
    rho, -rho;
];

Q = [-1; 0; 0];

g_circ_right_initial = zeros(3, N_segments);
g_circ_right_initial(1, :) = l_0;
f_equilibrium = @(v_g_circ_right) check_equilibrium_with_shear(v_g_circ_right, Q, arm, force_funcs, pressures, l_0, A);
equilibrium_soln = fsolve(f_equilibrium, g_circ_right_initial(:));
equilibrium_soln = reshape(equilibrium_soln, [3, N_segments]);

% g_circ_right_initial = zeros(2, N_segments);
% g_circ_right_initial(1, :) = l_0;
% v_g_circ_right_inital = g_circ_right_initial(:);
% f_equilibrium = @(v_g_circ_right) check_equilibrium_shear_free(v_g_circ_right, Q, arm, force_funcs, pressures, l_0, A);
% equilibrium_soln = fsolve(f_equilibrium, v_g_circ_right_inital);
% equilibrium_soln = v_lks_to_mat_g_circ(equilibrium_soln);

arm.set_base_curve(equilibrium_soln);

%%% Plot the arm
ax = axes(figure());
border_length_cm = 45;
ylim(ax, [0, 1] * border_length_cm / 100);
xlim(ax, [-0.5, 0.5] * border_length_cm / 100);
arm.plot(ax);

function v_residuals = check_equilibrium_with_shear(v_g_circ_right, Q, arm_obj, force_funcs, pressures, l_0, A)
    N_segments = length(arm_obj.arms);
    g_circ_right = reshape(v_g_circ_right, [3, N_segments]);
    arm_obj.set_base_curve(g_circ_right);

    mat_residuals = zeros(3, N_segments);
    % For each point, solve the residual
    for i = 1 : N_segments
        arm_i = arm_obj.arms{i};

        % Compute current position, and distance vector to end point
        g_i = arm_i.g_o;
        g_end = arm_obj.get_tip_pose();
        delta_g_i = inv(g_i) * g_end;
        r_i = SE2.translation(delta_g_i);
        r_i = [r_i(1); r_i(2); 0];

        forces_i = zeros(length(arm_i.muscles), 1);
        for j = 1 : length(arm_i.muscles)
            % Compute internal reaction forces
            muscle_j = arm_i.muscles(j);
            epsilon_j = (muscle_j.l - l_0) / l_0;
            
            forces_i(j) = force_funcs{j}(epsilon_j, pressures(j));
        end

        residuals_i = A * forces_i;

        residuals_i(1) = residuals_i(1) + dot(Q(1:2), g_i(1:2, 1));

        moment_from_Qf = norm(cross(r_i, diag([1, 1, 0]) * Q));
        residuals_i(2) = moment_from_Qf + residuals_i(2) + Q(3);

        residuals_i(3) = g_circ_right(2, i);

        mat_residuals(:, i) = residuals_i;
    end

    v_residuals = mat_residuals(:);
end

function v_residuals = check_equilibrium_shear_free(v_q, Q, arm_obj, force_funcs, pressures, l_0, A)
    N_segments = length(arm_obj.arms);
    g_circ_right = v_lks_to_mat_g_circ(v_q);
    arm_obj.set_base_curve(g_circ_right);

    mat_residuals = zeros(2, N_segments);
    % For each point, solve the residual
    for i = 1 : N_segments
        arm_i = arm_obj.arms{i};

        % Compute current position, and distance vector to end point
        g_i = arm_i.g_o;
        g_end = arm_obj.get_tip_pose();
        delta_g_i = inv(g_i) * g_end;
        r_i = SE2.translation(delta_g_i);
        r_i = [r_i(1); r_i(2); 0];

        forces_i = zeros(length(arm_i.muscles), 1);
        for j = 1 : length(arm_i.muscles)
            % Compute internal reaction forces
            muscle_j = arm_i.muscles(j);
            epsilon_j = (muscle_j.l - l_0) / l_0;
            
            forces_i(j) = force_funcs{j}(epsilon_j, pressures(j));
        end

        residuals_i = A * forces_i;

        residuals_i(1) = residuals_i(1) + dot(Q(1:2), g_i(1:2, 1));

        moment_from_Qf = norm(cross(diag([1, 1, 0]) * Q, r_i));
        residuals_i(2) = moment_from_Qf + residuals_i(2) + Q(3);

        mat_residuals(:, i) = residuals_i;
    end

    v_residuals = mat_residuals(:);
end

function mat_g_circ_right = v_lks_to_mat_g_circ(v_lk)
    N_segments = length(v_lk)/2;
    mat_lk = reshape(v_lk, [2, N_segments]);
    mat_g_circ_right = [mat_lk(1, :); zeros(1, N_segments); mat_lk(2, :)];
end