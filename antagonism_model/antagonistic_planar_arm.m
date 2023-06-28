%% Parameterize the arm
rho = 1 * 0.0254; % M, RadiusSE
 
% Construct pose matrices
% Transform from world frame to base curve
g_o = SE2.hat([0, 0, pi/2]);

g_o_A = SE2.hat([0, rho, 0]);
g_o_B = SE2.hat([0, -rho, 0]);
g_o_muscles = {g_o_A, eye(3), g_o_B};
g_0_muscles = {g_o*g_o_A; g_o; g_o*g_o_B};

l_0 = 0.443; % Default length

% Create arm object and initialize plotting
planar_arm_obj = Arm2D(g_o, g_o_muscles, l_0, 'plot_unstrained', false);
planar_arm_obj.rho = rho;

fig = figure();
line_options_muscles = struct("LineWidth", 3);
line_options_spacers = struct("Linewidth", 2.5);
line_options_base_curve = struct("Linestyle", ":");

% Initialize plotting
planar_arm_obj.initialize_plotting(axes(fig), "line_options_muscles", line_options_muscles, ...
    "line_options_spacers", line_options_spacers, "line_options_base_curve", line_options_base_curve);

border_length_cm = 45;
ylim(planar_arm_obj.ax, [0, 1] * border_length_cm / 100);
xlim(planar_arm_obj.ax, [-0.5, 0.5] * border_length_cm / 100);

v_l_0 = l_0 * ones(length(g_o_muscles), 1);
test_h_tilde = [l_0 * 0.9; 0; 0.5]; 
planar_arm_obj.set_base_curve(test_h_tilde);

%% Solve the statics of the 3-muscle arm
f_force_outer = @(strain, pressure) actuatorForce_key(strain, pressure);
f_force_inner = @(strain, pressure) -actuatorForce_key(strain, pressure);
force_funcs = {f_force_outer, f_force_inner, f_force_outer};

pressures = [30, 0, 0];
A = [
    1 1 1;
    rho, 0, -rho;
];
check_equilibrium(test_h_tilde, planar_arm_obj, force_funcs, pressures, l_0, A);

f_equilibrium = @(h_o_tilde) check_equilibrium(h_o_tilde, planar_arm_obj, force_funcs, pressures, l_0, A);
equilibrium_soln = fsolve(f_equilibrium, [l_0; 0; 0]);

planar_arm_obj.set_base_curve(equilibrium_soln);
planar_arm_obj.plot_arm();

function residuals = check_equilibrium(h_o_tilde, arm_obj, force_funcs, pressures, l_0, A)
    forces = zeros(length(force_funcs), 1);

    arm_obj.set_base_curve(h_o_tilde);

    for i = 1 : length(force_funcs)
        h_i_tilde = arm_obj.muscles(i).h_tilde;
        epsilon_i = (h_i_tilde(1) - l_0) / l_0;

        forces(i) = force_funcs{i}(epsilon_i, pressures(i));
    end
    residuals = A * forces;
end