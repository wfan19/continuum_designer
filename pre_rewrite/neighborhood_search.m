%% Neighborhood search first for a 2-muscle arm
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

%% Let's visualize the arm with a target base curve
length_goal = 0.9 * arm.l_0;
curvature_goal = pi/2;

g_circ_right_goal = zeros(3, N_segments);
g_circ_right_goal(1, :) = length_goal;
g_circ_right_goal(3, :) = curvature_goal;

arm.set_base_curve(g_circ_right_goal);
g_tip_goal = arm.get_tip_pose();

ax = axes(figure());
arm.plot(ax);

%% Solve inverse mechanics for the unloaded arm: what are the pressures that get us to this shape?
% Method 1: just call an optimization routine?
% - Method 1a: minimize tip pose error
% Build the error function
Q = [0; -40; 0];
f_tip_error = @(pressures) tip_error(arm, g_tip_goal, Q, pressures); 

% TODO: Read Gina's Biomimetics & Bioinspiration

%% Do a stupid sweep of pressures - how nonconvex is our searchsapce?
N = 30;
p1 = linspace(0, 100, N);
p2 = linspace(0, 100, N);

[P1, P2] = meshgrid(p1, p2);
errors = zeros(N, N);
for i = 1 : N
    for j = 1 : N
        errors(i, j) = f_tip_error([P1(i, j); P2(i, j)]);
    end
end

%% Plot the results of the sweep
figure()
surf(P1, P2, errors);
xlabel("Pressure 1 (red)")
ylabel("Pressure 2 (blue)")
zlabel("Tip pose error")

% Plot the gradients too
figure()
[dP1, dP2] = gradient(errors, 0.2);
quiver(P1, P2, dP1, dP2);
xlabel("Pressure 1 (red)")
ylabel("Pressure 2 (blue)")

%%
options = optimoptions("fmincon", "DiffMinChange", 0.1);
pressures_optim = fmincon(f_tip_error, [30; 0], [], [], [], [], [0; 0], [100; 100]);

%% Now load the arm - how does the shape change?
Q = [0; -5; 0];
% Solve arm eq configuration under this new loading scenario

%% 
function error = tip_error(arm, g_tip_goal, Q, pressures)
    % Wrap the calls to the solver in evalc to avoid printing spam
    evalc("arm.solve_for_base_curve(pressures, Q);");
    g_tip = arm.get_tip_pose;

    K = diag([1, 1, 0.5]);
    v_error = se2.vee(logm(g_tip_goal * inv(g_tip)));
    error = v_error' * K * v_error;
end