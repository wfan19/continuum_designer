%% Solve the statics of a 3-muscle antagonistic arm under variable strain.
%%% Define the geometry of an individual segment
% Create the base segment
N_segments = 10;
l_0 = 0.5; % Default length

rho = 1 * 0.0254; % Define inter-muscle geometry
arm = make_default_arm(rho, N_segments, l_0);

%% Solve the statics of the 3-muscle arm

% Define the scenario
pressures = [0, 40, 0, 0];
Q = [0; -1; 0];

% Initialize the problem
arm.solve_for_base_curve(pressures, Q)

%% Plot the arm
ax = axes(figure());
border_length_cm = 45;
axis equal
hold on
arm.plot(ax, Q);