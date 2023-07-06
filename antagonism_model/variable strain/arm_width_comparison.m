% Define a base-curve to maintain
% Values come from the equilibrium solution of a 1-inch radius arm
base_curve_l = 0.6893;
baes_curve_curvature = 1.8039;

% Create some arms
radii = linspace(0.02, 0.12, 10);
N_segments = 20;
N_muscles = 4;
l_0 = 0.75;
arms = cell(1, length(radii));
for i = 1 : length(radii)
    arms{i} = make_default_arm(radii(i), N_segments, l_0);
end

% Set the base curve of each arm
g_circ_right = zeros(3, N_segments);
g_circ_right(1, :) = base_curve_l;
g_circ_right(3, :) = baes_curve_curvature;

for i = 1 : length(arms)
    arms{i}.set_base_curve(g_circ_right);
end

fig = figure("position", [0, 0, 1600, 400]);
for i = 1 : length(arms)
    ax = subplot(1, length(arms), i);
    arms{i}.plot(ax);
end

% Reaction forces are equivalent across each arm - they only depend on
% the base-curve
Q = [0; -40; 0]; % Apply a 40N downward force
reaction_forces = arms{1}.calc_reaction_forces(Q);

% For each arm, calculate the muscle forces needed to counterbalance the
% reaction force
muscle_forces = zeros(N_muscles, N_segments, length(radii));
for i = 1 : length(arms)
    arms_i = arms{i};
    internal_forces = -reaction_forces;

    muscle_forces_i = lsqminnorm(arms_i.A, internal_forces);
    
    fprintf("\n\n Forces for muscle with radius %.3f: \n", radii(i));
    disp(muscle_forces_i)

    muscle_forces(:, :, i) = muscle_forces_i;
end

%% Plot all force curves for each muscle
s = linspace(0, 1, N_segments);
[S, R] = meshgrid(s, radii);

fig = figure("Position", [0, 0, 2000, 400]);
muscle_types = ["Bellow", "Muscle", "Muscle", "Bellow"];
full_zlim = [min(muscle_forces, [], "all"), max(muscle_forces, [], "all")];
for i = 1 : 4
    ax = subplot(1, 4, i);
    mesh(S, R, reshape(muscle_forces(i, :, :), N_segments, length(radii))')
    xlabel("Position along arm (%)")
    ylabel("Arm radius")
    zlabel("Actuator force")
    title(sprintf("Actuator %d - %s", i, muscle_types(i)))
    clim(full_zlim);
end
linkaxes(fig.Children, "z");

% linkaxes(fig.Children, "z");
% 
% for i = 1 : length(fig.Children)
%     clim(fig.Children(i), fig.Children(1).ZLim);
% end
% 
% linkaxes(fig.Children, "off")


fprintf("\n\n")