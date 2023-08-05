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

%% Plot just the bellow force curves:

bellow_forces = reshape(muscle_forces(1, :, :), N_segments, length(radii))';
bellow_forces_s0 = reshape(muscle_forces(1, 1, :), 1, length(radii));

% Create a smooth interpolation of the bellow force curve


ax = axes(figure());
mesh(ax, S, R, bellow_forces)
view(-35, 20)
grid on
xlabel("Position along arm (%)")
ylabel("Radius (m)")
zlabel("Bellow Force Needed(N)")

%% Plot just the bellow force curves (with contour)
ax = axes(figure());
hold on
mesh(ax, S, R, bellow_forces)
contour3(ax, S, R, bellow_forces, bellow_forces_s0([2:2:end - 1, end]), 'k', "linewidth", 2)
view(-35, 20)
grid on
xlabel("Position along arm (%)")
ylabel("Radius (m)")
zlabel("Bellow Force Needed(N)")

%% Plot bellow force curve and where constant taper would end up
r_base = 0.06;
r_tip_1 = 0.02;
r_tip_2 = 0.04;
r_tip_3 = 0.06;

n_points = N_segments;
rs_1 = linspace(r_tip_1, r_base, n_points);
rs_2 = linspace(r_tip_2, r_base, n_points);
rs_3 = linspace(r_tip_3, r_base, n_points);

fs_1 = 0;