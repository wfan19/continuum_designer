%% Environment for validating a jacobian for multisegment continuum arms
% Calculate the forward kinematics of the arm
q_0 = [1, 0, 1, 0, 1, 0]';
n_segments = length(q_0) / 2;
n_timestamps = 10;
curvature_base = linspace(0, 0.5, n_timestamps);
qs = repmat(q_0, 1, n_timestamps);
qs(2, :) = curvature_base;

g_tip_poses = zeros(3, n_timestamps);

% Compute forward kinematics for each configuration vector, and then plot
% the results.
arms = PCCArm.empty(0, n_timestamps);
for i = 1 : size(qs, 2)
    q_i = qs(:, i);
    arms(i) = PCCArm(n_segments);
    g_tip_poses(:, i) = arms(i).update(q_i);
end

%%% Validate the jacobian
g_tips_predicted = zeros(3, n_timestamps);
g_tips_predicted(:, 1) = g_tip_poses(:, 1);
for i = 1 : size(qs, 2) - 1
    % Given the step in configuration, find the change in pose predicted by
    % the jacobian
    q_i = qs(:, i);
    delta_q = qs(:, i+1) - qs(:, i);
    delta_g_predicted = pcc_jacobian(q_i) * delta_q;
    g_tip_next_predicted = g_tips_predicted(:, i) + delta_g_predicted;

    % Retrieve the actual change in pose
    g_tip_next_actual = g_tip_poses(:, i+1);
    
    % Compare the two
    g_error = g_tip_next_actual - g_tip_next_predicted;
    fprintf("g(%i) - g_predicted(%i): [%.3f %.3f %.3f]\n", i+1, i+1, g_error(1), g_error(2), g_error(3))

    g_tips_predicted(:, i+1) = g_tip_next_predicted;
end

%%% Plotting
% Plot the arms
ax = axes(figure());
for i = 1 : size(qs, 2)
    arms(i).plot_arm(ax)
end

% Plot the predicted vs actual tip trajectory
hold(ax, 'on')
plot(g_tips_predicted(1, :), g_tips_predicted(2, :), "bx")
plot(g_tip_poses(1, :), g_tip_poses(2, :), "rx")

%% Jacobian function implementation
function mat_jacobian = pcc_jacobian(q)
    mat_jacobian = zeros(3, length(q));
end
