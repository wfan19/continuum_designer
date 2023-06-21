%% Environment for validating a jacobian for multisegment continuum arms
% Calculate the forward kinematics of the arm
g_base = SE2.hat([0, 0, 0]);
q_0 = [1, 0, 1, 0, 1, 0, 1, 0]';
n_segments = length(q_0) / 2;

% Create curvature sweep
n_timestamps = 60;
k_0 = 0;
delta_k = 0.5;
curvature_base = linspace(k_0, k_0 + delta_k, n_timestamps);
qs = repmat(q_0, 1, n_timestamps);
qs(2, :) = curvature_base;
qs(4, :) = (2*curvature_base).^2;
qs(8, :) = -curvature_base;

%%% Compute forward kinematics for each configuration vector, and then plot the results.
joint_poses = cell(1, n_segments);      % I'd use a 3d array here if matlab's 3d arrays were better :(
joint_poses(:) = {zeros(3, n_timestamps)};
arms = PCCArm.empty(0, n_timestamps);
for i_q = 1 : size(qs, 2)
    q_i = qs(:, i_q);
    arms(i_q) = PCCArm(n_segments, g_base);
    arms(i_q).update(q_i);
    for i_rod = 1 : n_segments
        joint_poses{i_rod}(:, i_q) = arms(i_q).rods(i_rod).calc_posns();
    end
end

% Plot the arms
ax = axes(figure());
for i = 1 : size(qs, 2)
    arms(i).plot_arm(ax)
end
hold(ax, 'on')

disp("============================")
joint_poses_predicted = cell(1, n_segments); % Lets save all the delta gs
%%% Validate the jacobian
for joint = 1 : n_segments
    % Matrix of the poses for this joint as we sweep the configuration
    g_joint_predicted = zeros(3, n_timestamps);
    g_joint_predicted(:, 1) = joint_poses{joint}(:, 1);
    
    fprintf("\n- - - - - - Joint %i - - - - - -\n", joint)
    for i = 1 : size(qs, 2) - 1
        % Given the step in configuration, find the change in pose predicted by the jacobian
        q_i = qs(:, i);
        delta_q = qs(:, i+1) - qs(:, i);
        delta_g_world = pcc_jacobian(arms(i), q_i, joint) * delta_q;

        % Apply the transformation
        last_g_joint = g_joint_predicted(:, i);
        g_joint_next_predicted = last_g_joint + delta_g_world;
        fprintf("delta_g(%i):\n", i)
        disp(delta_g_world)
    
        % Retrieve the actual change in pose
        g_joint_next_actual = joint_poses{joint}(:, i+1);
        
        % Compare the two
        g_error = g_joint_next_actual - g_joint_next_predicted;
        fprintf("g(%i) - g_predicted(%i): [%.3f %.3f %.3f]\n", i+1, i+1, g_error(1), g_error(2), g_error(3))
    
        g_joint_predicted(:, i+1) = g_joint_next_predicted;
    end

    joint_poses_predicted{joint} = g_joint_predicted;

    % Plot the predicted vs actual tip trajectory
    plot(g_joint_predicted(1, :), g_joint_predicted(2, :), "bx")
    plot(joint_poses{joint}(1, :), joint_poses{joint}(2, :), "rx")
end