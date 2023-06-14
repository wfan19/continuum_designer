%% Environment for validating a jacobian for multisegment continuum arms
% Calculate the forward kinematics of the arm
q_0 = [1, 0, 1, 0, 1, 0]';
n_timestamps = 10;
curvature_base = linspace(0, 0.5, n_timestamps);
qs = repmat(q_0, n_timestamps);
qs(2, :) = curvature_base;

ax = axes(figure());
for i = 1 : size(qs, 2)
    q_i = qs(:, i);
    arm_i = PCCArm(3);
    arm_i.update(q_i)
    arm_i.plot_arm(ax)
end

