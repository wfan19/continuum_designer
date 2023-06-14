%% Environment for validating a jacobian for multisegment continuum arms
color_blue = [100 143 255] / 255;
color_red = [220 38 127] / 255;
color_yellow = [255 176 0] / 255;

colors = {color_blue, color_red, color_yellow};

% Define the geometry of our 3-link continuum arm
g_base = SE2.hat([0, 0, -pi/2]);

% Create the set of rods in the multisegment arm
n_rods = 3;
default_length = 1;
rods = Muscle2D.empty(0, n_rods);
for i = 1 : n_rods
    rods(i)= Muscle2D(default_length, color=colors{i});
end

% Calculate the forward kinematics of the arm
q = [1, 0.5, 1, -0.3, 1, 0];
f_map_l_k_to_twist_vector = @(l, k) [l; 0; k]; % Build the twist-vector for a constant-curvature rod, given length and curvature

for i = 1 : length(rods)
    q_i = q(2*i - 1 : 2*i);
    rods(i).h_tilde = f_map_l_k_to_twist_vector(q_i(1), q_i(2));
    
    if i == 1
        rods(i).g_0 = g_base;
    else
        rods(i).g_0 = SE2.hat(rods(i-1).calc_posns());
    end
end

% Plot the rods
ax = axes(figure());
axis equal
line_options = struct(LineWidth=5);
tform_options = struct(FrameSize=0.1);
for i = 1 : length(rods)
    rods(i).plot_muscle(ax, line_options = line_options);
    rods(i).plot_tforms(ax, plot_options=tform_options);
end