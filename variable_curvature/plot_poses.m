function plot_poses(poses, ax, linestyle)
    arguments
        poses
        ax
        linestyle="k-";
    end

    axis_length = 0.15;

    % Extract position components
    p_x = squeeze(poses(1, 3, :));
    p_y = squeeze(poses(2, 3, :));

    % Extract x unit vector components
    x_u = squeeze(poses(1, 1, :));
    x_v = squeeze(poses(2, 1, :));

    y_u = squeeze(poses(1, 2, :));
    y_v = squeeze(poses(2, 2, :));

    % Plot
    hold(ax, "on")
    plot(ax, p_x, p_y, linestyle, "linewidth", 2);
    quiver(ax, p_x, p_y, x_u, x_v, axis_length, "red", "linewidth", 2);
    quiver(ax, p_x, p_y, y_u, y_v, axis_length, "green", "linewidth", 2);
    axis(ax, "equal")
end