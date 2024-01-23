function cell_g_circ_out = find_base_curve_use_heuristics(pose_base, tip_poses, tip_load, N_segs, f_cost)
    % Given a set of target tip poses and loads, find the base curves that
    % optimize a given cost function
    
    % Note: here we employ a specific data structure for efficiently
    % encoding shear-free base curve geometries for arms with multiple
    % segments reaching multiple target poses. We are forced to do this
    % to input the data into Matlab's optimization functions. Normally, I
    % would prefer to work with a cell-array with each element being each
    % arm-series' matrix of twist vectors (mat_g_circ_right). Hence the
    % conversion function.

    % For a base-curve planning problem for an m-segment arm reaching
    % n-targets, the geometry matrix G is defined as 
    % [
    %   l_11, l_12, l_13, ..., l_1m;
    %   k_11, k_12, k_13, ..., k_1m;
    %   l_21, l_22, l_23, ..., l_2m;
    %   k_21, k_22, k_23, ..., k_2m;
    %   ...
    %   l_n1, l_n2, ...,       l_nm;
    %   k_n1, k_n2, ...,       k_nm;
    % ]
    %
    % where l_ij and k_ij are respectively the length and curvature of
    % segment i in the arm reaching target j.
    %
    % Additionally, each column in a geometry matrix (the lengths and
    % curvatures describing the segments of a single arm) is a "geometry
    % vector". 
    %

    % ========== Old version =========
    % cell_g_circ = cell(1, size(tip_poses, 2));
    % mat_geom_out = zeros(N_segs*2, length(tip_poses));
    % 
    % for i_pose = 1 : size(tip_poses, 2)
    %     opts = optimoptions("fmincon", "algorithm", "interior-point", "display", "notify-detailed");
    %     f_constraint = @(v_in) base_curve_tip_constraint_v(v_in, Pose2.hat(pose_base), Pose2.hat(tip_poses(:, i_pose)));
    % 
    %     [soln, res] = fmincon(@base_curve_cost_v, v_geo_0, [], [], [], [], [], [], f_constraint, opts);
    %     cell_g_circ{i_pose} = v_geom_to_g_circ(soln);
    %     mat_geom_out(:, i_pose) = soln;
    % end


    %============ New ===========
    N_segs = 3;
    N_poses = size(tip_poses, 2);
    v_geo_0 = repmat([1; 0], N_segs, 1);
    mat_geo_0 = repmat(v_geo_0, 1, N_poses);

    % Create a simple arm (we don't care about its neutral length or width)
    % We only care about its base curve - using the arm object lets us
    % access the functions for computing reaction forces along the arm
    simple_arm = ArmSeriesFactory.constant_2d_muscle_arm(N_segs, 0.01, 1);

    % TODO: Create this with a script
    A_select_ls = -[
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0;
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
    ];
    b_zero = zeros(9, 1);

    [soln, res] = fmincon(f_cost, mat_geo_0, A_select_ls, b_zero, [], [], [], [], @base_curve_tip_constraint);
    cell_g_circ_out = mat_geom_to_g_circ(soln);

    function [ineq_residual, eq_residual] = base_curve_tip_constraint(mat_geom)
        % In:
        %   mat_geo: Geometry matrix describing lengths and curvatures of
        %            N_arms each with M segments
        % Out:
        %   ineq_residual: Inequality constraint residuals (= 0 )
        %   eq_residual: equality constraint residuals
        ineq_residual = 0;  % We don't have any inequality constraints

        N_poses = size(mat_geom, 2);
        N_segs = size(mat_geom, 1);
        eq_residual = zeros(3, N_poses);
        cell_g_circ_right = mat_geom_to_g_circ(mat_geom);
        for i_arm = 1 : length(cell_g_circ_right)
            % Fetch the twist matrix of the current arm series
            mat_g_circ_right_i = cell_g_circ_right{i_arm};

            % Integrate all the g_circ_rights/twists along the arm to find
            % the tip pose
            pose = Pose2.hat(pose_base);
            for j = 1 : N_segs
                pose = pose * Twist2.expm(mat_g_circ_right_i(:, j));
            end

            tip_pose = pose;
            g_target_i = Pose2.hat(tip_poses(:, i_arm));
            rdelta_pose = inv(tip_pose) * g_target_i;
            eq_residual(:, i_arm) = Twist2.vee(logm(rdelta_pose));
        end
    end
    
    function mat_g_circ_right = v_geom_to_g_circ(v_geom)
        % Convert a "geometry vector" to a matrix of twists
        % By "geometry vector" I mean [l_1; k_1; l_2; k_2; ...; l_n; k_n]
        n_g_circ = length(v_geom)/2;
        mat_l_k = reshape(v_geom, [2, n_g_circ]);
        lengths = mat_l_k(1, :);        % lengths
        shears = zeros(1, n_g_circ);    % shears
        curvatures = mat_l_k(2, :);     % curvatures
    
        mat_g_circ_right = [lengths; shears; curvatures];
    end

    function cell_g_circ_out = mat_geom_to_g_circ(mat_geom)
        % Convert a "geometry matrix" to a cell array of matrices of twists
        
        N_poses = size(mat_geom, 2);
        N_segs = size(mat_geom, 1) / 2;
        cell_g_circ_out = cell(1, N_poses);
        for i = 1 : N_poses
            v_geom_i = mat_geom(:, i);
            cell_g_circ_out{i} = v_geom_to_g_circ(v_geom_i);
        end
    end

end

