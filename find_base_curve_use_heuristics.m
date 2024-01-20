function cell_g_circ_out = find_base_curve_use_heuristics(pose_base, tip_poses, tip_loads, N_segs, f_cost)
    % Given a set of target tip poses and loads, find the base curves that
    % optimize a given cost function
    
    % Note: here we employ a specific data structure for efficiently
    % encoding shear-free base curve geometries for arms with multiple
    % segments reaching multiple target poses.

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

    N_segs = 3;
    v_geo_0 = repmat([1; 0], N_segs, 1);
    
    cell_g_circ = cell(1, size(tip_poses, 2));
    mat_geom_out = zeros(N_segs*2, length(tip_poses));


    for i_pose = 1 : size(tip_poses, 2)
        opts = optimoptions("fmincon", "algorithm", "interior-point", "display", "notify-detailed");
        f_constraint = @(v_in) base_curve_tip_constraint_v(v_in, Pose2.hat(pose_base), Pose2.hat(tip_poses(:, i_pose)));

        [soln, res] = fmincon(@base_curve_cost_v, v_geo_0, [], [], [], [], [], [], f_constraint, opts);
        cell_g_circ{i_pose} = v_geom_to_g_circ(soln);
        mat_geom_out(:, i_pose) = soln;
    end


    %============ New ===========
    N_poses = size(tip_poses, 2);
    mat_geo_0 = repmat(v_geo_0, 1, N_poses);

    [soln, res] = fmincon(@base_curve_cost_sum_of_var_from_each_segment, mat_geo_0, [], [], [], [], [], [], @base_curve_tip_constraint);
    %[soln, res] = fmincon(@base_curve_cost_dist_of_all_ls, mat_geo_0, [], [], [], [], [], [], @base_curve_tip_constraint);
    cell_g_circ_out = mat_geom_to_g_circ(soln);


    %% Helper functions
    function v_cost = base_curve_cost_v(v_in)
        % Formulate cost term as the variance of the lengths
        mat_g_circ_right  = v_geom_to_g_circ(v_in);
    
        lengths = mat_g_circ_right(1, :);
        v_cost = std(lengths);
    end

    function cost = base_curve_cost_sum_of_var_from_each_arm(mat_geom)
        % Note: This seems to reproduce the results of individually
        % minimizing each arm
        cell_g_circ_right = mat_geom_to_g_circ(mat_geom);
        v_stds = zeros(length(cell_g_circ_right), 1);
        
        for i = 1 : length(cell_g_circ_right)
            lengths_i = cell_g_circ_right{i}(1, :);
            v_stds(i) = std(lengths_i);
        end
        cost = sum(v_stds);
    end

    function cost = base_curve_cost_sum_of_var_from_each_segment(mat_geom)
        % Note: This seems to reproduce the results of individually
        % minimizing each arm
        cell_g_circ_right = mat_geom_to_g_circ(mat_geom);
        v_stds = zeros(length(cell_g_circ_right), 1);
        
        mat_g_circ_right = cat(3, cell_g_circ_right{:});

        for i = 1 : length(cell_g_circ_right)
            v_stds(i) = std(mat_g_circ_right(1, i, :));
        end
        cost = sum(v_stds);
    end

    function cost = base_curve_cost_norm_of_variances(mat_geom)
        cell_g_circ_right = mat_geom_to_g_circ(mat_geom);
        v_cost = zeros(length(cell_g_circ_right), 1);
        
        for i = 1 : length(cell_g_circ_right)
            lengths_i = cell_g_circ_right{i}(1, :);
            v_cost(i) = std(lengths_i);
        end

        cost = norm(v_cost);
    end

    function cost = base_curve_cost_dist_of_all_ls(mat_geom)
        cell_g_circ_right = mat_geom_to_g_circ(mat_geom);
        all_twists = [cell_g_circ_right{:}];
        all_ls = all_twists(1, :);

        cost = std(all_ls);
    end

    function cost = base_curve_cost_variance_from_target(mat_geom)
        cell_g_circ_right = mat_geom_to_g_circ(mat_geom);
        all_twists = [cell_g_circ_right{:}];
        all_ls = all_twists(1, :);

        target_l = 0.17;
        dist_from_target_l = abs(all_ls - 0.17);
        cost = norm(dist_from_target_l);
    end
    
    function [ineq_residual, eq_residual] = base_curve_tip_constraint_v(v_in, base_pose, goal_pose)
        % Constraint function for determining whether a set of shearless twist
        % vectors actually reach a given goal pose. 
        %
        % To be used as an equality constraitn function in calls to `fmincon()`
        %
        % v_in: vector of lengths and curvatures [l_1, k_1, l_2, k_2, ...]^T
        n_twists = length(v_in) / 2;
        mat_g_circ_right = v_geom_to_g_circ(v_in);
    
        % Integrate all the g_circ_rights in order
        pose = base_pose;
        for j = 1 : n_twists
            pose = pose * Twist2.expm(mat_g_circ_right(:, j));
        end
    
        % Compute the residual for the equality constraint
        rdelta_pose = inv(pose) * goal_pose;
        eq_residual = Twist2.vee(logm(rdelta_pose));

        % Set the residual for the inequality constraint to 0 (we aren't
        % using it)
        ineq_residual = 0;
    end

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

