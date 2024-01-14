function cell_g_circ = find_base_curve_use_heuristics(pose_base, tip_poses, tip_loads, N_segs, f_cost)
    % Given a set of target tip poses and loads, find the base curves 

    N_segs = 3;
    twist_0 = repmat([1, 0], 1, N_segs);
    
    cell_g_circ = cell(1, size(tip_poses, 2));
    for i_pose = 1 : size(tip_poses, 2)
        opts = optimoptions("fmincon", "algorithm", "interior-point", "display", "notify-detailed");
        f_constraint = @(v_in) base_curve_tip_constraint(v_in, Pose2.hat(pose_base), Pose2.hat(tip_poses(:, i_pose)));

        [soln, ~] = fmincon(@base_curve_cost, twist_0, [], [], [], [], [], [], f_constraint, opts);
        cell_g_circ{i_pose} = v_geom_to_g_circ(soln);
    end

    function v_cost = base_curve_cost(v_in)
        % Formulate cost term as the variance of the lengths
        mat_g_circ_right  = v_geom_to_g_circ(v_in);
    
        lengths = mat_g_circ_right(1, :);
        v_cost = std(lengths);
    end
    
    function [ineq_residual, eq_residual] = base_curve_tip_constraint(v_in, base_pose, goal_pose)
        % Constraint function for determining whether a set of shearless twist
        % vectors actually reach a given goal pose. 
        %
        % To be used as an equality constraitn function in calls to `fmincon()`
        %
        % v_in: vector of lengths and curvatures [lambda_1, kappa_1, lambda_2, kappa_2, ...]
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
    
    function mat_g_circ_right = v_geom_to_g_circ(v_geom)
        % Convert a "geometry vector" to a matrix of twists
        % By "geometry vector" I mean [l_1, k_1, l_2, k_2, ..., l_n, k_n]
        n_g_circ = length(v_geom)/2;
        mat_lambda_kappa = reshape(v_geom, [2, n_g_circ]);
        lambdas = mat_lambda_kappa(1, :);
        kappas = mat_lambda_kappa(2, :);
    
        mat_g_circ_right = [lambdas; zeros(1,n_g_circ); kappas];
    end

end

