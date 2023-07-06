classdef variable_strain_segment < matlab.mixin.Copyable
    %VARIABLE_STRAIN_SEGMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        arms

        l_0 = 1;

        force_funcs % Cell array of the force surface functions for each individual muscle
    end
    
    methods
        function obj = variable_strain_segment(N_segments, base_arm_obj, force_funcs)
            % Create individual segment object: same as the overall arm but
            % each segment only integrates 1/N
            ds = 1/N_segments;
            segment_arm = copy(base_arm_obj);
            segment_arm.muscle_o.max_s = ds;
            for i = 1 : length(segment_arm.muscles)
                segment_arm.muscles(i).max_s = ds;
            end

            % Create a list of individiual segment objects
            obj.arms = cell(1, N_segments); 
            for i = 1 : length(obj.arms)
                obj.arms{i} = copy(segment_arm);
            end

            % Set the arm into a straight neutral config
            % This also places the segments where they should go.
            l_0_full = base_arm_obj.muscle_o.l;
            neutral_base_curve = zeros(3, N_segments);
            neutral_base_curve(1, :) = l_0_full;
            obj.set_base_curve(neutral_base_curve);
            obj.l_0 = l_0_full;

            obj.force_funcs = force_funcs;
        end

        function set_base_curve(obj, g_circ_right)
            for i = 1 : length(obj.arms) - 1
                % First update the twist vector
                obj.arms{i}.set_base_curve(g_circ_right(:, i));
            
                % Now move the next arm's base poses according to the twist vector of
                % this one.
                g_o_next = SE2.hat(obj.arms{i}.muscle_o.calc_posns());
                obj.arms{i+1}.g_o = g_o_next;
                
                for j = 1 : length(obj.arms{i+1}.muscles)
                    muscle_j = obj.arms{i+1}.muscles(j);
                    muscle_j.g_0 = obj.arms{i+1}.g_o * muscle_j.g_o_i;
                end
            end
            obj.arms{end}.set_base_curve(g_circ_right(:, end));
        end

        function g_circ_right = get_base_curve(obj)
            g_circ_right = zeros(3, length(obj.arms));

            for i = 1 : length(obj.arms)
                g_circ_right(:, i) = obj.arms{i}.muscle_o.h_tilde;
            end
        end

        function mat_reactions = calc_reaction_forces(obj, Q)
            g_tip = obj.get_tip_pose();
        
            mat_reactions = zeros(3, length(obj.arms));
            for i = 1 : length(obj.arms)
                % For each segment, calculate the reaction forces at its base
                segment = obj.arms{i};
                g_i = segment.g_o;
                g_i_tip = inv(g_i) * g_tip;        
        
                Q_right_circ_tip = SE2.left_lifted_action(g_i)' * Q;
                Q_right_circ_i = SE2.adjoint(g_i_tip)' * Q_right_circ_tip;
        
                mat_reactions(:, i) = Q_right_circ_i;
            end
        end

        function internal_forces = calc_internal_forces(obj, pressures)
            N_segments = length(obj.arms);
            internal_forces = zeros(3, N_segments);
            for i = 1 : N_segments
                segment = obj.arms{i};
                N_muscles = length(segment.muscles);
                forces_i = zeros(N_muscles, 1);
        
                % Find the strain and thus corresponding force in each muscle
                for j = 1 : N_muscles
                    muscle_j = segment.muscles(j);
                    epsilon_j = (muscle_j.l - obj.l_0) / obj.l_0;
                    forces_i(j) = obj.force_funcs{j}(epsilon_j, pressures(j));
                end
        
                % Use the segment geometry to build the equilibrium matrix
                % TODO: this should be part of an arm or segment?
                g_o_muscles = {segment.muscles.g_o_i};
                t_o_muscles = cell2mat(cellfun(@(mat) SE2.translation(mat), g_o_muscles, "uniformoutput", false));
                dy_o_muscles = t_o_muscles(2, :);
                A = zeros(3, N_muscles);
                A(1, :) = 1;
                A(3, :) = dy_o_muscles;
        
                % Calculate the total internal force vector
                internal_forces(:, i) = A * forces_i;
            end
        end

        function v_residuals = check_equilibrium(obj, g_circ_right, Q, pressures)
            obj.set_base_curve(g_circ_right);
        
            reaction_forces = obj.calc_reaction_forces(Q);
            internal_forces = obj.calc_internal_forces(pressures);
        
            mat_residuals = internal_forces + reaction_forces;
            mat_residuals(2, :) = g_circ_right(2, :);
            v_residuals = mat_residuals(:);
        end

        function equilibrium_soln = solve_for_base_curve(obj, pressures, Q)
            arguments
                obj
                pressures
                Q = [0; 0; 0]
            end
            g_circ_right_initial = obj.get_base_curve(); % Should this initialize from neutral, or from the last state? Should we make the obj stateless???
            f_equilibrium = @(v_g_circ_right) obj.check_equilibrium(v_g_circ_right, Q, pressures);
            options = optimoptions('fsolve',"MaxFunctionEvaluations", 1e5);
            
            equilibrium_soln = fsolve(f_equilibrium, g_circ_right_initial, options);
            obj.set_base_curve(equilibrium_soln);
        end
        
        function plot(obj, ax, Q)
            arguments
                obj
                ax = axes(figure());
                Q = [0; 0; 0]
            end
            for i = 1 : length(obj.arms)
                obj.arms{i}.plot_arm(ax);
            end

            xy = SE2.translation(obj.get_tip_pose());
            uv = Q(1:2) * obj.l_0/10;
            quiver(ax, xy(1), xy(2), uv(1), uv(2), "off", "linewidth", 3, "MaxHeadSize", 1);
        end

        function g_tip = get_tip_pose(obj)
            g_tip = SE2.hat(obj.arms{end}.muscle_o.calc_posns());
        end

        function neutral_base_curve = get_neutral_base_curve(obj)
            % Get the matrix of g_circ_right at the arm's neutral state.
            N_segments = length(obj.arms);
            neutral_base_curve = zeros(3, N_segments);
            neutral_base_curve(1, :) = obj.l_0;
        end
    end

    % Copy constructor
    methods (Access = protected)

        % Copy constructor
        % Inherited from matlab.mixin.Copyable
        function cp = copyElement(obj)
            % Regular copy of all elements
            cp = copyElement@matlab.mixin.Copyable(obj);
            
            for i = 1 : length(cp.arms)
                cp.arms{i} = copy(obj.arms{i});
            end
        end
    end
end

