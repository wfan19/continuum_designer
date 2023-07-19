classdef variable_strain_segment < matlab.mixin.Copyable
    %VARIABLE_STRAIN_SEGMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        arms

        l_0 = 1;

        force_funcs % Cell array of the force surface functions for each individual muscle

        A
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

            % Create our A matrix
            % TODO: This should eventually be per-segment
            N_muscles = length(base_arm_obj.muscles);
            obj.A = zeros(3, N_muscles);
            g_o_muscles = base_arm_obj.g_o_muscles(:)'; % Ensure it's a row cell array
            for i = 1 : length(g_o_muscles)
                g_o_muscle_i = g_o_muscles{i};
                obj.A(:, i) = transpose(inv(SE2.adjoint(g_o_muscle_i))) * [1; 0; 0];
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
        
                % Transform the force from a world-frame force Q to ..?
                Q_right_circ_tip = SE2.left_lifted_action(g_tip)' * Q;

                % Compute the left-force at the tip, which is the same as
                % the left-force at the base.
                Q_left_circ_i = inv(SE2.adjoint(g_i_tip))' * Q_right_circ_tip;
                mat_reactions(:, i) = Q_left_circ_i;

                % Gina's formulation
                %{
                reaction_i = zeros(3, 1);
                r_i = SE2.translation(g_tip) - SE2.translation(g_i);
                r_i_body = SE2.translation(inv(g_i) * g_tip);
                moment_from_Qf = [0, 0, 1] * cross([r_i; 0], diag([1, 1, 0]) * Q);

                reaction_i(1:2) = g_i(1:2, 1:2)' * Q(1:2);
                reaction_i(3) = moment_from_Qf + Q(3);
                mat_reactions(:, i) = reaction_i;
                %}
            end
        end

        function mat_strains = get_muscle_strains(obj, pressures)
            N_segments = length(obj.arms);
            N_muscles = length(obj.arms{1}.muscles);
            mat_strains = zeros(N_muscles, N_segments);

            for i = 1 : N_segments
                arm_i = obj.arms{i};
                for j = 1 : N_muscles
                    % Solve for the muscle's free-contraction length
                    forcefunc_j = obj.force_funcs{j};
                    options = optimset("Display", "off");
                    neutral_strain = fsolve(@(strain) forcefunc_j(strain, pressures(j)), 0, options);
                    free_contraction_length = obj.l_0 * (1 + neutral_strain);

                    % I think strain should be calculated from
                    % free-contraction length, not neutral length?
                    mat_strains(j, i) = (arm_i.muscles(j).l - obj.l_0) / obj.l_0;
                end
            end
        end

        function mat_forces = get_muscle_forces(obj, pressures)
            mat_strains = obj.get_muscle_strains(pressures);
            mat_forces = zeros(size(mat_strains));

            N_muscles = length(obj.arms{1}.muscles);
            for i = 1 : N_muscles
                strains_i = mat_strains(i, :);
                f_force = @(e) obj.force_funcs{i}(e, pressures(i));
                mat_forces(i, :) = arrayfun(f_force, strains_i);
            end
        end

        function internal_forces = calc_internal_forces(obj, pressures)
            muscle_forces = obj.get_muscle_forces(pressures);
            internal_forces = obj.A * muscle_forces;
        end

        function mat_residuals = check_equilibrium(obj, g_circ_right, pressures, Q)
            obj.set_base_curve(g_circ_right);
        
            reaction_forces = obj.calc_reaction_forces(Q);
            internal_forces = obj.calc_internal_forces(pressures);
        
            mat_residuals = internal_forces + reaction_forces;
            mat_residuals(2, :) = g_circ_right(2, :);
        end

        function equilibrium_soln = solve_for_base_curve(obj, pressures, Q)
            arguments
                obj
                pressures
                Q = [0; 0; 0]
            end
            g_circ_right_initial = obj.get_base_curve(); % Should this initialize from neutral, or from the last state? Should we make the obj stateless???
            f_equilibrium = @(g_circ_right) obj.check_equilibrium(g_circ_right, pressures, Q);
            options = optimoptions('fsolve',"MaxFunctionEvaluations", 1e5);
            
            equilibrium_soln = fsolve(f_equilibrium, g_circ_right_initial, options);
            obj.set_base_curve(equilibrium_soln);

            disp("Solver residuals: ")
            disp(f_equilibrium(equilibrium_soln))
        end
        
        function plot(obj, ax, Q)
            arguments
                obj
                ax = axes(figure());
                Q = [0; 0; 0]
            end
            hold(ax, "on");
            xy = SE2.translation(obj.get_tip_pose());
            uv = Q(1:2) * obj.l_0/10;
            quiver(ax, xy(1), xy(2), uv(1), uv(2), "off", "linewidth", 3, "MaxHeadSize", 1);
            hold(ax, "off");

            for i = 1 : length(obj.arms)
                obj.arms{i}.plot_arm(ax);
            end
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

