classdef variable_strain_segment
    %VARIABLE_STRAIN_SEGMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        arms
    end
    
    methods
        function obj = variable_strain_segment(N_segments, base_arm_obj)
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
        
        function plot(obj, ax)
            arguments
                obj
                ax = axes(figure());
            end
            for i = 1 : length(obj.arms)
                obj.arms{i}.plot_arm(ax);
            end
        end

        function g_tip = get_tip_pose(obj)
            g_tip = SE2.hat(obj.arms{end}.muscle_o.calc_posns());
        end
    end
end

