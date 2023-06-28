classdef variable_strain_segment
    %VARIABLE_STRAIN_SEGMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        arms
    end
    
    methods
        function obj = variable_strain_segment(N_segments, base_arm_obj)
            %VARIABLE_STRAIN_SEGMENT Construct an instance of this class
            %   Detailed explanation goes here
            obj.arms = cell(1, N_segments);
            obj.arms{1} = base_arm_obj;

            l_0_full = 0.443; % Default length
            l_0_seg = l_0_full / N_segments;

            % Now create N-segments based on this initial segment
            for i = 1 : N_segments - 1
                % The starting pose of the next arm is the tip pose of the
                % current arm.
                g_tip_i = SE2.hat(obj.arms{i}.muscle_o.calc_posns());
                obj.arms{i+1} = Arm2D( ...
                    g_tip_i, base_arm_obj.g_o_muscles, l_0_seg, 'plot_unstrained', false ...
                );

                % Copy over properties...
                obj.arms{i+1}.rho = base_arm_obj.rho;
                obj.arms{i+1}.n_spacers = base_arm_obj.n_spacers;
            end
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

