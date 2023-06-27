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
                g_tip_i = SE2.hat(obj.arms{i}.muscle_o.calc_posns());
                g_o_i = obj.arms{i}.g_o;
                tform_along_i = inv(g_o_i) * g_tip_i;
                g_0_muscles_i = {obj.arms{i}.muscles.g_0};
                
                g_o_next = obj.arms{i}.g_o * tform_along_i;
                g_0_muscles_next = rmatmul_cell(g_0_muscles_i, tform_along_i);
            
                obj.arms{i+1} = Arm2D(g_o_next, g_0_muscles_next, l_0_seg, 'plot_unstrained', false);
                obj.arms{i+1}.rho = base_arm_obj.rho;
                obj.arms{i+1}.n_spacers = base_arm_obj.n_spacers;
            end
        end
        
        function plot(obj)
            ax = axes(figure());

            for i = 1 : length(obj.arms)
                obj.arms{i}.plot_arm(ax);
            end
        end
    end
end

