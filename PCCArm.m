classdef PCCArm
    %PCCARM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        rods
    end
    
    methods
        function obj = PCCArm(n_rods)
            color_blue = [100 143 255] / 255;
            color_red = [220 38 127] / 255;
            color_yellow = [255 176 0] / 255;
            
            colors = {color_blue, color_red, color_yellow};
            
            % Define the geometry of our 3-link continuum arm
            g_base = SE2.hat([0, 0, -pi/2]);
            
            % Create the set of rods in the multisegment arm
            default_length = 1;
            obj.rods = Muscle2D.empty(0, n_rods);
            for i = 1 : n_rods
                obj.rods(i)= Muscle2D(default_length, color=colors{mod(i, 3) + 1});
                obj.rods(i).g_0 = g_base;

                if i > 1
                    obj.rods(i).g_0 = SE2.hat(obj.rods(i-1).calc_posns());
                end
            end
        end
        
        function tip_pose = update(obj, q)
            % Calculate the forward kinematics of the arm
            f_map_l_k_to_twist_vector = @(l, k) [l; 0; k]; % Build the twist-vector for a constant-curvature rod, given length and curvature
            
            for i = 1 : length(obj.rods)
                q_i = q(2*i - 1 : 2*i);
                obj.rods(i).h_tilde = f_map_l_k_to_twist_vector(q_i(1), q_i(2));
                
                if i > 1
                    obj.rods(i).g_0 = SE2.hat(obj.rods(i-1).calc_posns());
                end
            end

            tip_pose = obj.rods(end).calc_posns();
        end

        function plot_arm(obj, ax, options)
            arguments
                obj
                ax = gca
                options.line_options = struct(LineWidth=5)
                options.tform_options = struct(FrameSize=0.1)
            end
            % Plot the rods
            axis(ax, 'equal')
            for i = 1 : length(obj.rods)
                obj.rods(i).plot_muscle(ax, line_options=options.line_options);
                obj.rods(i).plot_tforms(ax, resolution=10, plot_options=options.tform_options);
            end
        end
    end
end

