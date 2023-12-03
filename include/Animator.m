classdef Animator < handle
    %% Animation creator utility
    
    properties
        fig             % Matlab figure object
        i_frame = 1;    % Current frame counter
        n_frames = 1;   % Total number of frames to be cycled through
        frames;         % 4D array of current frames
        color_map;      % Color map vector
    end
    
    methods
        %% Constructor
        function obj = Animator(fig, n_frames)
            obj.fig = fig;
            obj.n_frames = n_frames;
        end
        
        %% Capture figure's current frame to frame buffer
        function obj = get_frame(obj)
            frame = getframe(obj.fig);
            if obj.i_frame == 1
                posn = get(obj.fig, "Position");
                width = posn(3);
                height = posn(4);
                
                % Preallocate data (for storing frame data)
                obj.frames = zeros(height, width, 1, length(obj.n_frames), 'uint8');
                [obj.frames(:, :, 1, obj.i_frame), obj.color_map] = ...
                    rgb2ind(frame.cdata, 256, 'nodither');
            else
                obj.frames(:,:,1, obj.i_frame) = ...
                    rgb2ind(frame.cdata, obj.color_map, 'nodither');
            end
            obj.i_frame = obj.i_frame + 1;
        end
        
        %% Write frame buffer to a gif file in the /media directory
        function obj = save_gif(obj, file_name, options)
            arguments
                obj
                file_name = datestr(now);
                options.frame_rate = 24;
            end
            delay_time = 1/options.frame_rate;
            imwrite(obj.frames, obj.color_map, sprintf("figures/%s.gif", file_name), ...
                'DelayTime', delay_time, 'LoopCount', inf);
        end
        
        %% Reset the animator
        function obj = reset(obj)
            obj.i_frame = 1;
            obj.frames = [];
            obj.color_map = [];
        end
    end
end

