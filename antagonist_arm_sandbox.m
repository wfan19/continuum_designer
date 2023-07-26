function antagonist_arm_sandbox
    %% First we create the arm
    % TODO: This is copy paste and will be refactored
    l_0 = 0.5;
    rho = 0.0254;
    
    g_o_X = Pose2.hat([0, rho, 0]);
    g_o_A = Pose2.hat([0, rho * 1/3, 0]);
    g_o_B = Pose2.hat([0, -rho * 1/3, 0]);
    g_o_Y = Pose2.hat([0, -rho, 0]);
    g_o_rods = {g_o_X; g_o_A; g_o_B; g_o_Y};
    
    g_0_o = Pose2.hat([0, 0, -pi/2]);
    
    arm_segment = ArmSegment(Pose2, g_0_o, g_o_rods, l_0);
    
    arm_segment.rod_o.mechanics.l_0 = l_0; % Default length of the whole segment
    
    arm_segment.rods(1).mechanics = BasicPolyBellowMechanics(l_0);
    arm_segment.rods(2).mechanics = GinaMuscleMechanics(l_0);
    arm_segment.rods(3).mechanics = GinaMuscleMechanics(l_0);
    arm_segment.rods(4).mechanics = BasicPolyBellowMechanics(l_0);
    
    % TODO: Better way to create a series of segments.
    N_segments = 5;
    arm_segments = ArmSegment.empty(0, N_segments);
    for i = 1 : N_segments
        arm_segments(i) = copy(arm_segment);
    end
    arm_series= ArmSeries(arm_segments);

    % Define load scenario
    % TODO: Make this a slider
    Q = [0; -5; 0];

    arm_series.solve_equilibrium_gina([0; 0; 0; 0;], Q);

    %% Initialize the interactive UI
    fig = uifigure;
    fig.Name = "2D Antagonistic Arm Sandbox";
    
    gl = uigridlayout(fig,[2 2]);
    gl.RowHeight = {'fit','1x'};
    gl.ColumnWidth = {'fit','1x'};

    % Initalize the plotting axis
    ax = uiaxes(gl);
    Plotter2D.plot_arm_series(arm_series, ax);
    
    % Create the sliders for controlling actuators
    N_rods = length(arm_series.segments(1).rods);
    actuator_layout = uigridlayout(gl, [N_rods, 1]);
    actuator_sliders = cell(1, N_rods);
    for i = 1 : N_rods
        actuator_sliders{i} = uislider(actuator_layout);
        actuator_sliders{i}.ValueChangedFcn = {@on_pressure_change};
    end

    arm_designer_layout = uigridlayout(gl, [3, 1]);
    width_slider = uislider(arm_designer_layout, "limits",[0.01, 0.12]);
    width_slider.ValueChangedFcn = {@on_width_change};

    pressures = [0;0;0;0];
    
    function on_pressure_change(src, event)
        % Show loading animation on mouse pointer
        fig = ancestor(src, "figure");
        set(fig, "Pointer", "watch");
        drawnow;

        % Update the arm
        pressures_temp = [src.Parent.Children.Value];
        pressures = pressures_temp(:); % Make sure it's a column vector
        arm_series.solve_equilibrium_gina(pressures, Q);
        cla(ax);
        Plotter2D.plot_arm_series(arm_series, ax);

        % Revert mouse pointer back to normal state
        set(fig, "Pointer", "arrow");
    end

    function on_width_change(src, event)
        fig = ancestor(src, "figure");
        set(fig, "Pointer", "watch");
        drawnow

        l_0 = 0.5;
        rho = src.Value;
        
        g_o_X = Pose2.hat([0, rho, 0]);
        g_o_A = Pose2.hat([0, rho * 1/3, 0]);
        g_o_B = Pose2.hat([0, -rho * 1/3, 0]);
        g_o_Y = Pose2.hat([0, -rho, 0]);
        g_o_rods = {g_o_X; g_o_A; g_o_B; g_o_Y};
        
        g_0_o = Pose2.hat([0, 0, -pi/2]);
        
        arm_segment = ArmSegment(Pose2, g_0_o, g_o_rods, l_0);
        
        arm_segment.rod_o.mechanics.l_0 = l_0; % Default length of the whole segment
        
        arm_segment.rods(1).mechanics = BasicPolyBellowMechanics(l_0);
        arm_segment.rods(2).mechanics = GinaMuscleMechanics(l_0);
        arm_segment.rods(3).mechanics = GinaMuscleMechanics(l_0);
        arm_segment.rods(4).mechanics = BasicPolyBellowMechanics(l_0);
        
        % TODO: Better way to create a series of segments.
        arm_segments = ArmSegment.empty(0, N_segments);
        for i = 1 : N_segments
            arm_segments(i) = copy(arm_segment);
        end

        arm_series_new = ArmSeries(arm_segments);
        arm_series_new.g_circ_right = arm_series.g_circ_right;
        
        arm_series = arm_series_new;
        arm_series.solve_equilibrium_gina(pressures, Q);
        cla(ax);
        Plotter2D.plot_arm_series(arm_series, ax);

        set(fig, "Pointer", "arrow");
    end

end