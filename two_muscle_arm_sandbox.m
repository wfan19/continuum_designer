function two_muscle_arm_sandbox
    %% First we create the arm
    % TODO: This is copy paste and will be refactored
    l_0 = 0.5;
    rho = 0.0254;

    N_segments = 5;
    arm_series = ArmSeriesFactory.tapered_2d_muscle_arm( ...
        N_segments, rho, rho, l_0 ...
    );
    arm_series.set_mechanics(GinaMuscleMechanics(l_0));

    % Define load scenario
    % TODO: Make this a slider
    Q = [1; 0; 0];

    %arm_series.solve_equilibrium_gina([0; 0], Q);


    Q = [0; 0; 0];
    fig_main_panel = uifigure;
    fig_main_panel.Name = "2D Antagonistic Arm Sandbox";
    fig_main_panel.Position = [500, 500, 1200, 800];

    top_gl = uigridlayout(fig_main_panel);
    top_gl.ColumnWidth = {'3x', '2x'};
    top_gl.RowHeight = {'3x', '2x'};

    % Initalize the plotting axis
    ax = uiaxes(top_gl);
    ax.Layout.Column = 1;
    ax.Layout.Row = 1;
    Plotter2D.plot_arm_series(arm_series, ax);

    function reload_sim()
        % Show loading animation on mouse pointer
        set(fig_main_panel, "Pointer", "watch");
        drawnow;

        % Solve the equilibrium with the new pressure values
        cla(ax);
        g_circ_right_eq = arm_series.solve_equilibrium_gina(pressures, Q, "frame", frame);
        Plotter2D.plot_arm_series(arm_series, ax);
        Plotter2D.plot_g_circ_right(arm_series, g_circ_panel)

        % Revert mouse pointer back to normal state
        set(fig_main_panel, "Pointer", "arrow");
    end
    
    %% Create the sliders for controlling actuators
    % First we create the layout to hold the actuator control sliders
    N_rods = length(arm_series.segments(1).rods);
    actuator_gl = uigridlayout(top_gl, [2, N_rods]);
    actuator_gl.RowHeight = {'3x', '1x'};
    actuator_gl.Layout.Column = 1;
    actuator_gl.Layout.Row = 2;

    % Next, for each rod in the arm we create a slider and a label for the
    % slider
    actuator_sliders = cell(1, N_rods);
    actuator_slider_labels = cell(1, N_rods);
    for i = 1 : N_rods
        actuator_sliders{i} = uislider(actuator_gl);
        actuator_sliders{i}.ValueChangedFcn = {@on_pressure_change}; % Attach the callback function to the slider
        actuator_sliders{i}.Orientation = "vertical";
        actuator_sliders{i}.Layout.Column = i;
        actuator_sliders{i}.Layout.Row = 1;

        actuator_slider_labels{i} = uilabel(actuator_gl);
        actuator_slider_labels{i}.Text = sprintf("P%d", i);
        actuator_slider_labels{i}.Layout.Column = i;
        actuator_slider_labels{i}.Layout.Row = 2;
        actuator_slider_labels{i}.HorizontalAlignment = "center";
    end

    pressures = [0;0;0;0];
    % Callback function for when an actuator control slider is changed
    function on_pressure_change(src, event)

        % Fetch each slider's pressure value and store in a list
        pressures_temp = cellfun(@(slider)slider.Value, actuator_sliders);
        pressures = pressures_temp(:); % Make sure it's a column vector

        reload_sim()
    end

    %% Create sliders for tweaking arm design parameters
    % Create the layout that will hold the sliders
    arm_designer_layout = uigridlayout(top_gl, [2, 2]);
    arm_designer_layout.ColumnWidth = {'1x', '3x'};
    arm_designer_layout.Layout.Row = 1;
    arm_designer_layout.Layout.Column = 2;
    
    % Create the sliders for design parameters
    arm_designer_sliders = cell(1, 2);
    arm_designer_slider_labels = cell(1, 2);
    slider_limits = [
        0.02, 0.12;
        0.01, 0.12,
    ];
    names = ["R outer base", "R outer tip"];
    for i = 1 : length(arm_designer_sliders)
        arm_designer_slider_labels{i} = uilabel(arm_designer_layout);
        arm_designer_slider_labels{i}.Text = names(i);
        arm_designer_slider_labels{i}.Layout.Column = 1;
        arm_designer_slider_labels{i}.Layout.Row = i;
        
        arm_designer_sliders{i} = uislider(arm_designer_layout, "limits", slider_limits(i, :));
        arm_designer_sliders{i}.ValueChangedFcn = {@on_width_change};
        arm_designer_sliders{i}.Layout.Column = 2;
        arm_designer_sliders{i}.Layout.Row = i;
    end    

    % Callback function that is called whenever the slider is dragged
    function on_width_change(src, event)
        N_segments = arm_series.N_segments;

        % Fetch the values of all the design sliders as a list
        design_params = cellfun(@(slider) slider.Value, arm_designer_sliders);
        
        % Unpack the design params into individual values
        rho_base = design_params(1);
        rho_tip = design_params(2);
        arm_series_new = ArmSeriesFactory.tapered_2d_muscle_arm(N_segments, rho_base, rho_tip, l_0);
        arm_series_new.set_mechanics(GinaMuscleMechanics(l_0));
        for j = 1 : length(arm_series.segments(1).rods)
            arm_series_new.set_mechanics(arm_series.segments(1).rods(j).mechanics, j);
        end
        arm_series = arm_series_new;
        reload_sim();
    end
    % TODO: Sliders for changing the inner radius as well

    %% Create sliders for changing the load scenario
    loading_layout = uigridlayout(top_gl, [4, 2]);
    loading_layout.ColumnWidth = {'1x', '3x'};
    loading_layout.Layout.Row = 2;
    loading_layout.Layout.Column = 2;
    
    load_labels = ["X load (N)", "Y load (N)", "Moment (Nm)"];
    load_lims = [
        -10, 10;
        -10, 10;
        -5, 5;
    ];

    cell_load_labels = cell(1, length(load_labels));
    cell_load_sliders = cell(1, length(load_labels));
    for i = 1 : length(cell_load_labels)
        cell_load_labels{i} = uilabel(loading_layout);
        cell_load_labels{i}.Text = load_labels(i);
        cell_load_labels{i}.Layout.Column = 1;
        cell_load_labels{i}.Layout.Row = i;
    
        cell_load_sliders{i} = uislider(loading_layout, "limits", load_lims(i, :));
        cell_load_sliders{i}.ValueChangedFcn = {@on_load_change};
        cell_load_sliders{i}.Layout.Row = i;
        cell_load_sliders{i}.Layout.Column = 2;
    end

    load_type_label = uilabel(loading_layout);
    load_type_label.Text = "Frame of load";
    load_type_label.Layout.Column = 1;
    load_type_label.Layout.Row = 4;

    frame = "World";
    load_type_switch = uiswitch(loading_layout);
    load_type_switch.Items = {'World', 'Tip'};
    load_type_switch.Layout.Column = 2;
    load_type_switch.Layout.Row = 4;
    load_type_switch.ValueChangedFcn = {@on_load_change};

    function on_load_change(src, event)
        loads = cellfun(@(slider)slider.Value, cell_load_sliders);
        Q = loads(:); % Make sure it's a column vector

        frame = load_type_switch.Value;
        reload_sim()
    end

    %% Create panel that houses the detailed g_circ and strain force surface plots
    fig_plots = uifigure;
    fig_plots.Position = [300, 300, 1500, 500];
    fig_gl = uigridlayout(fig_plots, [1,1]);
    fig_gl.ColumnWidth = {'fit'};
    fig_gl.RowHeight = {'fit'};
    
    %g_circ_panel = uipanel(fig_gl);
    %g_circ_panel.AutoResizeChildren = "off";
    g_circ_panel = fig_gl;
    Plotter2D.plot_g_circ_right(arm_series, g_circ_panel)
end
