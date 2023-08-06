function antagonist_arm_sandbox
    %% First we create the arm
    % TODO: This is copy paste and will be refactored
    l_0 = 0.5;
    rho_outer = 0.0254;
    rho_inner = rho_outer * 1/3;

    N_segments = 5;
    arm_series = ArmSeriesFactory.tapered_2d_antagonist_arm( ...
        N_segments, rho_inner, rho_outer, rho_inner, rho_outer, l_0 ...
    );
    arm_series.set_mechanics(BasicPolyBellowMechanics(l_0), [1, 4]);
    arm_series.set_mechanics(GinaMuscleMechanics(l_0), [2, 3]);

    % Define load scenario
    % TODO: Make this a slider
    Q = [0; 0; 0];

    arm_series.solve_equilibrium_gina([0; 0; 0; 0;], Q);

    %% Initialize the interactive UI
    fig = uifigure;
    fig.Name = "2D Antagonistic Arm Sandbox";
    fig.Position = [500, 500, 1200, 800];

    top_gl = uigridlayout(fig);
    top_gl.ColumnWidth = {'3x', '2x'};
    top_gl.RowHeight = {'3x', '2x'};

    % Initalize the plotting axis
    ax = uiaxes(top_gl);
    ax.Layout.Column = 1;
    ax.Layout.Row = 1;
    Plotter2D.plot_arm_series(arm_series, ax);
    
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
        % Show loading animation on mouse pointer
        fig = ancestor(src, "figure");
        set(fig, "Pointer", "watch");
        drawnow;

        % Fetch each slider's pressure value and store in a list
        pressures_temp = cellfun(@(slider)slider.Value, actuator_sliders);
        pressures = pressures_temp(:); % Make sure it's a column vector

        % Solve the equilibrium with the new pressure values
        cla(ax);
        g_circ_right_eq = arm_series.solve_equilibrium_gina(pressures, Q);
        Plotter2D.plot_arm_series(arm_series, ax);

        % Revert mouse pointer back to normal state
        set(fig, "Pointer", "arrow");
    end

    %% Create sliders for tweaking arm design parameters
    % Create the layout that will hold the sliders
    arm_designer_layout = uigridlayout(top_gl, [2, 2]);
    arm_designer_layout.ColumnWidth = {'1x', '3x'};
    arm_designer_layout.Layout.Row = 1;
    arm_designer_layout.Layout.Column = 2;
    % TODO: Sliders for changing the inner radius as well

    % Create the sliders one by one from a list of names and ranges
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
        % Set loading animation for mouse pointer
        fig = ancestor(src, "figure");
        set(fig, "Pointer", "watch");
        drawnow

        % Fetch the values of all the design sliders as a list
        design_params = cellfun(@(slider) slider.Value, arm_designer_sliders);
        
        % Unpack the design params into individual values
        rho_outer_base = design_params(1);
        rho_outer_tip = design_params(2);

        % Create a new arm with the design params
        rho_inner_base = 1/3 * rho_outer_base;
        rho_inner_tip = 1/3 * rho_outer_tip;
        arm_series_new = ArmSeriesFactory.tapered_2d_antagonist_arm( ...
            N_segments, rho_inner_base, rho_outer_base, rho_inner_tip, rho_outer_tip, l_0 ...
        );
        arm_series_new.set_mechanics(BasicPolyBellowMechanics(l_0), [1, 4]);
        arm_series_new.set_mechanics(GinaMuscleMechanics(l_0), [2, 3]);
        
        % Solve the new equilibrium and plot the arm again
        cla(ax);
        arm_series_new.solve_equilibrium_gina(pressures, Q);
        Plotter2D.plot_arm_series(arm_series_new, ax);
        
        % Save the new arm as the current arm
        arm_series = arm_series_new;

        % Set the mouse pointer back to normal
        set(fig, "Pointer", "arrow");
        drawnow;
    end

    %% Create sliders for changing the load scenario
    loading_layout = uigridlayout(top_gl, [2, 1]);
    loading_layout.ColumnWidth = {'1x', '3x'};
    loading_layout.Layout.Row = 2;
    loading_layout.Layout.Column = 2;

    vertical_load_label = uilabel(loading_layout);
    vertical_load_label.Text = "Vertical load (N)";
    vertical_load_label.Layout.Column = 1;
    vertical_load_label.Layout.Row = 1;

    vertical_load_slider = uislider(loading_layout, "limits", [0, 40]);
    vertical_load_slider.ValueChangedFcn = {@on_load_change};
    vertical_load_slider.Layout.Row = 1;
    vertical_load_slider.Layout.Column = 2;

    function on_load_change(src, event)
        fig = ancestor(src, "figure");
        set(fig, "Pointer", "watch")
        drawnow

        vertical_load = event.Value;
        Q = [0; -vertical_load; 0];
        
        cla(ax);
        arm_series.solve_equilibrium_gina(pressures, Q);
        Plotter2D.plot_arm_series(arm_series, ax);
        
        set(fig, "Pointer", "arrow");
        drawnow;
    end
end
