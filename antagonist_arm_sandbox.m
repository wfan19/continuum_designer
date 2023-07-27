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
    Q = [0; -5; 0];

    arm_series.solve_equilibrium_gina([0; 0; 0; 0;], Q);

    %% Initialize the interactive UI
    fig = uifigure;
    fig.Name = "2D Antagonistic Arm Sandbox";

    top_gl = uigridlayout(fig);
    top_gl.ColumnWidth = {'3x', '2x'};
    top_gl.RowHeight = {'3x', '2x'};

    % Initalize the plotting axis
    ax = uiaxes(top_gl);
    ax.Layout.Column = 1;
    ax.Layout.Row = 1;
    Plotter2D.plot_arm_series(arm_series, ax);
    
    % Create the sliders for controlling actuators
    N_rods = length(arm_series.segments(1).rods);
    actuator_gl = uigridlayout(top_gl, [2, N_rods]);
    actuator_gl.RowHeight = {'3x', '1x'};
    actuator_gl.Layout.Column = 1;
    actuator_gl.Layout.Row = 2;

    actuator_sliders = cell(1, N_rods);
    actuator_slider_labels = cell(1, N_rods);
    for i = 1 : N_rods
        actuator_sliders{i} = uislider(actuator_gl);
        actuator_sliders{i}.ValueChangedFcn = {@on_pressure_change};
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
    function on_pressure_change(src, event)
        % Show loading animation on mouse pointer
        fig = ancestor(src, "figure");
        set(fig, "Pointer", "watch");
        drawnow;

        % Update the arm
        pressures_temp = cellfun(@(slider)slider.Value, actuator_sliders);
        pressures = pressures_temp(:); % Make sure it's a column vector
        arm_series.solve_equilibrium_gina(pressures, Q);
        cla(ax);
        Plotter2D.plot_arm_series(arm_series, ax);

        % Revert mouse pointer back to normal state
        set(fig, "Pointer", "arrow");
    end

    %% Create sliders for tweaking arm design parameters
    arm_designer_layout = uigridlayout(top_gl, [2, 1]);
    arm_designer_layout.ColumnWidth = {'1x', '3x'};
    arm_designer_layout.Layout.Row = 1;
    arm_designer_layout.Layout.Column = 2;

    arm_designer_sliders = cell(1, 2);
    arm_designer_slider_labels = cell(1, 2);
    limits = [
        0.02, 0.12;
        0.01, 0.12,
    ];
    names = ["R outer base", "R outer tip"];
    for i = 1 : length(arm_designer_sliders)
        arm_designer_slider_labels{i} = uilabel(arm_designer_layout);
        arm_designer_slider_labels{i}.Text = names(i);
        arm_designer_slider_labels{i}.Layout.Column = 1;
        arm_designer_slider_labels{i}.Layout.Row = i;
        
        arm_designer_sliders{i} = uislider(arm_designer_layout, "limits", limits(i, :));
        arm_designer_sliders{i}.ValueChangedFcn = {@on_width_change};
        arm_designer_sliders{i}.Layout.Column = 2;
        arm_designer_sliders{i}.Layout.Row = i;
    end    

    function on_width_change(src, event)
        fig = ancestor(src, "figure");
        set(fig, "Pointer", "watch");
        drawnow

        design_params = cellfun(@(slider) slider.Value, arm_designer_sliders);
        
        rho_outer_base = design_params(1);
        rho_outer_tip = design_params(2);

        rho_inner_base = 1/3 * rho_outer_base;
        rho_inner_tip = 1/3 * rho_outer_tip;
        arm_series_new = ArmSeriesFactory.tapered_2d_antagonist_arm( ...
            N_segments, rho_inner_base, rho_outer_base, rho_inner_tip, rho_outer_tip, l_0 ...
        );
        arm_series_new.set_mechanics(BasicPolyBellowMechanics(l_0), [1, 4]);
        arm_series_new.set_mechanics(GinaMuscleMechanics(l_0), [2, 3]);
        arm_series_new.solve_equilibrium_gina(pressures, Q);
        cla(ax);
        Plotter2D.plot_arm_series(arm_series_new, ax);
        
        arm_series = arm_series_new;

        set(fig, "Pointer", "arrow");
    end

end