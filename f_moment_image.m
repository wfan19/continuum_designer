function f_moment_image(f_1, f_2, neutral_lengths, widths, g_circ_right_goal, options)

    arguments
        f_1
        f_2
        neutral_lengths
        widths
        g_circ_right_goal
        options.fig = 0
        options.N_widths = 20
        options.N_neutral_lengths = 20
    end

    if length(widths) == 2
        widths = linspace(widths(1), widths(2), options.N_widths);
    end

    if length(neutral_lengths) == 2
        neutral_lengths = linspace(neutral_lengths(1), neutral_lengths(2), options.N_neutral_lengths);
    end

    N_w = options.N_widths;
    N_l_0 = options.N_neutral_lengths;

    cell_wl = cell(N_w, N_l_0);
    cell_arm_segments = cell(N_w, N_l_0);
    cell_rxns = cell(N_w, N_l_0);
    cell_colors = cell(N_w, N_l_0);
    for i = 1 : N_w
        rho_i = widths(i);
        for j = 1 : N_l_0
            l_0_j = neutral_lengths(j);
            % Calculate the internal reaction moment that the muscles would create
            % at that curvature.
            segment_ij = ArmSegmentFactory.make_2d_2muscle(rho_i, l_0_j);
            segment_ij.set_mechanics(RodMechanicsBase(l_0_j, @(s, p) f_1(s)), 1);
            segment_ij.set_mechanics(RodMechanicsBase(l_0_j, @(s, p) f_2(s)), 2);
        
            cell_arm_segments{i, j} = segment_ij;
            cell_wl{i, j} = [rho_i; l_0_j];
            placeholder_p = [0;0];
            cell_rxns{i, j} = segment_ij.calc_internal_reaction(placeholder_p, g_circ_right_goal);
            cell_colors{i, j} = [i/N_w * 0.8; j/N_l_0 * 0.75 + 0.25; 1];
        end
    end
    
    mat_rl = [cell_wl{:}];
    mat_rxns = [cell_rxns{:}];
    mat_hsv = [cell_colors{:}];
    mat_rgb = hsv2rgb(mat_hsv');

    if strcmp(class(options.fig), "matlab.ui.Figure")
        subplot(1, 2, 1, "parent", options.fig);
        scatter(mat_rl(1, :), mat_rl(2, :), [], mat_rgb, "filled");
        xlabel("Radius (m)");
        ylabel("Neutral length (m)")
        
        subplot(1, 2, 2, "parent", options.fig);
        hold on
        scatter(mat_rxns(1, :), mat_rxns(3, :), [], mat_rgb, "filled");

        u_rxns_all_spaced = nan(3, 1);
        for i = 1 : size(cell_rxns, 1)
            mat_rxns_i = [cell_rxns{i, :}, nan(3, 1)];
            u_rxns_all_spaced = horzcat(u_rxns_all_spaced, mat_rxns_i);
        end

        v_rxns_all_spaced = nan(3, 1);
        for i = 1 : size(cell_rxns, 2)
            mat_rxns_i = [cell_rxns{:, i}, nan(3, 1)];
            v_rxns_all_spaced = horzcat(v_rxns_all_spaced, mat_rxns_i);
        end

        plot(u_rxns_all_spaced(1, :), u_rxns_all_spaced(3, :), 'k')
        plot(v_rxns_all_spaced(1, :), v_rxns_all_spaced(3, :), 'k')
        colorbar
        
        grid on
        xlabel("Reaction stretch (N)")
        ylabel("Reaction moment (Nm)")
    end
end

