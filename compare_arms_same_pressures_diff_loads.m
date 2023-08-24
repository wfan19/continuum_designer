function compare_arms_same_pressures_diff_loads(cell_arms, pressures, loads, fig, test_name)

    tl = tiledlayout(fig, 1, length(cell_arms));
    for i_load = 1 : size(loads, 2)
        load_i = loads(:, i_load);
        for i_arm = 1 : length(cell_arms)
            arm_i = cell_arms{i_arm};
            arm_i.solve_equilibrium_gina(pressures, load_i);
    
            ax = nexttile(i_arm);
            Plotter2D.plot_arm_series(arm_i, ax);
            grid on
        end
    end
    linkaxes(tl.Children);

    set(fig, "Renderer", "painters");
    filename_base = sprintf("figures/%s", test_name);
    saveas(fig, filename_base + ".png");
    saveas(fig, filename_base + ".fig");
    saveas(fig, filename_base + ".svg");
    
end