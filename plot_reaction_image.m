function plot_reaction_image(cell_rxns, mat_rxns, mat_rgb)
    % Given a matrix mat_rxns which consists of stretch, shear, and bending
    % reaction forces, and their associated colors, then this function will
    % plot them in a grid
    hold on
    
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
    
    scatter(mat_rxns(1, :), mat_rxns(3, :), [], mat_rgb, "filled");
    
    grid on
    xlabel("Reaction stretch (N)")
    ylabel("Reaction moment (Nm)")
end

