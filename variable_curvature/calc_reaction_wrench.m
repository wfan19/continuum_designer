function reaction_wrenches = calc_reaction_wrench(mat_segment_twists, p, struct_design)
    N_actuators = length(struct_design.fs);
    N_twists = size(mat_segment_twists, 2);
    lengths_o = mat_segment_twists(1, :);
    
    lengths = zeros(N_actuators, N_twists);
    strains = zeros(N_actuators, N_twists);
    forces = zeros(N_actuators, N_twists);
    for i = 1 : length(struct_design.rhos)
        lengths(i, :) = lengths_o - struct_design.rhos(i) * mat_segment_twists(3, :);
        strains(i, :) = (lengths(i, :) - struct_design.l_0) / struct_design.l_0;
        for j = 1 : length(strains)
            forces(i, j) = struct_design.fs{i}(strains(i, j), p(i));
        end
    end
    reaction_wrenches = struct_design.mat_A * forces;
end