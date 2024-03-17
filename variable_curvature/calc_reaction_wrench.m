function [reaction_wrenches, J] = calc_reaction_wrench(mat_segment_twists, p, struct_design)
    N_actuators = length(struct_design.fs);
    N_twists = size(mat_segment_twists, 2);
    lengths_o = mat_segment_twists(1, :);
    
    lengths = zeros(N_actuators, N_twists);
    strains = zeros(N_actuators, N_twists);
    forces = zeros(N_actuators, N_twists);
    for i = 1 : N_actuators
        lengths(i, :) = lengths_o - struct_design.rhos(i) * mat_segment_twists(3, :);
        strains(i, :) = (lengths(i, :) - struct_design.l_0) / struct_design.l_0;
        for j = 1 : length(strains)
            forces(i, j) = struct_design.fs{i}(strains(i, j), p(i));
        end
    end
    reaction_wrenches = struct_design.mat_A * forces;

    if nargout == 2
        da_dtwists = cell(1, N_twists);
        for i = 1 : N_twists
            df_dtwist_i = zeros(N_actuators, 3);
            for j = 1 : N_actuators
                % Compute the rate of change of the force of actuator j in
                % segment i as the curvature of segment i is changed
                P = [p(j)^3, p(j)^2, p(j), 1];
                K = JacobBellowMechanics.get_stiffness_mat(); % TODO: Get the stiffness matrix!!
                strain_i = strains(j, i);
                dstrain_i = [3*strain_i^2; 2*strain_i; 1; 0];
                
                % Compute each component of the gradient and then combine
                df_j_dlength = 1/struct_design.l_0 * P * K *dstrain_i; % Change of actuator force wrt change of segment length
                df_j_dshear = 0;    % Change of actuator force wrt change of segment shear
                df_j_dcurvature = struct_design.rhos(j)/struct_design.l_0 * P * K * dstrain_i;  % Change of actuator force wrt change of segment curvature
                df_j_dtwist_i = [df_j_dlength, df_j_dshear, df_j_dcurvature];

                % Save it
                df_dtwist_i(j, :) = df_j_dtwist_i;
            end
            da_dtwists{i} = struct_design.mat_A * df_dtwist_i;
        end
        J = blkdiag(da_dtwists{:});
    end
end