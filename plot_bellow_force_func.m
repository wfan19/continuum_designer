psi_to_kpa = 6.89476;

resolution = 10;
pressure_min = 0;
pressure_max = 60 * psi_to_kpa;
pressures = linspace(pressure_min, pressure_max, resolution);

strain_min = -0.4;
strain_max = 0.1;
strains = linspace(strain_min, strain_max, resolution);

[P, S] = meshgrid(pressures, strains);

F = bellow_force_func(P, S);

figure();
contour_mat = contour(P, S, F, [0,0]);
free_contraction_line = contour_mat(:, 2:end);

hold on
plot3(free_contraction_line(1, :), free_contraction_line(2, :), zeros(1, length(free_contraction_line)), "r", linewidth=3)
mesh(P, S, F);
view(10,10)


function F = bellow_force_func(P, S)
    psi_to_kpa = 6.89476;
    P_psi = P / psi_to_kpa;
    f_neutral_length = @(p) 7e-7*p.^3 - 5e-5*p.^2 - 1e-3*p + 0.4282;
    neutral_lengths = f_neutral_length(P_psi);
    neutral_strains = (neutral_lengths - f_neutral_length(0)) / f_neutral_length(0);
    K = 0.1;
    F = K * (S - neutral_strains);
end