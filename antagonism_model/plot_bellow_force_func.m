psi_to_kpa = 6.89476;

resolution = 20;
pressure_min = 0;
pressure_max = 20;
pressures = linspace(pressure_min, pressure_max, resolution);

strain_min = -0.2;
strain_max = 0.5;
strains = linspace(strain_min, strain_max, resolution);

[S, P] = meshgrid(strains, pressures);

F = bellow_force_func(S, P);

figure();
contour_mat = contour(S, P, F, [0,0]);
free_contraction_line = contour_mat(:, 2:end);

hold on
plot3(free_contraction_line(1, :), free_contraction_line(2, :), zeros(1, length(free_contraction_line)), "r", linewidth=3)
mesh(S, P, F);
view(10,10)

function F = wrapped_gina_force_func(S, P)
    F = zeros(size(P));
    for i = 1 : size(P, 1)
        for j = 1 : size(P, 2)
            F(i, j) = actuatorForce_key(S(i, j), P(i, j));
        end
    end
    F = -F;
end

function F = bill_force_func(S, P)
    psi_to_kpa = 6.89476;
    P_psi = P / psi_to_kpa;
    f_neutral_length = @(p) 7e-7*p.^3 - 5e-5*p.^2 - 1e-3*p + 0.4282;
    neutral_lengths = f_neutral_length(P_psi);
    neutral_strains = (neutral_lengths - f_neutral_length(0)) / f_neutral_length(0);
    K = 0.1;
    F = K * (S - neutral_strains);
end

function F = bellow_force_func(S, P)
    F = zeros(size(P));

    for i = 1 : size(P, 1)
        for j = 1 : size(P, 2)
            s = S(i, j);
            p = P(i, j);

            F(i, j) = bellowForce(s, p);
        end
    end
end