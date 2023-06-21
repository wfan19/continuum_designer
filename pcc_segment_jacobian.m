function jac = pcc_segment_jacobian(l_i, k_i)
    if abs(k_i) < 1e-4
        jac = [
            1, 0;
            0, l_i / 2;
            0, 1
        ];
    else
        jac = [
                1/k_i * sin(k_i), (l_i/k_i^2) * (k_i*cos(k_i) - sin(k_i));
                -1/k_i * (cos(k_i) - 1), (-l_i/k_i^2) * (-sin(k_i) * k_i - cos(k_i) + 1);
                0, 1
        ];
    end
end
