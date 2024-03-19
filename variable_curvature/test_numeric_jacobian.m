function [e, df, dx, J_x] = test_numeric_jacobian(func, test_x, dx_min, dx_max)
    dx_range = dx_max - dx_min;
    dx = dx_range * rand(size(test_x)) + dx_min;
    test_x_perturbed = test_x + dx;

    [f_x, J_x] = func(test_x);
    [f_x_perturbed, ~] = func(test_x_perturbed);

    df = f_x_perturbed - f_x;

    df_analytic = J_x * dx(:);
    e = norm(df_analytic(:) - df(:));
end