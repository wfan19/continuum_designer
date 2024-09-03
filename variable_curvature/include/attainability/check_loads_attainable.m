function in_bndry = check_loads_attainable(bndry_af, bndry_am, test_f, test_m)

    N_tests = size(test_f, 2);
    N_nodes = size(bndry_af, 1);

    in_bndry = false(size(test_f));

    for i = 1 : N_nodes
        [A_i, b_i] = vert2lcon([bndry_af(i, :)', bndry_am(i, :)']);

        test_pts_i = [test_f(i, :); test_m(i, :)];
        in_bndry(i, :) = all(A_i * test_pts_i < b_i, 1);
    end

end

