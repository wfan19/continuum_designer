function cell_mats_out = lmatmul_cell(A, cell_mats_in)
    cell_mats_out = cellfun(@(cell_mats) A * cell_mats, cell_mats_in, 'uniformoutput', false);
end

