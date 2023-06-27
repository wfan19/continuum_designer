function cell_mats_out = rmatmul_cell(cell_mats_in, A)
    cell_mats_out = cellfun(@(cell_mats) cell_mats * A, cell_mats_in, 'uniformoutput', false);
end

