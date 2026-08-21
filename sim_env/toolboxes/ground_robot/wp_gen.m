function wp_list = wp_gen(coord_lo, coord_hi, N)
    X = randi([coord_lo, coord_hi], N, 1);
    Y = randi([coord_lo, coord_hi], N, 1);
    wp_list = [X Y];
end