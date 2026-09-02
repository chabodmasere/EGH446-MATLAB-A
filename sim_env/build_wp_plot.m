%% Build path for RVWP guidance

start_position = [0, 0];

wp_plot = [start_position;
    wp_ordered];

assignin('base','wp_plot',wp_plot);