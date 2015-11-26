classdef iCubSimConfig < wbmSimConfig
    properties
        main_figure;
        main_title   = 'iCub-Simulator:';
        axes         = zeros(1,4);
        plot_objs    = cell(1,4);
    end
    properties(Constant)
        main_pos     = [50 400 600 650];
        axes_pos     = [0.51 0.20 0.45 0.40;
                        0.01 0.20 0.45 0.40;
                        0.51 0.62 0.45 0.40;
                        0.01 0.62 0.45 0.40];
        axes_colors  = repmat([0.8 0.8 0.8], 4, 1);
        axis_limits  = [-0.5 0.5 -0.42 0.58 0 1];
        patch_shape  = [-0.45 -0.45 0.45 0.45;
                        -0.37 0.53 0.53 -0.37;
                        0 0 0 0];
        patch_color  = [0.6 0.6 0.8];
    end
end
