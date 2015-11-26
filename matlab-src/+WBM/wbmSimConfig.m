classdef wbmSimConfig
    properties
        main_figure;
        main_title@char;
        main_pos     = zeros(1,4);
        axes         = zeros(1,4);
        axes_pos     = zeros(4,4);
        axes_colors  = zeros(4,3);
        axis_limits  = zeros(1,6);
        patch_shape  = zeros(3,4);
        patch_color  = zeros(1,3);
        plot_objs    = cell(1,4);
    end
end
