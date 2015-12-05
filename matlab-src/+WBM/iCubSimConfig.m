classdef iCubSimConfig %< wbmSimConfig
    properties
        main_title   = 'iCub-Simulator:';
        hFigure_main
        hAxes        = zeros(1,4);
        plot_objs    = cell(1,4);
    end
    properties(Constant)
        MAIN_POS     = [50 400 600 650];
        AXES_POS     = [0.51 0.20 0.45 0.40;
                        0.01 0.20 0.45 0.40;
                        0.51 0.62 0.45 0.40;
                        0.01 0.62 0.45 0.40];
        AXES_COLORS  = repmat([0.8 0.8 0.8], 4, 1);
        AXIS_LIMITS  = [-0.5 0.5 -0.42 0.58 0 1];
        PATCH_SHAPE  = [-0.45 -0.45 0.45 0.45;
                        -0.37 0.53 0.53 -0.37;
                        0 0 0 0];
        PATCH_COLOR  = [0.6 0.6 0.8];
    end
end
