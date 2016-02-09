classdef iCubSimConfig < WBM.absWbmSimConfig
    properties
        main_title@char = 'iCub-Simulator:';
        hFigure_main
        hAxes@double   vector = zeros(1,4);
        plot_objs@cell vector = cell(1,4);
    end
    properties(Constant)
        MAIN_POS@double    vector = [50 400 600 650];
        AXES_POS@double    matrix = [0.51 0.20 0.45 0.40;
                                     0.01 0.20 0.45 0.40;
                                     0.51 0.62 0.45 0.40;
                                     0.01 0.62 0.45 0.40];
        AXES_COLORS@double matrix = repmat([0.8 0.8 0.8], 4, 1);
        AXIS_LIMITS@double vector = [-0.5 0.5 -0.42 0.58 0 1];
        PATCH_SHAPE@double matrix = [-0.45 -0.45 0.45 0.45;
                                     -0.37 0.53 0.53 -0.37;
                                     0 0 0 0];
        PATCH_COLOR@double vector = [0.6 0.6 0.8];
    end
end
