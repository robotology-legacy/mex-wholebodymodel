classdef (Abstract) wbmSimConfig
    properties
        main_title@char
        hFigure_main
        hAxes     = zeros(1,4);
        plot_objs = cell(1,4);
    end
    properties(Constant)
        MAIN_POS@double vector
        AXES_POS@double matrix
        AXES_COLORS@double matrix
        AXIS_LIMITS@double vector
        PATCH_SHAPE@double matrix
        PATCH_COLOR@double vector
        % MAIN_POS     = zeros(1,4);
        % AXES_POS     = zeros(4,4);
        % AXES_COLORS  = zeros(4,3);
        % AXIS_LIMITS  = zeros(1,6);
        % PATCH_SHAPE  = zeros(3,4);
        % PATCH_COLOR  = zeros(1,3);
    end
end
