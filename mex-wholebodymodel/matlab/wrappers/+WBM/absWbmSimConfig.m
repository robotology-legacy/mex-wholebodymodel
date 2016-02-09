classdef (Abstract) absWbmSimConfig
    properties(Abstract)
        main_title@char
        hFigure_main
        hAxes@double   vector
        plot_objs@cell vector
    end
    properties(Abstract, Constant)
        MAIN_POS@double    vector
        AXES_POS@double    matrix
        AXES_COLORS@double matrix
        AXIS_LIMITS@double vector
        PATCH_SHAPE@double matrix
        PATCH_COLOR@double vector
    end
end
