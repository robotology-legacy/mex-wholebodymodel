classdef (Abstract) absSimConfig
    properties(Abstract, Constant)
        DF_MAIN_POS@double     vector
        DF_AXES_POS@double     matrix
        DF_AXES_COLORS@double  matrix
        DF_AXIS_LIMITS@double  vector
        DF_GROUND_SHAPE@double matrix
        DF_GROUND_COLOR@double vector
    end

    properties(Abstract)
        robot_body@WBM.wbmSimBody
        environment@WBM.wbmSimEnvironment

        hMainFigure@matlab.ui.Figure
        main_title@char
        main_pos@double     vector

        hAxes@double        vector
        axes_pos@double     matrix
        axes_colors@double  matrix
        axis_limits@double  vector

        plot_objs@cell      vector
    end
end
