classdef (Abstract) wbmSimConfig < handle
    properties(Abstract, Dependent)
        % public properties for fast get/set methods:
        custom_view@double vector % custom viewpoint for a perspective view.
        axes_vwpts@cell    matrix % viewpoint positions of the axes.
    end

    properties(Abstract, Constant)
        DF_WND_POS@double      vector
        DF_WND_SIZE@double     vector
        DF_AXES_NBR@uint8      scalar
        DF_AXES_POS@double     matrix
        DF_AXES_COLORS@double  matrix
        DF_AXES_VWPTS@cell     matrix
        DF_AXES_VIEWS@cell     vector
        DF_AXIS_LIMITS@double  vector
        DF_GROUND_SHAPE@double matrix
        DF_GROUND_COLOR@double vector
    end

    properties(Abstract)
        robot_body@WBM.wbmSimBody
        environment@WBM.wbmSimEnvironment

        hWndFigure@matlab.ui.Figure
        wnd_title@char
        wnd_pos@double     vector
        wnd_size@double    vector

        nAxes@uint8        scalar
        hAxes@double       vector
        axes_pos@double    matrix
        axes_colors@double matrix
        axes_views@cell    vector
        axis_limits@double vector

        gfx_objects@cell   vector

        show_light@logical scalar
        light_pos@double   vector

        mkvideo@logical    scalar
        vid_axes_id@uint8  scalar
        vid_filename@char
        vid_fps@double     scalar
    end

    properties(Abstract, SetAccess = protected, GetAccess = public)
        pl_stack@cell     matrix
        pl_time_idx@uint8 matrix
    end
end
