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
        trajectories@WBM.wbmLinkTrajectory vector
        target_pts@WBM.wbmTargetPoint      vector

        hWndFigure@matlab.ui.Figure
        wnd_title@char
        wnd_pos@double      vector
        wnd_size@double     vector
        show_wnd@logical    scalar

        nAxes@uint8         scalar
        hAxes@double        vector
        axes_pos@double     matrix
        axes_colors@double  matrix
        axes_views@cell     vector
        axis_limits@double  vector

        vwpts_annot@cell    vector
        gfx_objects@cell    vector

        show_light@logical  scalar
        light_pos@double    vector

        show_titles@logical scalar
        tit_font_sz@double  scalar
        tit_font_color

        show_legend@logical scalar
        lgd_font_sz@double  scalar
        lgd_font_color
        lgd_bkgrd_color
        lgd_edge_color
        lgd_location@char
        lgd_orient@char

        mkvideo@logical     scalar
        vid_axes_idx@uint8  scalar
        vid_filename@char
        vid_fps@double      scalar
    end

    properties(Abstract, SetAccess = protected, GetAccess = public)
        pl_stack@cell      matrix
        pl_time_idx@uint32 matrix

        zoom_axes@double   matrix
        shift_axes@cell    matrix
    end

    methods(Sealed)
        function setPayloadStack(obj, vb_idx, manip)
            len = length(vb_idx);

            if ischar(manip)
                manip = {manip};
            elseif ( isvector(vb_idx) && (len > 1) && iscellstr(manip) )
                % make sure that both lists are column vectors ...
                vb_idx = vb_idx(:);
                manip  = manip(:);
            else
                error('wbmSimConfig::setPayloadStack: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            if (len ~= size(manip,1))
                error('wbmSimConfig::setPayloadStack: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
            end

            % create the payload stack with object index and the used manipulator (hand):
            obj.pl_stack = cell(len, 2);
            for i = 1:len
                obj.pl_stack{i,1} = vb_idx(i,1);
            end

            for i = 1:len
                m = manip{i,1};
                switch m
                    case {'lh', 'rh', 'bh'}
                        % left hand, right hand or both hands:
                        obj.pl_stack{i,2} = m;
                    otherwise
                        error('wbmSimConfig::setPayloadStack: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
                end
            end
            % initialize the utilization time indices of the given payload objects:
            obj.pl_time_idx = uint32(zeros(len,2)); % [start_idx, end_idx]
        end

        function setPayloadUtilTime(obj, obj_idx, start_idx, end_idx)
            len = size(obj.pl_stack,1);
            if ( (obj_idx < 1) || (obj_idx > len) )
                error('wbmSimConfig::setPayloadUtilTime: %s', WBM.wbmErrorMsg.VAL_OUT_OF_BOUNDS);
            end
            if (~start_idx || ~end_idx)
                error('wbmSimConfig::setPayloadUtilTime: %s', WBM.wbmErrorMsg.VALUE_LTE_ZERO);
            end
            obj.pl_time_idx(obj_idx,1) = start_idx;
            obj.pl_time_idx(obj_idx,2) = end_idx;
        end

    end
end
