classdef genericSimConfig < WBM.absSimConfig
    properties(Dependent)
        % public properties for fast get/set methods:
        custom_view@double vector % custom viewpoint for a perspective view.
        axes_vwpts@cell    matrix % viewpoint positions of the axes.
    end

    properties(Constant)
        DF_WND_POS@double        vector = [50  400]; % x (left), y (bottom)
        DF_WND_SIZE@double       vector = [600 650]; % width, height
        DF_AXES_NBR@uint8        scalar = 4; % number of axes to divide the figure into an m-by-n grid.
        % position of the new axes in the figure, specified as a four-element vector
        % [x (left), y (bottom), wid (width), hgt (height)]:
        %                                   x     y     wid   hgt
        DF_AXES_POS@double       matrix = [0.54  0.15  0.45  0.40;
                                           0.01  0.15  0.45  0.40;
                                           0.54  0.60  0.45  0.40;
                                           0.01  0.60  0.45  0.40];
        DF_AXES_COLORS@double    matrix = repmat([0.8  0.8  0.8], 4, 1);
        % default viewpoint positions to determine the orientation of the axes [az (azimuth), el (elevation)]:
        %                                              az    el
        DF_AXES_VWPTS@cell       matrix = {'front',  [-90     1];
                                           'side_l', [ 0      1];
                                           'side_r', [ 180    1];
                                           'top',    [-90    90];
                                           'custom', [-37.5  30]};
        % default viewpoints of the simulation to show in the visualizer:
        DF_AXES_VIEWS@cell       vector = {'custom', 'top', 'side_l', 'front'};

        DF_AXIS_LIMITS@double    vector = [-0.5  0.5  -0.42  0.58  0  1];

        DF_GROUND_SHAPE@double   matrix = [-0.45  -0.45  0.45   0.45;  % x
                                           -0.37   0.53  0.53  -0.37;  % y
                                               0      0     0      0]; % z
        DF_GROUND_COLOR@double   vector = [0.6  0.6  0.8];
        DF_GROUND_COLOR_2@double vector = [201 220 222] ./ 255; % hex: #c9dcde

        DF_VID_FILENAME@char            = 'robot-simulation.avi';
        DF_VID_FPS@double               = 30; % frames per second (frame rate)
    end

    properties
        robot_body@WBM.wbmSimBody
        environment@WBM.wbmSimEnvironment

        hWndFigure@matlab.ui.Figure
        wnd_title@char            = '';
        wnd_pos@double     vector = WBM.genericSimConfig.DF_WND_POS;
        wnd_size@double    vector = WBM.genericSimConfig.DF_WND_SIZE;

        nAxes@uint8        scalar = WBM.genericSimConfig.DF_AXES_NBR;
        hAxes@double       vector = [];
        axes_pos@double    matrix = WBM.genericSimConfig.DF_AXES_POS;
        axes_colors@double matrix = WBM.genericSimConfig.DF_AXES_COLORS;
        axes_views@cell    vector = WBM.genericSimConfig.DF_AXES_VIEWS;
        axis_limits@double vector = WBM.genericSimConfig.DF_AXIS_LIMITS;

        gfx_objects@cell   vector

        mkvideo@logical    scalar = false;
        vid_axes_id@uint8  scalar = 0; % 0 ... capture all axes (entire figure window) for the movie
        vid_filename@char         = WBM.genericSimConfig.DF_VID_FILENAME;
        vid_fps@double     scalar = WBM.genericSimConfig.DF_VID_FPS;
    end

    properties(Access = protected)
        maxes_vwpts@cell matrix = WBM.genericSimConfig.DF_AXES_VWPTS;
    end

    methods
        function obj = genericSimConfig(wnd_title, rob_sim_body, env_settings)
            if (nargin < 2)
                error('genericSimConfig::genericSimConfig: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end

            obj.wnd_title  = wnd_title;
            obj.robot_body = rob_sim_body;

            obj.hAxes    = zeros(1,obj.nAxes);
            obj.gfx_objects = cell(1,obj.nAxes);

            if exist('env_settings', 'var')
                obj.environment = env_settings;
                return
            end
            % else, use the default environment settings ...
            obj.environment = WBM.wbmSimEnvironment;
            obj.environment.background_color_opt = 'white';
            obj.environment.ground_shape         = WBM.genericSimConfig.DF_GROUND_SHAPE;
            obj.environment.ground_color         = WBM.genericSimConfig.DF_GROUND_COLOR;
            obj.environment.ground_edge_color    = 'black';
            obj.environment.origin_pt_color      = 'black';
            obj.environment.origin_pt_size       = 3.5;
            obj.environment.objects              = [];
        end

        function addView(obj, vp_name, vp)
            if ~ischar(vp_name)
                error('genericSimConfig::addView: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            WBM.utilities.chkfun.checkRVecDim(vp, 2, 'genericSimConfig::addView');

            len = size(obj.maxes_vwpts,1);
            pos = len + 1;
            obj.maxes_vwpts{pos,1} = vp_name;
            obj.maxes_vwpts{pos,2} = vp;
        end

        function vp = get.custom_view(obj)
            vp = obj.maxes_vwpts{10,1};
        end

        function set.custom_view(obj, vp)
            WBM.utilities.chkfun.checkRVecDim(vp, 2, 'genericSimConfig::set.custom_view');
            obj.maxes_vwpts{10,2} = vp;
        end

        function axes_vp = get.axes_vwpts(obj)
            axes_vp = obj.maxes_vwpts;
        end

    end
end
