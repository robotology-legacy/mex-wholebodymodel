classdef genericSimConfig < WBM.wbmSimConfig
    properties(Dependent)
        % public properties for fast get/set methods:
        custom_view@double vector % custom viewpoint for a perspective view.
        axes_vwpts@cell    matrix % viewpoint positions of the axes.
    end

    properties(Constant)
        DF_WND_POS@double       vector = [50  400]; % [x (left), y (bottom)]
        DF_WND_SIZE@double      vector = [600 650]; % [width, height]
        DF_AXES_NBR@uint8       scalar = 4; % number of axes to divide the figure into an m-by-n grid.
        % position of the new axes in the figure, specified as a four-element vector
        % [x (left), y (bottom), wid (width), hgt (height)]:
        %                                  x     y     wid   hgt
        DF_AXES_POS@double      matrix = [0.54  0.05  0.45  0.42;
                                          0.01  0.05  0.45  0.42;
                                          0.54  0.60  0.45  0.42;
                                          0.01  0.60  0.45  0.42];
        DF_AXES_COLORS@double   matrix = repmat([0.8  0.8  0.8], 4, 1);
        % default viewpoint positions to determine the orientation of the axes [az (azimuth), el (elevation)]:
        %                                             az    el
        DF_AXES_VWPTS@cell      matrix = {'front',  [-90     1];
                                          'side_l', [ 0      1];
                                          'side_r', [ 180    1];
                                          'top',    [-90    90];
                                          'custom', [-37.5  30]};
        DF_VWPTS_ANNOT@cell     vector = {'front', 'l. side', 'r. side', ...
                                          'top', 'perspective'};
        % default viewpoints of the simulation to show in the visualizer:
        DF_AXES_VIEWS@cell      vector = {'custom', 'top', 'side_l', 'front'};

        DF_AXIS_LIMITS@double   vector = [-0.5  0.5  -0.42  0.58  0  1];

        DF_GROUND_SHAPE@double  matrix = [-0.45  -0.45  0.45   0.45;  % x
                                          -0.37   0.53  0.53  -0.37;  % y
                                              0      0     0      0]; % z
        DF_GROUND_COLOR@double  vector = [0.6  0.6  0.8];
        DF_GROUND_COLOR2@double vector = [201  220  222] ./ 255; % hex: #c9dcde

        DF_LIGHT_POS@double     vector = [0.25  0  1.25]; % (x,y,z)

        DF_VID_FILENAME@char           = 'robot-simulation.avi';
        DF_VID_FPS@double              = 30; % frames per second (frame rate)
    end

    properties
        robot_body@WBM.wbmSimBody
        environment@WBM.wbmSimEnvironment
        trajectories@WBM.wbmLinkTrajectory vector = WBM.wbmLinkTrajectory.empty;
        target_pts@WBM.wbmTargetPoint      vector = WBM.wbmTargetPoint.empty;

        hWndFigure@matlab.ui.Figure
        wnd_title@char             = '';
        wnd_pos@double      vector = WBM.genericSimConfig.DF_WND_POS;
        wnd_size@double     vector = WBM.genericSimConfig.DF_WND_SIZE;

        nAxes@uint8         scalar = WBM.genericSimConfig.DF_AXES_NBR;
        hAxes@double        vector = [];
        axes_pos@double     matrix = [];
        axes_colors@double  matrix = [];
        axes_views@cell     vector = {};
        axis_limits@double  vector = WBM.genericSimConfig.DF_AXIS_LIMITS;

        vwpts_annot@cell    vector = {};
        gfx_objects@cell    vector = {};

        show_light@logical  scalar = false;
        light_pos@double    vector = WBM.genericSimConfig.DF_LIGHT_POS;

        show_titles@logical scalar = true;
        tit_font_sz@double  scalar = 9;
        tit_font_color             = WBM.wbmColor.dimgray;

        show_legend@logical scalar = false;
        lgd_font_sz@double  scalar = 9;
        lgd_font_color             = 'black';
        lgd_bkgrd_color            = WBM.wbmColor.whitesmoke;
        lgd_edge_color             = WBM.wbmColor.dimgray;
        lgd_location@char          = 'northeast';
        lgd_orient@char            = 'vertical';

        mkvideo@logical     scalar = false;
        vid_axes_idx@uint8  scalar = 0; % 0 ... capture all axes (entire figure window) for the video
        vid_filename@char          = WBM.genericSimConfig.DF_VID_FILENAME;
        vid_fps@double      scalar = WBM.genericSimConfig.DF_VID_FPS;
    end

    properties(SetAccess = protected, GetAccess = public)
        pl_stack@cell      matrix = {};         % index list of volume objects that are defined as payload.
        pl_time_idx@uint32 matrix = uint32([]); % utilization time index of the payload objects [start_idx (obj. grabbed), end_idx (obj. released)].

        % index lists of axes to be zoomed or shifted:
        zoom_axes@double   matrix = []; % [ax_idx, zfac], zfac = 1 ... keeps the original size (zoom in: zfac > 1, zoom out: zfac > 0 and zfac < 1)
        shift_axes@cell    matrix = {}; % {ax_idx, sh_vec}, sh_vec = [x, y, z]
    end

    properties(Access = protected)
        maxes_vwpts@cell     matrix = WBM.genericSimConfig.DF_AXES_VWPTS;
    end

    methods
        function obj = genericSimConfig(wnd_title, rob_sim_body, varargin)
            switch nargin
                case 4
                    % nax          = varargin{1}
                    % env_settings = varargin{2}
                    obj.nAxes       = varargin{1,1};
                    obj.environment = varargin{1,2};
                case 3
                    if isa(varargin{1,1}, 'WBM.wbmSimEnvironment')
                        % env_settings = varargin{1}
                        obj.environment = varargin{1,1};
                    elseif isscalar(varargin{1,1})
                        % nax = varargin{1}
                        obj.nAxes = varargin{1,1};
                        defaultEnvSettings(obj);
                    else
                        error('genericSimConfig::genericSimConfig: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
                    end
                case 2
                    % use the default environment settings ...
                    defaultEnvSettings(obj);
                otherwise
                    error('genericSimConfig::genericSimConfig: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            obj.wnd_title  = wnd_title;
            obj.robot_body = rob_sim_body;

            obj.hAxes       = zeros(1,obj.nAxes);
            obj.gfx_objects = cell(1,obj.nAxes);

            if (obj.nAxes <= 4)
                % use the default axes settings ...
                obj.axes_pos    = WBM.genericSimConfig.DF_AXES_POS;
                obj.axes_colors = WBM.genericSimConfig.DF_AXES_COLORS;
                obj.axes_views  = WBM.genericSimConfig.DF_AXES_VIEWS;
                if obj.show_titles
                    obj.vwpts_annot = WBM.genericSimConfig.DF_VWPTS_ANNOT;
                end
            end
        end

        function zoomAxes(obj, axes_idx, zoom_fac)
            if ( ~isvector(axes_idx) || ~isvector(zoom_fac) )
                error('genericSimConfig::zoomAxes: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end
            axes_idx = axes_idx(:);
            zoom_fac = zoom_fac(:);

            if ~isreal(zoom_fac)
                error('genericSimConfig::zoomAxes: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            n = size(axes_idx,1);
            if (n ~= size(zoom_fac,1))
                error('genericSimConfig::zoomAxes: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
            end
            if ( (max(axes_idx) > obj.nAxes) || (min(axes_idx) < 1) )
                error('genericSimConfig::zoomAxes: %s', WBM.wbmErrorMsg.VAL_OUT_OF_BOUNDS);
            end
            if (n > 1)
                WBM.utilities.chkfun.checkNumListAscOrder(axes_idx, 'genericSimConfig::zoomAxes')
            end
            obj.zoom_axes = zeros(n,2);
            obj.zoom_axes(1:n,1) = axes_idx; % axes indices to be zoomed
            obj.zoom_axes(1:n,2) = zoom_fac; % zoom factors
        end

        function shiftAxes(obj, axes_idx, shift_vec)
            if ( ~isvector(axes_idx) || ~ismatrix(shift_vec) )
                error('genericSimConfig::shiftAxes: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end
            axes_idx = axes_idx(:);

            if ( ~isreal(axes_idx) || ~isreal(shift_vec) )
                error('genericSimConfig::shiftAxes: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            [m, n] = size(shift_vec);
            if (m ~= size(axes_idx,1))
                error('genericSimConfig::shiftAxes: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
            end
            if (n ~= 3)
                error('genericSimConfig::shiftAxes: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end
            if ( (max(axes_idx) > obj.nAxes) || (min(axes_idx) < 1) )54
                error('genericSimConfig::shiftAxes: %s', WBM.wbmErrorMsg.VAL_OUT_OF_BOUNDS);
            end
            if (m > 1)
                WBM.utilities.chkfun.checkNumListAscOrder(axes_idx, 'genericSimConfig::shiftAxes')
            end
            obj.shift_axes = cell(1,2);
            obj.shift_axes{1,1} = uint8(axes_idx); % axes indices to be shifted
            obj.shift_axes{1,2} = shift_vec;       % shift vectors [x, y, z]
        end

        function addView(obj, vp_name, vp)
            if ~ischar(vp_name)
                error('genericSimConfig::addView: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            WBM.utilities.chkfun.checkRVecDim(vp, 2, 'genericSimConfig::addView');

            if strcmp(vp_name, obj.axes_views)
                error('genericSimConfig::addView: The name is already reserved!');
            end
            len = size(obj.maxes_vwpts,1);
            idx = len + 1;
            obj.maxes_vwpts{idx,1} = vp_name;
            obj.maxes_vwpts{idx,2} = vp;
        end

        function vp = get.custom_view(obj)
            idx = find(strcmp(obj.maxes_vwpts(1:end,1), 'custom'));
            if isempty(idx)
                error('genericSimConfig::get.custom_view: %s', WBM.wbmErrorMsg.NAME_NOT_EXIST);
            end
            vp = obj.maxes_vwpts{idx,2};
        end

        function set.custom_view(obj, vp)
            WBM.utilities.chkfun.checkRVecDim(vp, 2, 'genericSimConfig::set.custom_view');
            idx = find(strcmp(obj.maxes_vwpts(1:end,1), 'custom'));
            if isempty(idx)
                % define a custom viewpoint ...
                idx = size(obj.maxes_vwpts,1) + 1;
                obj.maxes_vwpts{idx,1} = 'custom';
            end
            obj.maxes_vwpts{idx,2} = vp;
        end

        function axes_vp = get.axes_vwpts(obj)
            axes_vp = obj.maxes_vwpts;
        end

    end

    methods(Access = protected)
        function defaultEnvSettings(obj)
            obj.environment = WBM.wbmSimEnvironment;
            obj.environment.bkgrd_color_opt = 'white';
            obj.environment.grnd_shape      = WBM.genericSimConfig.DF_GROUND_SHAPE;
            obj.environment.grnd_color      = WBM.genericSimConfig.DF_GROUND_COLOR;
            obj.environment.grnd_edge_color = 'black';
            obj.environment.orig_pt_color   = 'black';
            obj.environment.orig_pt_size    = 3.5;
            obj.environment.vb_objects      = WBM.vbObject.empty;
        end

    end
end
