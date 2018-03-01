% Copyright (C) 2015-2018, by Martin Neururer
% Author: Martin Neururer
% E-mail: martin.neururer@student.tuwien.ac.at / martin.neururer@gmail.com
% Date:   January, 2018
%
% Departments:
%   Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia and
%   Automation and Control Institute - TU Wien.
%
% This file is part of the Whole-Body Model Library for Matlab (WBML).
%
% The development of the WBM-Library was made in the context of the master
% thesis "Learning Task Behaviors for Humanoid Robots" and is an extension
% for the Matlab MEX whole-body model interface, which was supported by the
% FP7 EU project CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and
% Robotics (b)), <http://www.codyco.eu>.
%
% Permission is granted to copy, distribute, and/or modify the WBM-Library
% under the terms of the GNU Lesser General Public License, Version 2.1
% or any later version published by the Free Software Foundation.
%
% The WBM-Library is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU Lesser General Public License for more details.
%
% A copy of the GNU Lesser General Public License can be found along
% with the WBML. If not, see <http://www.gnu.org/licenses/>.

classdef vbCuboid < WBM.vbObject
    % :class:`!vbCuboid` is a class to create *rectangular cuboid objects* for
    % the environment scenario of the robot simulation.
    %
    % Attributes:
    %   origin (double, vector): (3 x 1) Cartesian position of the origin of the
    %                            cuboid. Default origin: :math:`[0, 0, 0]^T`.
    %   rotm   (double, matrix): (3 x 3) rotation matrix of the cuboid. If
    %                            undefined, the default orientation is the
    %                            *identity matrix*.
    %   tform  (double, matrix): (4 x 4) Transformation matrix of the cuboid. If
    %                            :attr:`origin` and :attr:`rotm` are undefined,
    %                            then by default, the transformation matrix is an
    %                            *identity matrix*.
    %   frame  (double, vector): (7 x 1) VQ-transformation vector (position and
    %                            orientation) of the cuboid. If :attr:`origin`
    %                            and :attr:`rotm` are undefined, then the frame
    %                            vector uses the default transformation vector
    %                            :attr:`~WBM.vbObject.DF_FRAME`.
    %
    %   description       (char, vector): Short description string about the
    %                                     cuboid object (default: *empty*).
    %   ismovable      (logical, scalar): Boolean flag to indicate if the cuboid
    %                                     object is movable (default: *false*).
    %   line_width      (double, scalar): Line width of the edges of the cuboid,
    %                                     specified as a positive value in points
    %                                     (default width: 0.4).
    %   edge_color (double/char, vector): Edge color of the cuboid, specified
    %                                     by a RGB-triplet or a color name
    %                                     (default color: *'black'*).
    %   face_color (double/char, vector): Face color of the cuboid, specified
    %                                     by a RGB-triplet or a color name
    %                                     (default color: :attr:`!wbmColor.lightsteelblue`).
    %   face_alpha      (double, scalar): Face transparency of the cuboid, specified
    %                                     by a scalar in range :math:`[0,1]`
    %                                     (default alpha: 0.2).
    %
    %   obj_type      (char, vector): Type of the object (*read only*), specified by
    %                                 one of these values:
    %
    %                                    - ``'obs'``: The cuboid defines an *obstacle* (for the robot).
    %                                    - ``'bdy'``: The cuboid is only an arbitrary *volume body* in
    %                                      the environment and not a specific obstacle.
    %
    %                                 The default object type is ``'bdy'``.
    %   isobstacle (logical, scalar): Boolean flag to indicate if the cuboid object
    %                                 is also defined as an obstacle (*read only*).
    %
    %                                 **Note:** The value will be set to *true*, if
    %                                 and only if :attr:`obj_type` is defined as an
    %                                 obstacle (*'obs'*), *false* otherwise (default).
    %   issolid    (logical, scalar): Boolean flag to indicate if the cuboid is a
    %                                 *solid* object (*read only*). Default: *false*.
    %
    %                                 **Note:** If a volumetric mass density is given,
    %                                 then the cuboid object is automatically defined
    %                                 as a *solid volume body*, i.e. the variable
    %                                 :attr:`!issolid` will be set to *true*.
    %   istool     (logical, scalar): Boolean flag to indicate if the cuboid defines
    %                                 also a tool to be used by the robot (*read only*).
    %                                 Default: *false*.
    %   init_frame  (double, vector): (7 x 1) initial frame vector (VQ-transformation)
    %                                 of the cuboid, in dependency of the given origin
    %                                 position and orientation (*read only*).
    %
    %                                 **Note:** If the attributes :attr:`origin` and
    %                                 :attr:`rotm` are not defined, then the initial
    %                                 frame uses the default transformation vector
    %                                 :attr:`~WBM.vbObject.DF_FRAME`.
    %   dimension   (double, vector): (1 x 3) vector of the form :math:`[\textrm{width, length, height}]`,
    %                                 which defines the dimensions of the cuboid (*read only*).
    %
    %                                 **Note:** If the dimensions of the cuboid are undefined,
    %                                 then the vector is by default a *0-vector*.
    %   vertices    (double, matrix): (8 x 3) *vertex positions matrix* of the cuboid
    %                                 at the given origin frame (*read only*).
    %
    %                                 **Note:** If the dimensions of cuboid are not
    %                                 defined, then the matrix is by default a *0-matrix*.
    %   com   (double, vector): (3 x 1) Cartesian position of the center of mass
    %                           (CoM) of the cuboid relative to the given origin
    %                           (*read only*).
    %
    %                           **Note:** By default, the CoM of the cuboid is
    %                           placed at the origin.
    %   rho   (double, scalar): Volumetric mass density of the cuboid, specified as a
    %                           positive value in :math:`[\si{\kg/\m^3}]` (*read only*).
    %
    %                           **Note:** The cuboid is defined as a *solid volume body*
    %                           if and only if the density value :math:`\rho > 0`. If the
    %                           density of the cuboid is undefined, then the default
    %                           value is 0.
    %   m_rb  (double, scalar): Mass of the cuboid object (rigid body *rb*) in
    %                           :math:`[\si{\kg}]` (*read only*).
    %
    %                           **Note:** If :attr:`rho` is not defined, then by
    %                           default the mass value is 0.
    %   I_cm  (double, matrix): (3 x 3) inertia matrix of the cuboid at the
    %                           center of mass *cm* (*read only*).
    %
    %                           **Note:** If :attr:`rho` is not defined, then the
    %                           inertia of the cuboid is by default the *identity
    %                           matrix*.
    %   mgrid         (struct): Data structure for the *grid point coordinates* of
    %                           the internal 3D-meshgrid of the cuboid (*read only*).
    %
    %                           The 3D-meshgrid will be used in the robot simulation
    %                           to simulate the cuboid object as a solid obstacle.
    %
    %                           **Note:** The data structure consists of the fields
    %                           ``X``, ``Y`` and ``Z`` that are representing the x, y
    %                           and z-coordinates over a grid, specified as 3D-arrays
    %                           (with three inputs each). The meshgrid for the cuboid
    %                           object can be created only if the object is defined as
    %                           an *obstacle*.
    properties(Dependent)
        origin@double vector
        rotm@double   matrix
        tform@double  matrix
        frame@double  vector
    end

    properties
                                   % default values:
        description@char         = '';  % annotation of the cuboid
        ismovable@logical scalar = false;
        line_width@double scalar = 0.4;
        edge_color               = 'black';
        face_color               = WBM.wbmColor.lightsteelblue;
        face_alpha@double scalar = 0.2; % transparency of the fill area
    end

    properties(SetAccess = private, GetAccess = public)
        obj_type@char             = 'bdy';
        isobstacle@logical scalar = false;
        issolid@logical    scalar = false;
        istool@logical     scalar = false;
        init_frame@double  vector = WBM.vbCuboid.DF_FRAME;
        dimension@double   vector = zeros(1,3);
        vertices@double    matrix = zeros(8,3);
        com@double         vector = zeros(3,1);
        rho@double         scalar = 0;
        m_rb@double        scalar = 0;
        I_cm@double        matrix = eye(3,3);
        mgrid@struct              = struct('X', [], 'Y', [], 'Z', []);
    end

    properties(Access = private)
        mcub_orig@double   vector = zeros(3,1); % initial Cartesian position of the origin [x,y,z]
        mcub_rotm@double   matrix = eye(3,3);   % initial rotation matrix (orientation)
        mcub_vtx_s@double  matrix = zeros(8,3); % vertex positions scaled to width, length & height
        mcub_vtx_sr@double matrix = zeros(8,3); % vertex positions scaled and set to the given orientation
        mcub_mg_s@struct          = struct('xx', [], 'yy', [], 'zz', []); % 3D-coordinates arrays of the scaled cuboid (s)
        mcub_mg_sr@struct         = struct('xx', [], 'yy', [], 'zz', []); % 3D-coordinates arrays of the scaled and rotated cuboid (sr)
        mcub_mgC_s@double  matrix = [];         % 3D-meshgrid coordinates matrix of the scaled cuboid
        mcub_msz@double    scalar = 0.01;       % mesh size (msz) of the internal 3D-grid (default: 0.01)
    end

    properties(Access = private, Constant)
        % vertex position matrix of the unit cuboid
        % with the CoM at the origin point 0:
        %           x:    y:    z:
        mverts = [-0.5  -0.5  -0.5;
                  -0.5   0.5  -0.5;
                   0.5   0.5  -0.5;
                   0.5  -0.5  -0.5;
                  -0.5  -0.5   0.5;
                  -0.5   0.5   0.5;
                   0.5   0.5   0.5;
                   0.5  -0.5   0.5];

        % vertex connection matrix to define which
        % vertices are to connect for the polygons
        % of the cuboid:
        mfaces = uint8([1 2 3 4;
                        1 4 8 5;
                        2 6 5 1;
                        3 7 8 4;
                        5 8 7 6;
                        7 3 2 6]);
    end

    methods
        function obj = vbCuboid(varargin)
            % Constructor.
            %
            % The constructor of the :class:`!vbCuboid` class can be called in
            % three different ways, where the *keyword arguments* in the square
            % brackets are optional:
            %
            %   - .. function:: vbCuboid(l_x, l_y, l_z[, orig, rotm[, obj_prop]])
            %   - .. function:: vbCuboid(l[, orig, rotm[, obj_prop]])
            %   - .. function:: vbCuboid()
            %
            % The first two options specifying as volume body object either
            % a *rectangular cuboid* or a *cube*. The third option without
            % parameters will be used for creating the *default object* of
            % a heterogeneous hierarchy in :class:`~WBM.vbObject` (see
            % :meth:`~WBM.vbObject.getDefaultScalarElement`).
            %
            % Arguments:
            %   varargin: Variable-length input argument list.
            %
            % Keyword Arguments:
            %   l_x  (double, scalar): Length along the x-direction (width).
            %   l_y  (double, scalar): Length along the y-direction (length).
            %   l_z  (double, scalar): Length along the z-direction (height).
            %   l    (double, scalar): Length of all sides of the cube.
            %   orig     (double, vector): (3 x 1) origin position of the cuboid.
            %   rotm     (double, matrix): (3 x 3) orientation matrix of the cuboid.
            %   obj_prop         (struct): Data structure for the *object properties*,
            %                              specified by following fields:
            %
            %                                 - ``line_width``: Line width of the edges of the cuboid.
            %                                 - ``edge_color``: Edge color of the cuboid (RGB-triplet
            %                                   or color name).
            %                                 - ``face_color``: Face color of the cuboid (RGB-triplet
            %                                   or color name).
            %                                 - ``face_alpha``: Face transparency of the cuboid in
            %                                   range :math:`[0,1]`.
            %
            %                              optional fields:
            %
            %                                 - ``description``: Annotation string of the
            %                                   cuboid object.
            %                                 - ``ismovable``: Boolean flag to indicate if
            %                                   the cuboid object is *movable* or *fixed*.
            %                                 - ``istool``: Boolean flag to indicate if the
            %                                   cuboid object is also a *tool* or not.
            %                                 - ``obj_type``: Object type, specified by the value
            %                                   *'obs'* or *'bdy'* (see :attr:`~vbCuboid.obj_type`).
            %                                 - ``rho``: Volumetric mass density of the cuboid
            %                                   object in :math:`[\si{\kg/\m^3}]`.
            % Returns:
            %   obj: An instance of the :class:`!vbCuboid` class.
            switch nargin
                case 6 % rectangular cuboid:
                    % l_x      = varargin{1}
                    % l_y      = varargin{2}
                    % l_z      = varargin{3}
                    % orig     = varargin{4}
                    % rotm     = varargin{5}
                    % obj_prop = varargin{6}

                    setObjData(obj, varargin{1,4:6});
                    createCuboid(obj, varargin{1,1:3});
                case 5
                    % l_x  = varargin{1}
                    % l_y  = varargin{2}
                    % l_z  = varargin{3}
                    % orig = varargin{4}
                    % rotm = varargin{5}

                    setObjData(obj, varargin{1,4:5});
                    createCuboid(obj, varargin{1,1:3});
                case 4 % rectangular square cuboid (cube):
                    % l        = varargin{1}
                    % orig     = varargin{2}
                    % rotm     = varargin{3}
                    % obj_prop = varargin{4}

                    setObjData(obj, varargin{1,2:4});
                    createCuboid(obj, varargin{1,1});
                case 3
                    if isscalar(varargin{1,2})
                        % simple cuboid:
                        % l_x = varargin{1}
                        % l_y = varargin{2}
                        % l_z = varargin{3}
                        createCuboid(obj, varargin{1,1:3});
                        return
                    else
                        % square cuboid:
                        % l    = varargin{1}
                        % orig = varargin{2}
                        % rotm = varargin{3}

                        setObjData(obj, varargin{1,2:3});
                        createCuboid(obj, varargin{1,1});
                    end
                case 1 % simple cube:
                    % l = varargin{1}
                    createCuboid(obj, varargin{1,1});
                    return
                case 0
                    % for initialization/preallocation of vbObject arrays ...
                    return
                otherwise
                    error('vbCuboid::vbCuboid: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            setCuboidAtPosRotm(obj);
        end

        function setInitFrame(obj, varargin)
            % Sets the origin frame of the cuboid to the given initial position
            % and orientation.
            %
            % The method can be called in three different ways:
            %
            %   - .. function:: setInitFrame(p, R)
            %   - .. function:: setInitFrame(vqT)
            %   - .. function:: setInitFrame()
            %
            % If no parameters are given, then :meth:`!setInitFrame` uses for
            % the initial origin frame of the cuboid the current frame vector
            % of the property :attr:`~vbCuboid.init_frame`.
            %
            % Arguments:
            %   varargin: Variable-length input argument list.
            %
            % Keyword Arguments:
            %   p   (double, vector): (3 x 1) Cartesian position to specify the
            %                         *origin* of the cuboid.
            %   R   (double, matrix): (3 x 3) rotation matrix to specify the
            %                         *orientation* of the cuboid.
            %   vqT (double, vector): (7 x 1) VQ-transformation (position and
            %                         orientation in quaternions) to specify
            %                         the *origin frame* of the cuboid.
            switch nargin
                case 3
                    p = varargin{1,1};
                    R = varargin{1,2};

                    if WBM.utilities.isZero(R)
                        error('vbCuboid::setInitFrame: %s', WBM.wbmErrorMsg.UDEF_ROT_MAT);
                    end

                    obj.init_frame = WBM.utilities.tfms.posRotm2frame(p, R);
                    obj.mcub_orig  = p;
                    obj.mcub_rotm  = R;
                case 2
                    vqT = varargin{1,1};

                    if WBM.utilities.isZero(vqT(4:7))
                        error('vbCuboid::setInitFrame: %s', WBM.wbmErrorMsg.UDEF_QUAT_VEC);
                    end

                    [obj.mcub_orig, obj.mcub_rotm] = WBM.utilities.tfms.frame2posRotm(vqT);
                    obj.init_frame = vqT;
                case 1
                    % use the current initial frame ...
                    [obj.mcub_orig, obj.mcub_rotm] = WBM.utilities.tfms.frame2posRotm(obj.init_frame);
                otherwise
                    error('vbCuboid::setInitFrame: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            setCuboidAtPosRotm(obj);
        end

        function hgo = getGObj(obj)
            % Creates and draws the cuboid object and returns a handle to the
            % created graphics object.
            %
            % Returns:
            %   hgo: Handle to the patch graphics object that contains the data
            %        for all the polygons.
            hgo = patch('Vertices', obj.vertices, 'LineWidth', obj.line_width, 'EdgeColor', obj.edge_color, ...
                        'Faces', obj.mfaces, 'FaceColor', obj.face_color, 'FaceAlpha', obj.face_alpha);
        end

        function hgo = updGObj(obj, hgo)
            % Updates the vertex coordinates of the cuboid.
            %
            % Arguments:
            %   hgo: Handle to the patch object that contains the data of all
            %        polygons of the cuboid.
            % Returns:
            %   hgo: Handle to the patch object with the changed vertex
            %        coordinates of the polygons.
            hgo.Vertices = obj.vertices;
        end

        function hmg = drawMGrid(obj, pt_color)
            % Draws the meshgrid of the cuboid and returns a graphics object
            % handle to it.
            %
            % Arguments:
            %   pt_color (double/char, vector): Color of the grid points, specified
            %                                   by a RGB-triplet or a color name
            %                                   (default color: *'green'*).
            % Returns:
            %   hmg: Handle to the scatter series object of the generated meshgrid.
            if ~obj.isobstacle
                error('vbCuboid::drawMGrid: %s', WBM.wbmErrorMsg.OBJ_NOT_OBSTACLE);
            end

            if (nargin == 1)
                pt_color = 'green';
            end
            if (nargout == 1)
                hmg = scatter3(obj.mgrid.X(:), obj.mgrid.Y(:), obj.mgrid.Z(:), ...
                               'Marker', '.', 'MarkerEdgeColor', pt_color);
                return
            end
            % else ...
            scatter3(obj.mgrid.X(:), obj.mgrid.Y(:), obj.mgrid.Z(:), ...
                     'Marker', '.', 'MarkerEdgeColor', pt_color);
        end

        function result = ptInObj(obj, pt_pos)
            % Determines if some specified points are below the surface,
            % i.e. inside, of the cuboid.
            %
            % Arguments:
            %   pt_pos (double, vector/matrix): A single position vector or a
            %                                   (n x 3) matrix with positions
            %                                   of some specified points.
            % Returns:
            %   result: (n x 1) vector with the logical results if the given
            %           points are below the surface of the cuboid. Each
            %           value is *true* if the given point is below the
            %           surface, *false* otherwise.

            % half length of each side of the cuboid ...
            lx_h = obj.mcub_vtx_s(7,1);
            ly_h = obj.mcub_vtx_s(7,2);
            lz_h = obj.mcub_vtx_s(7,3);

            if isvector(pt_pos)
                WBM.utilities.chkfun.checkVecLen(pt_pos, 3, 'vbCuboid::ptInObj')
                pt_pos = pt_pos(:); % make sure that pt_pos is a column vector

                % get the distance of the given point to the CoM and rotate this
                % point back to the initial condition of the cuboid (rect. pos.
                % with CoM at origin 0):
                d_r = obj.mcub_rotm.' * (pt_pos - obj.com); % back-rotated distance
                d_x = d_r(1,1);
                d_y = d_r(2,1);
                d_z = d_r(3,1);

                % check if the back-rotated point position is in the cuboid,
                % i.e. the point is below the surface of the cuboid:
                res_x = (d_x > -lx_h) && (d_x < lx_h);
                res_y = (d_y > -ly_h) && (d_y < ly_h);
                res_z = (d_z > -lz_h) && (d_z < lz_h);
            elseif ismatrix(pt_pos)
                [m, n] = size(pt_pos);
                if (n ~= 3)
                    error('vbCuboid::ptInObj: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
                end
                % get the distances of all points to the CoM:
                ds = zeros(m,3);
                cm = obj.com.';
                for i = 1:m
                    ds(i,1:3) = pt_pos(i,1:3) - cm;
                end
                ds_r = ds * obj.mcub_rotm; % back-rotated distances,
                                           % ds_r = (R^T * ds^T)^T
                ds_x = ds_r(1:m,1);
                ds_y = ds_r(1:m,2);
                ds_z = ds_r(1:m,3);

                % check if all points are below the cuboid's surface:
                res_x = (ds_x > -lx_h) && (ds_x < lx_h);
                res_y = (ds_y > -ly_h) && (ds_y < ly_h);
                res_z = (ds_z > -lz_h) && (ds_z < lz_h);
            else
                error('vbCuboid::ptInObj: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            % total result of all directions ...
            result = (res_x && res_y && res_z);
        end

        function orig = get.origin(obj)
            orig = obj.mcub_orig;
        end

        function set.origin(obj, orig)
            WBM.utilities.chkfun.checkCVecDim(orig, 3, 'vbCuboid::set.origin');
            obj.mcub_orig = orig;

            setCuboidAtPos(obj);
            if obj.isobstacle
                setMeshgridAtPos(obj);
            end
        end

        function rotm = get.rotm(obj)
            rotm = obj.mcub_rotm;
        end

        function set.rotm(obj, rotm)
            WBM.utilities.chkfun.checkMatDim(rotm, 3, 3, 'vbCuboid::set.rotm');
            if WBM.utilities.isZero(rotm)
                error('vbCuboid::set.rotm: %s', WBM.wbmErrorMsg.UDEF_ROT_MAT);
            end
            obj.mcub_rotm = rotm;
            setCuboidAtPosRotm(obj);
        end

        function tform = get.tform(obj)
            tform = eye(4,4);
            tform(1:3,1:3) = obj.mcub_rotm;
            tform(1:3,4)   = obj.mcub_orig;
        end

        function set.tform(obj, tform)
            [obj.mcub_orig, R] = WBM.utilities.tfms.tform2posRotm(tform);

            if WBM.utilities.isZero(R)
                error('vbCuboid::set.tform: %s', WBM.wbmErrorMsg.UDEF_ROT_MAT);
            end
            obj.mcub_rotm = R;
            setCuboidAtPosRotm(obj);
        end

        function vqT = get.frame(obj)
            vqT = zeros(7,1);
            vqT(1:3,1) = obj.mcub_orig;
            vqT(4:7,1) = WBM.utilities.tfms.rotm2quat(obj.mcub_rotm);
        end

        function set.frame(obj, vqT)
            if WBM.utilities.isZero(vqT(4:7))
                error('vbCuboid::set.frame: %s', WBM.wbmErrorMsg.UDEF_QUAT_VEC);
            end
            [obj.mcub_orig, obj.mcub_rotm] = WBM.utilities.tfms.frame2posRotm(vqT);
            setCuboidAtPosRotm(obj);
        end

    end

    methods(Access = private)
        function setObjData(obj, orig, rotm, obj_prop)
            if WBM.utilities.isZero(rotm)
                error('vbCuboid::setObjData: %s', WBM.wbmErrorMsg.UDEF_ROT_MAT);
            end
            obj.init_frame = WBM.utilities.tfms.posRotm2frame(orig, rotm);
            obj.mcub_orig  = orig;
            obj.mcub_rotm  = rotm;

            if (nargin == 4)
                % set the object properties of the cuboid ...
                if ~isstruct(obj_prop)
                    error('vbCuboid::setObjData: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
                end
                obj.line_width  = obj_prop.line_width;
                obj.edge_color  = obj_prop.edge_color;
                obj.face_color  = obj_prop.face_color;
                obj.face_alpha  = obj_prop.face_alpha;

                if isfield(obj_prop, 'description')
                    obj.description = obj_prop.description;
                end

                if isfield(obj_prop, 'ismovable')
                    obj.ismovable = obj_prop.ismovable;
                end

                if isfield(obj_prop, 'istool')
                    obj.istool = obj_prop.istool;
                end

                if isfield(obj_prop, 'obj_type')
                    switch obj_prop.obj_type
                        case 'bdy'
                            obj.isobstacle = false;
                        case 'obs'
                            obj.isobstacle = true;
                            if isfield(obj_prop, 'mesh_size')
                                obj.mcub_msz = obj_prop.mesh_size;
                            end
                        otherwise
                            error('vbCuboid::setObjData: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
                    end
                    obj.obj_type = obj_prop.obj_type;
                end

                if isfield(obj_prop, 'rho')
                    obj.issolid = true;
                    obj.rho     = obj_prop.rho;
                end
            end
        end

        function createCuboid(obj, varargin)
            switch nargin
                case 4
                    % general (rectangular) cuboid:
                    l_x = varargin{1,1}; % width
                    l_y = varargin{1,2}; % length
                    l_z = varargin{1,3}; % height

                    obj.dimension  = horzcat(l_x, l_y, l_z);
                    obj.mcub_vtx_s = horzcat(obj.mverts(1:8,1)*l_x, obj.mverts(1:8,2)*l_y, obj.mverts(1:8,3)*l_z);

                    if obj.issolid
                        if (obj.rho > 0)
                            % calculate the mass and the inertia of the cuboid ...                   wid, len, hgt
                            [obj.m_rb, obj.I_cm] = WBM.utilities.rb.massInertiaGObj('scub', obj.rho, l_y, l_x, l_z);
                        end
                    end
                case 2
                    % square cuboid (cube):
                    l = varargin{1,1}; % length of all sides

                    obj.dimension  = horzcat(l, l, l);
                    obj.mcub_vtx_s = horzcat(obj.mverts(1:8,1)*l, obj.mverts(1:8,2)*l, obj.mverts(1:8,3)*l);

                    if obj.issolid
                        if (obj.rho > 0)
                            [obj.m_rb, obj.I_cm] = WBM.utilities.rb.massInertiaGObj('scub', obj.rho, l);
                        end
                    end
                otherwise
                    error('vbCuboid::createCuboid: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end

            if obj.isobstacle
                initMeshgrid(obj);
            end
        end

        function initMeshgrid(obj)
            % the cuboid is defined as an obstacle (wall, table, etc.):
            % define the domains of the internal grid points ...
            ms   = obj.mcub_msz;        % mesh size
            lx_h = obj.mcub_vtx_s(7,1); % half length of each side
            ly_h = obj.mcub_vtx_s(7,2);
            lz_h = obj.mcub_vtx_s(7,3);

            gp_x = (-lx_h+ms):ms:(lx_h-ms);
            gp_y = (-ly_h+ms):ms:(ly_h-ms);
            gp_z = (-lz_h+ms):ms:(lz_h-ms);

            % create the corresponding 3D-grid of the scaled cuboid ...
            sz = size(gp_x,2)*size(gp_y,2)*size(gp_z,2);
            if (sz > 1e6)
                warning('vbCuboid::initMeshgrid: Huge data arrays can cause the system to run Low On Memory!');
            end
            [obj.mcub_mg_s.xx, obj.mcub_mg_s.yy, obj.mcub_mg_s.zz] = meshgrid(gp_x, gp_y, gp_z);
            % set the coordinates matrix of the grid ...
            obj.mcub_mgC_s = horzcat(obj.mcub_mg_s.xx(:), obj.mcub_mg_s.yy(:), obj.mcub_mg_s.zz(:));
        end

        function setCuboidAtPosRotm(obj)
            % rotate the cuboid to the given orientation:
            id = WBM.utilities.isIdentity(obj.mcub_rotm);
            if ~id
                obj.mcub_vtx_sr = obj.mcub_vtx_s * obj.mcub_rotm.'; % = (R * vtx_s^T)^T
            else
                % no rotation (identity matrix) ...
                obj.mcub_vtx_sr = obj.mcub_vtx_s;
            end
            setCuboidAtPos(obj);

            if obj.isobstacle
                setMeshgridAtPosRotm(obj, id);
            end
        end

        function setCuboidAtPos(obj)
            % set all vertices and the CoM at the given origin point:
            for i = 1:3
                obj.vertices(1:8,i) = obj.mcub_vtx_sr(1:8,i) + obj.mcub_orig(i,1);
            end
            obj.com = obj.mcub_orig;
        end

        function setMeshgridAtPosRotm(obj, id)
            % rotate the coordinates of the grid points:
            if ~id % is not identity?
                mgC_sr = obj.mcub_mgC_s * obj.mcub_rotm.';
                sz = size(obj.mcub_mg_s.xx);

                obj.mcub_mg_sr.xx = reshape(mgC_sr(:,1), sz);
                obj.mcub_mg_sr.yy = reshape(mgC_sr(:,2), sz);
                obj.mcub_mg_sr.zz = reshape(mgC_sr(:,3), sz);
            else
                % no rotation is needed (identity matrix) ...
                obj.mcub_mg_sr.xx = obj.mcub_mg_s.xx;
                obj.mcub_mg_sr.yy = obj.mcub_mg_s.yy;
                obj.mcub_mg_sr.zz = obj.mcub_mg_s.zz;
            end
            setMeshgridAtPos(obj);
        end

        function setMeshgridAtPos(obj)
            % set the 3D-grid at the given origin point:
            obj.mgrid.X = obj.mcub_mg_sr.xx + obj.mcub_orig(1,1);
            obj.mgrid.Y = obj.mcub_mg_sr.yy + obj.mcub_orig(2,1);
            obj.mgrid.Z = obj.mcub_mg_sr.zz + obj.mcub_orig(3,1);
        end

    end
end
