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
% GNU General Public License for more details.
%
% A copy of the GNU Lesser General Public License can be found along
% with the WBML. If not, see <http://www.gnu.org/licenses/>.

classdef vbCylinder < WBM.vbObject
    % :class:`!vbCylinder` is a class to specify *circular cylinder objects*
    % for the environment scenario of the robot simulation.
    %
    % Attributes:
    %   origin (double, vector): (3 x 1) Cartesian position of the origin of the
    %                            cylinder. Default origin: :math:`[0, 0, 0]^T`.
    %   rotm   (double, matrix): (3 x 3) rotation matrix of the cylinder. If
    %                            undefined, the default orientation is the
    %                            *identity matrix*.
    %   tform  (double, matrix): (4 x 4) Transformation matrix of the cylinder.
    %                            If :attr:`origin` and :attr:`rotm` are undefined,
    %                            then by default, the transformation matrix is an
    %                            *identity matrix*.
    %   frame  (double, vector): (7 x 1) VQ-transformation vector (position and
    %                            orientation) of the cylinder. If :attr:`origin`
    %                            and :attr:`rotm` are undefined, then the frame
    %                            vector uses the default transformation vector
    %                            :attr:`~WBM.vbObject.DF_FRAME`.
    %
    %   description       (char, vector): Short description string about the
    %                                     cylinder object (default: *empty*).
    %   ismovable      (logical, scalar): Boolean flag to indicate if the cylinder
    %                                     object is movable (default: *false*).
    %   line_width      (double, scalar): Line width of the edges of the cylinder,
    %                                     specified as a positive value in points
    %                                     (default width: 0.4).
    %   edge_color (double/char, vector): Edge color of the cylinder, specified
    %                                     by a RGB-triplet or a color name
    %                                     (default color: *'black'*).
    %   face_color (double/char, vector): Face color of the cylinder, specified
    %                                     by a RGB-triplet or a color name
    %                                     (default color: :attr:`!wbmColor.lightsteelblue`).
    %   face_alpha      (double, scalar): Face transparency of the cylinder, specified
    %                                     by a scalar in range :math:`[0,1]`
    %                                     (default alpha: 0.2).
    %
    %   obj_type      (char, vector): Type of the object (*read only*), specified by
    %                                 one of these values:
    %
    %                                    - ``'obs'``: The cylinder defines an *obstacle* (for the robot).
    %                                    - ``'bdy'``: The cylinder is only an arbitrary *volume body* in
    %                                      the environment and not a specific obstacle.
    %
    %                                 The default object type is ``'bdy'``.
    %   isobstacle (logical, scalar): Boolean flag to indicate if the cylinder object
    %                                 is also defined as an obstacle (*read only*).
    %
    %                                 **Note:** The value will be set to *true*, if
    %                                 and only if :attr:`obj_type` is defined as an
    %                                 obstacle (*'obs'*), *false* otherwise (default).
    %   issolid    (logical, scalar): Boolean flag to indicate if the cylinder is
    %                                 a *solid* object (*read only*). Default: *false*.
    %
    %                                 **Note:** If a volumetric mass density is given,
    %                                 then the cylinder object is automatically defined
    %                                 as a *solid volume body*, i.e. the variable
    %                                 :attr:`!issolid` will be set to *true*.
    %   istool     (logical, scalar): Boolean flag to indicate if the cylinder defines
    %                                 also a tool to be used by the robot (*read only*).
    %                                 Default: *false*.
    %   init_frame  (double, vector): (7 x 1) initial frame vector (VQ-transformation)
    %                                 of the cylinder, in dependency of the given origin
    %                                 position and orientation (*read only*).
    %
    %                                 **Note:** If the attributes :attr:`origin` and
    %                                 :attr:`rotm` are not defined, then the initial
    %                                 frame uses the default transformation vector
    %                                 :attr:`~WBM.vbObject.DF_FRAME`.
    %   dimension   (double, vector): (1 x 3) vector of the form :math:`[\textrm{radius_{inner}, radius_{outer}, height}]`,
    %                                 which defines the dimensions of the cylinder (*read only*).
    %
    %                                 **Note:** If the dimensions of the cylinder are undefined,
    %                                 then the vector is by default a *0-vector*.
    %   vertices            (struct): Data structure for the *vertex coordinates* of the cylinder
    %                                 at the given origin frame (*read only*).
    %
    %                                 **Note:** Since the cylinder object is *linearized* by a
    %                                 *n*-gonal prism (polyhedron), the data structure consists
    %                                 of the fields ``X``, ``Y`` and ``Z`` which representing
    %                                 the x, y and z-coordinate vectors of each vertex of the
    %                                 given polyhedron.
    %   com   (double, vector): (3 x 1) Cartesian position of the center of mass
    %                           (CoM) of the cylinder relative to the given origin
    %                           (*read only*).
    %
    %                           **Note:** By default, the CoM of the cylinder is
    %                           placed at the origin.
    %   rho   (double, scalar): Volumetric mass density of the cylinder, specified as a
    %                           positive value in :math:`[\si{\kg/\m^3}]` (*read only*).
    %
    %                           **Note:** The cylinder is defined as a *solid volume body*
    %                           if and only if the density value :math:`\rho > 0`. If the
    %                           density of the cylinder is undefined, then the default
    %                           value is 0.
    %   m_rb  (double, scalar): Mass of the cylinder object (rigid body *rb*) in
    %                           :math:`[\si{\kg}]` (*read only*).
    %
    %                           **Note:** If :attr:`rho` is not defined, then by
    %                           default the mass value is 0.
    %   I_cm  (double, matrix): (3 x 3) inertia matrix of the cylinder at the
    %                           center of mass *cm* (*read only*).
    %
    %                           **Note:** If :attr:`rho` is not defined, then the
    %                           inertia of the cylinder is by default the *identity
    %                           matrix*.
    %   mgrid         (struct): Data structure for the *grid point coordinates* of
    %                           the internal 3D-meshgrid of the cylinder (*read only*).
    %
    %                           The 3D-meshgrid will be used in the robot simulation
    %                           to simulate the cylinder object as a solid obstacle.
    %
    %                           **Note:** The data structure consists of the fields
    %                           ``X``, ``Y`` and ``Z`` that are representing the x, y
    %                           and z-coordinates over a grid, specified as 3D-arrays
    %                           (with three inputs each). The meshgrid for the cylinder
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
        description@char         = '';  % annotation of the cylinder
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
        init_frame@double  vector = WBM.vbCylinder.DF_FRAME;
        dimension@double   vector = zeros(1,3);
        vertices@struct           = struct('X', [], 'Y', [], 'Z', []);
        com@double         vector = zeros(3,1);
        rho@double         scalar = 0;
        m_rb@double        scalar = 0;
        I_cm@double        matrix = eye(3,3);
        mgrid@struct              = struct('X', [], 'Y', [], 'Z', []);
    end

    properties(Access = private)
        mcyl_orig@double   vector = zeros(3,1); % initial Cartesian position of the origin [x,y,z]
        mcyl_rotm@double   matrix = eye(3,3);   % initial rotation matrix (orientation)
        mcyl_vtx_s@struct         = struct('xx', [], 'yy', [], 'zz', []); % vertex positions scaled to the height of the cylinder
        mcyl_vtx_sr@struct        = struct('xx', [], 'yy', [], 'zz', []); % vertex positions scaled and set to the given orientation
        mcyl_vtxC_s@double matrix = [];         % vertex coordinates matrix of the scaled cylinder
        mcyl_nfcs@int16    scalar = 20;         % number of faces around the circumference of the (linearized) cylinder (default: 20)
        mcyl_mg_s@struct          = struct('xx', [], 'yy', [], 'zz', []); % 3D-coordinates arrays of the scaled cylinder (s)
        mcyl_mg_sr@struct         = struct('xx', [], 'yy', [], 'zz', []); % 3D-coordinates arrays of the scaled and rotated cylinder (sr)
        mcyl_mgC_s@double  matrix = [];         % 3D-meshgrid coordinates matrix of the scaled cylinder
        mcyl_msz@double    scalar = 0.01;       % mesh size (msz) of the internal 3D-grid (default: 0.01)
    end

    methods
        function obj = vbCylinder(varargin)
            % Constructor.
            %
            % The constructor of the :class:`!vbCylinder` class can be called in
            % two different ways, where the *keyword arguments* in the square
            % brackets are optional:
            %
            %   - .. function:: vbCylinder(ri, ro, h[, orig, rotm[, obj_prop]])
            %   - .. function:: vbCylinder(r, h[, orig, rotm[, obj_prop]])
            %
            % The first option specifies a *circular hollow cylinder* (cylindrical
            % shell) as volume body object and the second one a *right circular
            % cylinder*.
            %
            % Arguments:
            %   varargin: Variable-length input argument list.
            %
            % Keyword Arguments:
            %   ri   (double, scalar): Inner radius of the cylindrical shell.
            %   ro   (double, scalar): Outer radius of the cylindrical shell.
            %   r    (double, scalar): Radius of the right circular cylinder.
            %   h    (double, scalar): Height of the cylinder along the z-direction.
            %   orig (double, vector): (3 x 1) origin position of the cylinder.
            %   rotm (double, matrix): (3 x 3) orientation matrix of the cylinder.
            %   obj_prop     (struct): Data structure for the *object properties*,
            %                          specified by following fields:
            %
            %                                 - ``line_width``: Line width of the edges of the cylinder.
            %                                 - ``edge_color``: Edge color of the cylinder (RGB-triplet
            %                                   or color name).
            %                                 - ``face_color``: Face color of the cylinder (RGB-triplet
            %                                   or color name).
            %                                 - ``face_alpha``: Face transparency of the cylinder in
            %                                   range :math:`[0,1]`.
            %
            %                              optional fields:
            %
            %                                 - ``description``: Annotation string of the
            %                                   cylinder object.
            %                                 - ``ismovable``: Boolean flag to indicate if
            %                                   the cylinder object is *movable* or *fixed*.
            %                                 - ``istool``: Boolean flag to indicate if the
            %                                   cylinder object is also a *tool* or not.
            %                                 - ``obj_type``: Object type, specified by the value
            %                                   *'obs'* or *'bdy'* (see :attr:`~vbCylinder.obj_type`).
            %                                 - ``rho``: Volumetric mass density of the cylinder
            %                                   object in :math:`[\si{\kg/\m^3}]`.
            %
            % Returns:
            %   obj: An instance of the :class:`!vbCylinder` class.
            switch nargin
                case 6 % circular cylindrical tube:
                    % ri       = varargin{1}
                    % ro       = varargin{2}
                    % h        = varargin{3}
                    % orig     = varargin{4}
                    % rotm     = varargin{5}
                    % obj_prop = varargin{6}

                    setObjData(obj, varargin{1,4:6});
                    createCylinder(obj, varargin{1,1:3});
                case 5
                    if isscalar(varargin{1,3})
                        % circular cylindrical tube:
                        % ri   = varargin{1}
                        % ro   = varargin{2}
                        % h    = varargin{3}
                        % orig = varargin{4}
                        % rotm = varargin{5}

                        setObjData(obj, varargin{1,4:5});
                        createCylinder(obj, varargin{1,1:3});
                    else
                        % circular solid cylinder:
                        % r        = varargin{1}
                        % h        = varargin{2}
                        % orig     = varargin{3}
                        % rotm     = varargin{4}
                        % obj_prop = varargin{5}

                        setObjData(obj, varargin{1,3:5});
                        createCylinder(obj, varargin{1,1:2});
                    end
                case 4 % simple solid cylinder:
                    % r    = varargin{1}
                    % h    = varargin{2}
                    % orig = varargin{3}
                    % rotm = varargin{4}

                    setObjData(obj, varargin{1,3:4});
                    createCylinder(obj, varargin{1,1:2});
                case 3 % simple cylindrical tube:
                    % ri   = varargin{1}
                    % ro   = varargin{2}
                    % h    = varargin{3}
                    createCylinder(obj, varargin{1,1:3});
                    return
                case 2 % simple solid cylinder:
                    % r = varargin{1}
                    % h = varargin{2}
                    createCylinder(obj, varargin{1,1:2});
                    return
                otherwise
                    error('vbCylinder::vbCylinder: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            setCylinderAtPosRotm(obj);
        end

        function setInitFrame(obj, varargin)
            % Sets the origin frame of the cylinder to the given initial position
            % and orientation.
            %
            % The method can be called in three different ways:
            %
            %   - .. function:: setInitFrame(p, R)
            %   - .. function:: setInitFrame(vqT)
            %   - .. function:: setInitFrame()
            %
            % If no parameters are given, then :meth:`!setInitFrame` uses for
            % the initial origin frame of the cylinder the current frame vector
            % of the property :attr:`~vbCylinder.init_frame`.
            %
            % Arguments:
            %   varargin: Variable-length input argument list.
            %
            % Keyword Arguments:
            %   p   (double, vector): (3 x 1) Cartesian position to specify the
            %                         *origin* of the cylinder.
            %   R   (double, matrix): (3 x 3) rotation matrix to specify the
            %                         *orientation* of the cylinder.
            %   vqT (double, vector): (7 x 1) VQ-transformation (position and
            %                         orientation in quaternions) to specify
            %                         the *origin frame* of the cylinder.
            switch nargin
                case 3
                    p = varargin{1,1};
                    R = varargin{1,2};

                    if WBM.utilities.isZero(R)
                        error('vbCylinder::setInitFrame: %s', WBM.wbmErrorMsg.UDEF_ROT_MAT);
                    end

                    obj.init_frame = WBM.utilities.tfms.posRotm2frame(p, R);
                    obj.mcyl_orig  = p;
                    obj.mcyl_rotm  = R;
                case 2
                    vqT = varargin{1,1};

                    if WBM.utilities.isZero(vqT(4:7))
                        error('vbCylinder::setInitFrame: %s', WBM.wbmErrorMsg.UDEF_QUAT_VEC);
                    end

                    [obj.mcyl_orig, obj.mcyl_rotm] = WBM.utilities.tfms.frame2posRotm(vqT);
                    obj.init_frame = vqT;
                case 1
                    % use the current initial frame ...
                    [obj.mcyl_orig, obj.mcyl_rotm] = WBM.utilities.tfms.frame2posRotm(obj.init_frame);
                otherwise
                    error('vbCylinder::setInitFrame: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            setCylinderAtPosRotm(obj);
        end

        function hgo = getGObj(obj)
            % Creates and draws the cylinder object and returns a handle to the
            % created graphics object.
            %
            % Returns:
            %   hgo: Handle to the chart surface graphics object of the created
            %        cylinder.
            hgo = surf(obj.vertices.X, obj.vertices.Y, obj.vertices.Z, 'LineWidth', obj.line_width, ...
                       'EdgeColor', obj.edge_color, 'FaceColor', obj.face_color, 'FaceAlpha', obj.face_alpha);
        end

        function hgo = updGObj(obj, hgo)
            % Updates the vertex coordinates of the cylinder.
            %
            % Arguments:
            %   hgo: Handle to the chart surface graphics object with the
            %        x, y and z-coordinate data of the cylinder.
            % Returns:
            %   hgo: Handle to the chart surface graphics object with the
            %        changed vertex coordinates of the cylinder.
            hgo.XData = obj.vertices.X;
            hgo.YData = obj.vertices.Y;
            hgo.ZData = obj.vertices.Z;
        end

        function hmg = drawMGrid(obj, pt_color)
            % Draws the meshgrid of the cylinder and returns a graphics object
            % handle to it.
            %
            % Arguments:
            %   pt_color (double/char, vector): Color of the grid points, specified
            %                                   by a RGB-triplet or a color name
            %                                   (default color: *'green'*).
            % Returns:
            %   hmg: Handle to the scatter series object of the generated meshgrid.
            if ~obj.isobstacle
                error('vbCylinder::drawMGrid: %s', WBM.wbmErrorMsg.OBJ_NOT_OBSTACLE);
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
            % i.e. inside, of the cylinder [#f1]_.
            %
            % Arguments:
            %   pt_pos (double, vector/matrix): A single position vector or a
            %                                   (n x 3) matrix with positions
            %                                   of some specified points.
            % Returns:
            %   result: (n x 1) vector with the logical results if the given
            %           points are below the surface of the cylinder. Each
            %           value is *true* if the given point is below the
            %           surface, *false* otherwise.

            % calculation approach is taken and adapted from:
            % <https://stackoverflow.com/questions/19899612/cylinder-with-filled-top-and-bottom-in-matlab> and
            % <https://de.mathworks.com/matlabcentral/answers/43534-how-to-speed-up-the-process-of-determining-if-a-point-is-inside-a-cylinder>
            r = obj.dimension(1,2); % outer radius
            h = obj.dimension(1,3);
            hgt_h = h*0.5;

            if isvector(pt_pos)
                WBM.utilities.chkfun.checkVecLen(pt_pos, 3, 'vbCylinder::ptInObj')
                pt_pos = pt_pos(:); % make sure that pt_pos is a column vector

                % get the distance of the given point to the CoM and rotate this
                % point back to the initial condition of the cylinder (rect. pos.
                % with CoM at origin 0):
                d_r = obj.mcyl_rotm.' * (pt_pos - obj.com); % back-rotated distance
                d_x = d_r(1,1);
                d_y = d_r(2,1);
                d_z = d_r(3,1);

                % check if the back-rotated point position is in the cylinder,
                % i.e. the point is below the surface of the cylinder:
                res1 = sqrt(d_x*d_x + d_y*d_y) < r*r;
                res2 = (d_z > -hgt_h) && (d_z < hgt_h); % bottom & top
            elseif ismatrix(pt_pos)
                [m, n] = size(pt_pos);
                if (n ~= 3)
                    error('vbCylinder::ptInObj: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
                end
                % get the distances of all points to the CoM:
                ds = zeros(m,3);
                cm = obj.com.';
                for i = 1:m
                    ds(i,1:3) = pt_pos(i,1:3) - cm;
                end
                ds_r = ds * obj.mcyl_rotm; % back-rotated distances,
                                           % ds_r = (R^T * ds^T)^T
                ds_x = ds_r(1:m,1);
                ds_y = ds_r(1:m,2);
                ds_z = ds_r(1:m,3);

                % check if all points are below the surface of the cylinder:
                res1 = sqrt(ds_x.*ds_x + ds_y.*ds_y) < r*r;
                res2 = (ds_z > -hgt_h) && (ds_z < hgt_h); % bottom & top
            else
                error('vbCylinder::ptInObj: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            % total result ...
            result = (res1 && res2);
        end

        function orig = get.origin(obj)
            orig = obj.mcyl_orig;
        end

        function set.origin(obj, orig)
            WBM.utilities.chkfun.checkCVecDim(orig, 3, 'vbCylinder::set.origin');
            obj.mcyl_orig = orig;

            setCylinderAtPos(obj);
            if obj.isobstacle
                setMeshgridAtPos(obj);
            end
        end

        function rotm = get.rotm(obj)
            rotm = obj.mcyl_rotm;
        end

        function set.rotm(obj, rotm)
            WBM.utilities.chkfun.checkMatDim(rotm, 3, 3, 'vbCylinder::set.rotm');
            if WBM.utilities.isZero(rotm)
                error('vbCylinder::set.rotm: %s', WBM.wbmErrorMsg.UDEF_ROT_MAT);
            end
            obj.mcyl_rotm = rotm;
            setCylinderAtPosRotm(obj);
        end

        function tform = get.tform(obj)
            tform = eye(4,4);
            tform(1:3,1:3) = obj.mcyl_rotm;
            tform(1:3,4)   = obj.mcyl_orig;
        end

        function set.tform(obj, tform)
            [obj.mcyl_orig, R] = WBM.utilities.tfms.tform2posRotm(tform);

            if WBM.utilities.isZero(R)
                error('vbCylinder::set.tform: %s', WBM.wbmErrorMsg.UDEF_ROT_MAT);
            end
            obj.mcyl_rotm = R;
            setCylinderAtPosRotm(obj);
        end

        function vqT = get.frame(obj)
            vqT = zeros(7,1);
            vqT(1:3,1) = obj.mcyl_orig;
            vqT(4:7,1) = WBM.utilities.tfms.rotm2quat(obj.mcyl_rotm);
        end

        function set.frame(obj, vqT)
            if WBM.utilities.isZero(vqT(4:7))
                error('vbCylinder::set.frame: %s', WBM.wbmErrorMsg.UDEF_QUAT_VEC);
            end
            [obj.mcyl_orig, obj.mcyl_rotm] = WBM.utilities.tfms.frame2posRotm(vqT);
            setCylinderAtPosRotm(obj);
        end

    end

    methods(Access = private)
        function setObjData(obj, orig, rotm, obj_prop)
            if WBM.utilities.isZero(rotm)
                error('vbCylinder::setObjData: %s', WBM.wbmErrorMsg.UDEF_ROT_MAT);
            end
            obj.init_frame = WBM.utilities.tfms.posRotm2frame(orig, rotm);
            obj.mcyl_orig  = orig;
            obj.mcyl_rotm  = rotm;

            if (nargin == 4)
                % set the object properties of the cylinder ...
                if ~isstruct(obj_prop)
                    error('vbCylinder::setObjData: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
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

                if isfield(obj_prop, 'nfcs')
                    obj.mcyl_nfcs = obj_prop.nfcs;
                end

                if isfield(obj_prop, 'obj_type')
                    switch obj_prop.obj_type
                        case 'bdy'
                            obj.isobstacle = false;
                        case 'obs'
                            obj.isobstacle = true;
                            if isfield(obj_prop, 'mesh_size')
                                obj.mcyl_msz = obj_prop.mesh_size;
                            end
                        otherwise
                            error('vbCylinder::setObjData: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
                    end
                    obj.obj_type = obj_prop.obj_type;
                end

                if isfield(obj_prop, 'rho')
                    obj.issolid = true;
                    obj.rho     = obj_prop.rho;
                end
            end
        end

        function createCylinder(obj, varargin)
            switch nargin
                case 4
                    % circular cylindrical tube:
                    ri = varargin{1,1};
                    ro = varargin{1,2};
                    h  = varargin{1,3};

                    obj.dimension = horzcat(ri, ro, h);

                    [obj.mcyl_vtx_s.xx, obj.mcyl_vtx_s.yy, zz] = WBM.utilities.vb.solidRing(ri, ro, obj.mcyl_nfcs, 'rect');
                    obj.mcyl_vtx_s.zz = zz * h;

                    if obj.issolid
                        if (obj.rho > 0)
                            % calculate the mass and the inertia of the cylindrical tube ...
                            [obj.m_rb, obj.I_cm] = WBM.utilities.rb.massInertiaGObj('cylt', obj.rho, ri, ro, h);
                        end
                    end
                    r = ro;
                case 3
                    % circular solid cylinder:
                    r = varargin{1,1};
                    h = varargin{1,2};

                    obj.dimension = horzcat(0, r, h);

                    [obj.mcyl_vtx_s.xx, obj.mcyl_vtx_s.yy, zz] = WBM.utilities.vb.solidCylinder(r, obj.mcyl_nfcs);
                    obj.mcyl_vtx_s.zz = zz * h;

                    if obj.issolid
                        if (obj.rho > 0)
                            % mass and the inertia of the solid cylinder ...
                            [obj.m_rb, obj.I_cm] = WBM.utilities.rb.massInertiaGObj('scyl', obj.rho, r, h);
                        end
                    end
                otherwise
                    error('vbCylinder::createCylinder: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            % set the scaled vertex coordinates matrix ...
            obj.mcyl_vtxC_s = horzcat(obj.mcyl_vtx_s.xx(:), obj.mcyl_vtx_s.yy(:), obj.mcyl_vtx_s.zz(:));

            if obj.isobstacle
                initMeshgrid(obj, r, h);
            end
        end

        function initMeshgrid(obj, r, h)
            % the cylinder is defined as an obstacle:
            % calculation approach of the interior meshgrid is adapted from:
            % <https://de.mathworks.com/matlabcentral/answers/181569-fill-the-interior-of-a-cylinder-surface-surf-generated-by-parametric-equations>
            ms   = obj.mcyl_msz; % mesh size
            h_mg = (h*0.5) - ms; % half height of the (internal) meshgrid
            r_mg = r - ms;       % radius of the meshgrid

            % create a cubic 3D-grid that covers the scaled cylinder:
            gp_x = -r_mg:ms:r_mg;
            gp_y =  gp_x;
            gp_z = -h_mg:ms:h_mg;

            s  = size(gp_x,2);
            sz = s*s*size(gp_z,2);
            if (sz > 1e6)
                warning('vbCylinder::initMeshgrid: Huge data arrays can cause the system to run Low On Memory!');
            end
            [xx, yy, zz] = meshgrid(gp_x, gp_y, gp_z);

            % calculate the distances of all grid points ...
            ds_gp = sqrt(xx.*xx + yy.*yy);

            % find the distances that are inside of the cylinder and
            % create a logical index matrix of interior grid points:
            gp_idx = logical(zeros(size(ds_gp)));
            gp_idx(ds_gp <= r_mg) = 1;

            % get the interior grid points ...
            obj.mcyl_mg_s.xx = xx(gp_idx);
            obj.mcyl_mg_s.yy = yy(gp_idx);
            obj.mcyl_mg_s.zz = zz(gp_idx);
            % set the coordinates matrix of the grid ...
            obj.mcyl_mgC_s = horzcat(obj.mcyl_mg_s.xx(:), obj.mcyl_mg_s.yy(:), obj.mcyl_mg_s.zz(:));
        end

        function setCylinderAtPosRotm(obj)
            % rotate the cylinder to the given orientation:
            id = WBM.utilities.isIdentity(obj.mcyl_rotm);
            if ~id
                vtxC_sr = obj.mcyl_vtxC_s * obj.mcyl_rotm.'; % = (R * vtx_s^T)^T
                sz = size(obj.mcyl_vtx_s.xx);

                obj.mcyl_vtx_sr.xx = reshape(vtxC_sr(:,1), sz);
                obj.mcyl_vtx_sr.yy = reshape(vtxC_sr(:,2), sz);
                obj.mcyl_vtx_sr.zz = reshape(vtxC_sr(:,3), sz);
            else
                % no rotation (identity matrix) ...
                obj.mcyl_vtx_sr.xx = obj.mcyl_vtx_s.xx;
                obj.mcyl_vtx_sr.yy = obj.mcyl_vtx_s.yy;
                obj.mcyl_vtx_sr.zz = obj.mcyl_vtx_s.zz;
            end
            setCylinderAtPos(obj);

            if obj.isobstacle
                setMeshgridAtPosRotm(obj, id);
            end
        end

        function setCylinderAtPos(obj)
            % set all vertices and the CoM at the given origin point:
            obj.vertices.X = obj.mcyl_vtx_sr.xx + obj.mcyl_orig(1,1);
            obj.vertices.Y = obj.mcyl_vtx_sr.yy + obj.mcyl_orig(2,1);
            obj.vertices.Z = obj.mcyl_vtx_sr.zz + obj.mcyl_orig(3,1);
            obj.com        = obj.mcyl_orig;
        end

        function setMeshgridAtPosRotm(obj, id)
            % rotate the coordinates of the grid points:
            if ~id % is not identity?
                mgC_sr = obj.mcyl_mgC_s * obj.mcyl_rotm.';
                sz = size(obj.mcyl_mg_s.xx);

                obj.mcyl_mg_sr.xx = reshape(mgC_sr(:,1), sz);
                obj.mcyl_mg_sr.yy = reshape(mgC_sr(:,2), sz);
                obj.mcyl_mg_sr.zz = reshape(mgC_sr(:,3), sz);
            else
                % no rotation is needed (identity matrix) ...
                obj.mcyl_mg_sr.xx = obj.mcyl_mg_s.xx;
                obj.mcyl_mg_sr.yy = obj.mcyl_mg_s.yy;
                obj.mcyl_mg_sr.zz = obj.mcyl_mg_s.zz;
            end
            setMeshgridAtPos(obj);
        end

        function setMeshgridAtPos(obj)
            % set the 3D-grid at the given origin point:
            obj.mgrid.X = obj.mcyl_mg_sr.xx + obj.mcyl_orig(1,1);
            obj.mgrid.Y = obj.mcyl_mg_sr.yy + obj.mcyl_orig(2,1);
            obj.mgrid.Z = obj.mcyl_mg_sr.zz + obj.mcyl_orig(3,1);
        end

    end
end
