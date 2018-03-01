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

classdef vbSphere < WBM.vbObject
    % :class:`!vbSphere` is a class to specify *sphere objects* for the
    % environment scenario of the robot simulation.
    %
    % Attributes:
    %   origin (double, vector): (3 x 1) Cartesian position of the origin of the
    %                            sphere. Default origin: :math:`[0, 0, 0]^T`.
    %   rotm   (double, matrix): (3 x 3) rotation matrix of the sphere. If
    %                            undefined, the default orientation is the
    %                            *identity matrix*.
    %   tform  (double, matrix): (4 x 4) Transformation matrix of the sphere.
    %                            If :attr:`origin` and :attr:`rotm` are undefined,
    %                            then by default, the transformation matrix is an
    %                            *identity matrix*.
    %   frame  (double, vector): (7 x 1) VQ-transformation vector (position and
    %                            orientation) of the sphere. If :attr:`origin`
    %                            and :attr:`rotm` are undefined, then the frame
    %                            vector uses the default transformation vector
    %                            :attr:`~WBM.vbObject.DF_FRAME`.
    %
    %   description       (char, vector): Short description string about the
    %                                     sphere object (default: *empty*).
    %   ismovable      (logical, scalar): Boolean flag to indicate if the sphere
    %                                     object is movable (default: *false*).
    %   line_width      (double, scalar): Line width of the edges of the sphere,
    %                                     specified as a positive value in points
    %                                     (default width: 0.4).
    %   edge_color (double/char, vector): Edge color of the sphere, specified
    %                                     by a RGB-triplet or a color name
    %                                     (default color: *'black'*).
    %   face_color (double/char, vector): Face color of the sphere, specified
    %                                     by a RGB-triplet or a color name
    %                                     (default color: :attr:`!wbmColor.lightsteelblue`).
    %   face_alpha      (double, scalar): Face transparency of the sphere, specified
    %                                     by a scalar in range :math:`[0,1]`
    %                                     (default alpha: 0.2).
    %
    %   obj_type      (char, vector): Type of the object (*read only*), specified by
    %                                 one of these values:
    %
    %                                    - ``'obs'``: The sphere defines an *obstacle* (for the robot).
    %                                    - ``'bdy'``: The sphere is only an arbitrary *volume body* in
    %                                      the environment and not a specific obstacle.
    %
    %                                 The default object type is ``'bdy'``.
    %   isobstacle (logical, scalar): Boolean flag to indicate if the sphere object
    %                                 is also defined as an obstacle (*read only*).
    %
    %                                 **Note:** The value will be set to *true*, if
    %                                 and only if :attr:`obj_type` is defined as an
    %                                 obstacle (*'obs'*), *false* otherwise (default).
    %   issolid    (logical, scalar): Boolean flag to indicate if the sphere is
    %                                 a *solid* object (*read only*). Default: *false*.
    %
    %                                 **Note:** If a volumetric mass density is given,
    %                                 then the sphere object is automatically defined
    %                                 as a *solid volume body*, i.e. the variable
    %                                 :attr:`!issolid` will be set to *true*.
    %   istool     (logical, scalar): Boolean flag to indicate if the sphere defines
    %                                 also a tool to be used by the robot (*read only*).
    %                                 Default: *false*.
    %   init_frame  (double, vector): (7 x 1) initial frame vector (VQ-transformation)
    %                                 of the sphere, in dependency of the given origin
    %                                 position and orientation (*read only*).
    %
    %                                 **Note:** If the attributes :attr:`origin` and
    %                                 :attr:`rotm` are not defined, then the initial
    %                                 frame uses the default transformation vector
    %                                 :attr:`~WBM.vbObject.DF_FRAME`.
    %   dimension   (double, vector): (1 x 2) vector of the form :math:`[\textrm{radius_{inner}, radius_{outer}}]`,
    %                                 which defines the dimensions of the sphere (*read only*).
    %
    %                                 **Note:** If the dimensions of the sphere are undefined,
    %                                 then the vector is by default a *0-vector*.
    %   vertices            (struct): Data structure for the *vertex coordinates* of the sphere
    %                                 at the given origin frame (*read only*).
    %
    %                                 **Note:** Since the sphere object is *approximated* by
    %                                 a (n x n) *spherical polyhedron*, the data structure
    %                                 consists of the fields ``X``, ``Y`` and ``Z`` which
    %                                 representing the x, y and z-coordinate vectors of each
    %                                 vertex of the given spherical polyhedron.
    %   com   (double, vector): (3 x 1) Cartesian position of the center of mass
    %                           (CoM) of the sphere relative to the given origin
    %                           (*read only*).
    %
    %                           **Note:** By default, the CoM of the sphere is
    %                           placed at the origin.
    %   rho   (double, scalar): Volumetric mass density of the sphere, specified as a
    %                           positive value in :math:`[\si{\kg/\m^3}]` (*read only*).
    %
    %                           **Note:** The sphere is defined as a *solid volume body*
    %                           if and only if the density value :math:`\rho > 0`. If the
    %                           density of the sphere is undefined, then the default
    %                           value is 0.
    %   m_rb  (double, scalar): Mass of the sphere object (rigid body *rb*) in
    %                           :math:`[\si{\kg}]` (*read only*).
    %
    %                           **Note:** If :attr:`rho` is not defined, then by
    %                           default the mass value is 0.
    %   I_cm  (double, matrix): (3 x 3) inertia matrix of the sphere at the
    %                           center of mass *cm* (*read only*).
    %
    %                           **Note:** If :attr:`rho` is not defined, then the
    %                           inertia of the sphere is by default the *identity
    %                           matrix*.
    %   mgrid         (struct): Data structure for the *grid point coordinates* of
    %                           the internal 3D-meshgrid of the sphere (*read only*).
    %
    %                           The 3D-meshgrid will be used in the robot simulation
    %                           to simulate the sphere object as a solid obstacle.
    %
    %                           **Note:** The data structure consists of the fields
    %                           ``X``, ``Y`` and ``Z`` that are representing the x, y
    %                           and z-coordinates over a grid, specified as 3D-arrays
    %                           (with three inputs each). The meshgrid for the sphere
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
        description@char         = '';  % annotation of the sphere
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
        init_frame@double  vector = WBM.vbSphere.DF_FRAME;
        dimension@double   vector = zeros(1,2);
        vertices@struct           = struct('X', [], 'Y', [], 'Z', []);
        com@double         vector = zeros(3,1);
        rho@double         scalar = 0;
        m_rb@double        scalar = 0;
        I_cm@double        matrix = eye(3,3);
        mgrid@struct              = struct('X', [], 'Y', [], 'Z', []);
    end

    properties(Access = private)
        msph_orig@double   vector = zeros(3,1); % initial Cartesian position of the origin [x,y,z]
        msph_rotm@double   matrix = eye(3,3);   % initial rotation matrix (orientation)
        msph_vtx_s@struct         = struct('xx', [], 'yy', [], 'zz', []); % vertex positions scaled to the sphere radius
        msph_vtx_sr@struct        = struct('xx', [], 'yy', [], 'zz', []); % vertex positions scaled and set to the given orientation
        msph_vtxC_s@double matrix = [];         % vertex coordinates matrix of the scaled sphere
        msph_nfcs@double   scalar = 20;         % number of horizontal and vertical faces to generate a n-by-n sphere (default: 20)
        msph_mg_s@struct          = struct('xx', [], 'yy', [], 'zz', []); % 3D-coordinates arrays of the scaled sphere (s)
        msph_mg_sr@struct         = struct('xx', [], 'yy', [], 'zz', []); % 3D-coordinates arrays of the scaled and rotated sphere (sr)
        msph_mgC_s@double  matrix = [];         % 3D-meshgrid coordinates matrix of the scaled sphere
        msph_msz@double    scalar = 0.01;       % mesh size (msz) of the internal 3D-grid (default: 0.01)
    end

    methods
        function obj = vbSphere(varargin)
            % Constructor.
            %
            % The constructor of the :class:`!vbSphere` class can be called in
            % two different ways, where the *keyword arguments* in the square
            % brackets are optional:
            %
            %   - .. function:: vbSphere(ri, ro[, orig, rotm[, obj_prop]])
            %   - .. function:: vbSphere(r[, orig, rotm[, obj_prop]])
            %
            % The first option specifies a *hollow sphere* (sphere shell) as
            % volume body object and the second one a *solid sphere* (ball).
            %
            % Arguments:
            %   varargin: Variable-length input argument list.
            %
            % Keyword Arguments:
            %   ri   (double, scalar): Inner radius of the sphere shell.
            %   ro   (double, scalar): Outer radius of the sphere shell.
            %   r    (double, scalar): Radius of the solid sphere.
            %   orig (double, vector): (3 x 1) origin position of the sphere.
            %   rotm (double, matrix): (3 x 3) orientation matrix of the sphere.
            %   obj_prop     (struct): Data structure for the *object properties*,
            %                          specified by following fields:
            %
            %                                 - ``line_width``: Line width of the edges of the sphere.
            %                                 - ``edge_color``: Edge color of the sphere (RGB-triplet
            %                                   or color name).
            %                                 - ``face_color``: Face color of the sphere (RGB-triplet
            %                                   or color name).
            %                                 - ``face_alpha``: Face transparency of the sphere in
            %                                   range :math:`[0,1]`.
            %
            %                              optional fields:
            %
            %                                 - ``description``: Annotation string of the
            %                                   sphere object.
            %                                 - ``ismovable``: Boolean flag to indicate if
            %                                   the sphere object is *movable* or *fixed*.
            %                                 - ``istool``: Boolean flag to indicate if the
            %                                   sphere object is also a *tool* or not.
            %                                 - ``obj_type``: Object type, specified by the value
            %                                   *'obs'* or *'bdy'* (see :attr:`~vbSphere.obj_type`).
            %                                 - ``rho``: Volumetric mass density of the sphere
            %                                   object in :math:`[\si{\kg/\m^3}]`.
            %
            % Returns:
            %   obj: An instance of the :class:`!vbSphere` class.
            switch nargin
                case 5 % spherical shell:
                    % ri       = varargin{1}
                    % ro       = varargin{2}
                    % orig     = varargin{3}
                    % rotm     = varargin{4}
                    % obj_prop = varargin{5}

                    setObjData(obj, varargin{1,3:5});
                    createSphere(obj, varargin{1,1:2});
                case 4
                    if isscalar(varargin{1,2})
                        % spherical shell:
                        % ri   = varargin{1}
                        % ro   = varargin{2}
                        % orig = varargin{3}
                        % rotm = varargin{4}

                        setObjData(obj, varargin{1,3:4});
                        createSphere(obj, varargin{1,1:2});
                    else
                        % solid sphere:
                        % r        = varargin{1}
                        % orig     = varargin{2}
                        % rotm     = varargin{3}
                        % obj_prop = varargin{4}

                        setObjData(obj, varargin{1,2:4});
                        createSphere(obj, varargin{1,1});
                    end
                case 3 % simple solid sphere:
                    % r    = varargin{1}
                    % orig = varargin{2}
                    % rotm = varargin{3}

                    setObjData(obj, varargin{1,2:3});
                    createSphere(obj, varargin{1,1});
                case 2 % simple spherical shell:
                    % ri   = varargin{1}
                    % ro   = varargin{2}
                    createSphere(obj, varargin{1,1:2});
                    return
                case 1 % simple solid sphere:
                    % r = varargin{1}
                    createSphere(obj, varargin{1,1});
                    return
                otherwise
                    error('vbSphere::vbSphere: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            setSphereAtPosRotm(obj);
        end

        function setInitFrame(obj, varargin)
            % Sets the origin frame of the sphere to the given initial position
            % and orientation.
            %
            % The method can be called in three different ways:
            %
            %   - .. function:: setInitFrame(p, R)
            %   - .. function:: setInitFrame(vqT)
            %   - .. function:: setInitFrame()
            %
            % If no parameters are given, then :meth:`!setInitFrame` uses for
            % the initial origin frame of the sphere the current frame vector
            % of the property :attr:`~vbSphere.init_frame`.
            %
            % Arguments:
            %   varargin: Variable-length input argument list.
            %
            % Keyword Arguments:
            %   p   (double, vector): (3 x 1) Cartesian position to specify the
            %                         *origin* of the sphere.
            %   R   (double, matrix): (3 x 3) rotation matrix to specify the
            %                         *orientation* of the sphere.
            %   vqT (double, vector): (7 x 1) VQ-transformation (position and
            %                         orientation in quaternions) to specify
            %                         the *origin frame* of the sphere.
            switch nargin
                case 3
                    p = varargin{1,1};
                    R = varargin{1,2};

                    if WBM.utilities.isZero(R)
                        error('vbSphere::setInitFrame: %s', WBM.wbmErrorMsg.UDEF_ROT_MAT);
                    end

                    obj.init_frame = WBM.utilities.tfms.posRotm2frame(p, R);
                    obj.msph_orig  = p;
                    obj.msph_rotm  = R;
                case 2
                    vqT = varargin{1,1};

                    if WBM.utilities.isZero(vqT(4:7))
                        error('vbSphere::setInitFrame: %s', WBM.wbmErrorMsg.UDEF_QUAT_VEC);
                    end

                    [obj.msph_orig, obj.msph_rotm] = WBM.utilities.tfms.frame2posRotm(vqT);
                    obj.init_frame = vqT;
                case 1
                    % use the current initial frame ...
                    [obj.msph_orig, obj.msph_rotm] = WBM.utilities.tfms.frame2posRotm(obj.init_frame);
                otherwise
                    error('vbSphere::setInitFrame: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            setSphereAtPosRotm(obj);
        end

        function hgo = getGObj(obj)
            % Creates and draws the sphere object and returns a handle to the
            % created graphics object.
            %
            % Returns:
            %   hgo: Handle to the chart surface graphics object of the created
            %        sphere.
            hgo = surf(obj.vertices.X, obj.vertices.Y, obj.vertices.Z, 'LineWidth', obj.line_width, ...
                       'EdgeColor', obj.edge_color, 'FaceColor', obj.face_color, 'FaceAlpha', obj.face_alpha);
        end

        function hgo = updGObj(obj, hgo)
            % Updates the vertex coordinates of the sphere.
            %
            % Arguments:
            %   hgo: Handle to the chart surface graphics object with the
            %        x, y and z-coordinate data of the sphere.
            % Returns:
            %   hgo: Handle to the chart surface graphics object with the
            %        changed vertex coordinates of the sphere.
            hgo.XData = obj.vertices.X;
            hgo.YData = obj.vertices.Y;
            hgo.ZData = obj.vertices.Z;
        end

        function hmg = drawMGrid(obj, pt_color)
            % Draws the meshgrid of the sphere and returns a graphics object
            % handle to it.
            %
            % Arguments:
            %   pt_color (double/char, vector): Color of the grid points, specified
            %                                   by a RGB-triplet or a color name
            %                                   (default color: *'green'*).
            % Returns:
            %   hmg: Handle to the scatter series object of the generated meshgrid.
            if ~obj.isobstacle
                error('vbSphere::drawMGrid: %s', WBM.wbmErrorMsg.OBJ_NOT_OBSTACLE);
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
            % i.e. inside, of the sphere.
            %
            % Arguments:
            %   pt_pos (double, vector/matrix): A single position vector or a
            %                                   (n x 3) matrix with positions
            %                                   of some specified points.
            % Returns:
            %   result: (n x 1) vector with the logical results if the given
            %           points are below the surface of the sphere. Each
            %           value is *true* if the given point is below the
            %           surface, *false* otherwise.

            r = obj.dimension(1,2); % outer radius

            if isvector(pt_pos)
                WBM.utilities.chkfun.checkVecLen(pt_pos, 3, 'vbSphere::ptInObj')
                pt_pos = pt_pos(:); % make sure that pt_pos is a column vector

                % get the distance of the given point to the sphere's CoM:
                d   = pt_pos - obj.com;
                d_x = d(1,1);
                d_y = d(2,1);
                d_z = d(3,1);

                % check if the point position is below the surface of the sphere:
                result = sqrt(d_x*d_x + d_y*d_y + d_z*d_z) < r;
            elseif ismatrix(pt_pos)
                [m, n] = size(pt_pos);
                if (n ~= 3)
                    error('vbSphere::ptInObj: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
                end
                % get the distances of all points to the CoM:
                ds = zeros(m,3);
                cm = obj.com.';
                for i = 1:m
                    ds(i,1:3) = pt_pos(i,1:3) - cm;
                end
                ds_x = ds(1:m,1);
                ds_y = ds(1:m,2);
                ds_z = ds(1:m,3);

                % check if all points are below the surface of the sphere:
                result = sqrt(ds_x.*ds_x + ds_y.*ds_y + ds_z.*ds_z) < r;
            else
                error('vbSphere::ptInObj: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
        end

        function orig = get.origin(obj)
            orig = obj.msph_orig;
        end

        function set.origin(obj, orig)
            WBM.utilities.chkfun.checkCVecDim(orig, 3, 'vbSphere::set.origin');
            obj.msph_orig = orig;

            setSphereAtPos(obj);
            if obj.isobstacle
                setMeshgridAtPos(obj);
            end
        end

        function rotm = get.rotm(obj)
            rotm = obj.msph_rotm;
        end

        function set.rotm(obj, rotm)
            WBM.utilities.chkfun.checkMatDim(rotm, 3, 3, 'vbSphere::set.rotm');
            if WBM.utilities.isZero(rotm)
                error('vbSphere::set.rotm: %s', WBM.wbmErrorMsg.UDEF_ROT_MAT);
            end
            obj.msph_rotm = rotm;
            setSphereAtPosRotm(obj);
        end

        function tform = get.tform(obj)
            tform = eye(4,4);
            tform(1:3,1:3) = obj.msph_rotm;
            tform(1:3,4)   = obj.msph_orig;
        end

        function set.tform(obj, tform)
            [obj.msph_orig, R] = WBM.utilities.tfms.tform2posRotm(tform);

            if WBM.utilities.isZero(R)
                error('vbSphere::set.tform: %s', WBM.wbmErrorMsg.UDEF_ROT_MAT);
            end
            obj.msph_rotm = R;
            setSphereAtPosRotm(obj);
        end

        function vqT = get.frame(obj)
            vqT = zeros(7,1);
            vqT(1:3,1) = obj.msph_orig;
            vqT(4:7,1) = WBM.utilities.tfms.rotm2quat(obj.msph_rotm);
        end

        function set.frame(obj, vqT)
            if WBM.utilities.isZero(vqT(4:7))
                error('vbSphere::set.frame: %s', WBM.wbmErrorMsg.UDEF_QUAT_VEC);
            end
            [obj.msph_orig, obj.msph_rotm] = WBM.utilities.tfms.frame2posRotm(vqT);
            setSphereAtPosRotm(obj);
        end

    end

    methods(Access = private)
        function setObjData(obj, orig, rotm, obj_prop)
            if WBM.utilities.isZero(rotm)
                error('vbSphere::setObjData: %s', WBM.wbmErrorMsg.UDEF_ROT_MAT);
            end
            obj.init_frame = WBM.utilities.tfms.posRotm2frame(orig, rotm);
            obj.msph_orig  = orig;
            obj.msph_rotm  = rotm;

            if (nargin == 4)
                % set the object properties of the sphere ...
                if ~isstruct(obj_prop)
                    error('vbSphere::setObjData: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
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
                    obj.msph_nfcs = obj_prop.nfcs;
                end

                if isfield(obj_prop, 'obj_type')
                    switch obj_prop.obj_type
                        case 'bdy'
                            obj.isobstacle = false;
                        case 'obs'
                            obj.isobstacle = true;
                        otherwise
                            error('vbSphere::setObjData: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
                    end
                    obj.obj_type = obj_prop.obj_type;
                end

                if isfield(obj_prop, 'rho')
                    obj.issolid = true;
                    obj.rho     = obj_prop.rho;
                end
            end
        end

        function createSphere(obj, varargin)
            [xx, yy, zz] = sphere(obj.msph_nfcs);

            switch nargin
                case 3
                    % circular spherical shell:
                    ri = varargin{1,1};
                    ro = varargin{1,2};

                    obj.dimension = horzcat(ri, ro);

                    % inner (hollow) sphere ...
                    obj.msph_vtx_s.xx = xx * ri;
                    obj.msph_vtx_s.yy = yy * ri;
                    obj.msph_vtx_s.zz = zz * ri;
                    % outer sphere (hull) ...
                    obj.msph_vtx_s.xx = vertcat(obj.msph_vtx_s.xx, xx * ro);
                    obj.msph_vtx_s.yy = vertcat(obj.msph_vtx_s.yy, yy * ro);
                    obj.msph_vtx_s.zz = vertcat(obj.msph_vtx_s.zz, zz * ro);

                    if obj.issolid
                        if (obj.rho > 0)
                            % calculate the mass and the inertia of the spherical shell ...
                            [obj.m_rb, obj.I_cm] = WBM.utilities.rb.massInertiaGObj('sphs', obj.rho, ri, ro);
                        end
                    end
                    r = ro;
                case 2
                    % circular solid sphere:
                    r = varargin{1,1};

                    obj.dimension = horzcat(0, r);

                    obj.msph_vtx_s.xx = xx * r;
                    obj.msph_vtx_s.yy = yy * r;
                    obj.msph_vtx_s.zz = zz * r;

                    if obj.issolid
                        if (obj.rho > 0)
                            % mass and the inertia of the solid sphere ...
                            [obj.m_rb, obj.I_cm] = WBM.utilities.rb.massInertiaGObj('ssph', obj.rho, r);
                        end
                    end
                otherwise
                    error('vbSphere::createSphere: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            % set the scaled vertex coordinates matrix ...
            obj.msph_vtxC_s = horzcat(obj.msph_vtx_s.xx(:), obj.msph_vtx_s.yy(:), obj.msph_vtx_s.zz(:));

            if obj.isobstacle
                initMeshgrid(obj, r);
            end
        end

        function initMeshgrid(obj, r)
            % the sphere is defined as an obstacle:
            ms   = obj.msph_msz; % mesh size
            r_mg = r - ms;       % radius of the internal meshgrid

            % create a cubic 3D-grid that covers the scaled sphere:
            gp_x = -r_mg:ms:r_mg;
            gp_y =  gp_x;
            gp_z =  gp_x;

            s  = size(gp_x,2);
            sz = s*s*s;
            if (sz > 1e6)
                warning('vbSphere::initMeshgrid: Huge data arrays can cause the system to run Low On Memory!');
            end
            [xx, yy, zz] = meshgrid(gp_x, gp_y, gp_z);

            % calculate the distances of all grid points ...
            ds_gp = sqrt(xx.*xx + yy.*yy + zz.*zz);

            % find the distances that are inside of the sphere and
            % create a logical index matrix of interior grid points:
            gp_idx = logical(zeros(size(ds_gp)));
            gp_idx(ds_gp <= r_mg) = 1;

            % get the interior grid points ...
            obj.msph_mg_s.xx = xx(gp_idx);
            obj.msph_mg_s.yy = yy(gp_idx);
            obj.msph_mg_s.zz = zz(gp_idx);
            % set the coordinates matrix of the grid ...
            obj.msph_mgC_s = horzcat(obj.msph_mg_s.xx(:), obj.msph_mg_s.yy(:), obj.msph_mg_s.zz(:));
        end

        function setSphereAtPosRotm(obj)
            % rotate the sphere to the given orientation:
            id = WBM.utilities.isIdentity(obj.msph_rotm);
            if ~id
                vtxC_sr = obj.msph_vtxC_s * obj.msph_rotm.'; % = (R * vtx_s^T)^T
                sz = size(obj.msph_vtx_s.xx);

                obj.msph_vtx_sr.xx = reshape(vtxC_sr(:,1), sz);
                obj.msph_vtx_sr.yy = reshape(vtxC_sr(:,2), sz);
                obj.msph_vtx_sr.zz = reshape(vtxC_sr(:,3), sz);
            else
                % no rotation (identity matrix) ...
                obj.msph_vtx_sr.xx = obj.msph_vtx_s.xx;
                obj.msph_vtx_sr.yy = obj.msph_vtx_s.yy;
                obj.msph_vtx_sr.zz = obj.msph_vtx_s.zz;
            end
            setSphereAtPos(obj);

            if obj.isobstacle
                setMeshgridAtPosRotm(obj, id);
            end
        end

        function setSphereAtPos(obj)
            % set all vertices and the CoM at the given origin point:
            obj.vertices.X = obj.msph_vtx_sr.xx + obj.msph_orig(1,1);
            obj.vertices.Y = obj.msph_vtx_sr.yy + obj.msph_orig(2,1);
            obj.vertices.Z = obj.msph_vtx_sr.zz + obj.msph_orig(3,1);
            obj.com        = obj.msph_orig;
        end

        function setMeshgridAtPosRotm(obj, id)
            % rotate the coordinates of the grid points:
            if ~id % is not identity?
                mgC_sr = obj.msph_mgC_s * obj.msph_rotm.';
                sz = size(obj.msph_mg_s.xx);

                obj.msph_mg_sr.xx = reshape(mgC_sr(:,1), sz);
                obj.msph_mg_sr.yy = reshape(mgC_sr(:,2), sz);
                obj.msph_mg_sr.zz = reshape(mgC_sr(:,3), sz);
            else
                % no rotation is needed (identity matrix) ...
                obj.msph_mg_sr.xx = obj.msph_mg_s.xx;
                obj.msph_mg_sr.yy = obj.msph_mg_s.yy;
                obj.msph_mg_sr.zz = obj.msph_mg_s.zz;
            end
            setMeshgridAtPos(obj);
        end

        function setMeshgridAtPos(obj)
            % set the 3D-grid at the given origin point:
            obj.mgrid.X = obj.msph_mg_sr.xx + obj.msph_orig(1,1);
            obj.mgrid.Y = obj.msph_mg_sr.yy + obj.msph_orig(2,1);
            obj.mgrid.Z = obj.msph_mg_sr.zz + obj.msph_orig(3,1);
        end

    end
end
