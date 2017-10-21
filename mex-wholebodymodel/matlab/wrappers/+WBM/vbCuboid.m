classdef vbCuboid < WBM.vbObject
    properties(Dependent)
        origin@double vector % position of the origin (x,y,z) of the cuboid
        rotm@double   matrix % orientation (rotation matrix) of the cuboid
        tform@double  matrix % transformation matrix of the cuboid
        frame@double  vector % VQ-transformation (pos. & orientation) of the cuboid
    end

    properties
        %                          default values:
        line_width@double scalar = 0.4;
        edge_color               = 'black';
        face_color               = WBM.wbmColor.lightsteelblue;
        face_alpha@double scalar = 0.2; % transparency of the fill area
        description@char         = '';  % annotation for the cuboid
        ismovable@logical scalar = false;
    end

    properties(SetAccess = private, GetAccess = public)
        obj_type@char             = 'bdy';      % type of object: obstacle (obs) or volume body (bdy) (default: bdy)
        isobstacle@logical scalar = false;      % defines if the cuboid is an obstacle (default: false)
        issolid@logical    scalar = false;      % defines if the cuboid is solid (default: false)
        dimension@double   vector = zeros(1,3); % dimension (width, length & height) of the cuboid
        vertices@double    matrix = zeros(8,3); % vertex positions of the cuboid at the given origin and orientation
        com@double         vector = zeros(3,1); % position of the center of mass (CoM)
        rho@double         scalar = 0;          % volumetric mass density of the cuboid
        m_rb@double        scalar = 0;          % mass of the cuboid
        I_cm@double        matrix = zeros(3,3); % inertia of the cuboid at CoM (default: identity matrix)
        mgrid@struct              = struct('X', [], 'Y', [], 'Z', []); % internal 3D-meshgrid of the cuboid (only for obstacles)
    end

    properties(Access = private)
        mcub_orig@double   vector = zeros(3,1); % initial position of the origin (x,y,z)
        mcub_rotm@double   matrix = eye(3,3);   % initial rotation matrix (x,y,z)
        mcub_vtx_s@double  matrix = zeros(8,3); % vertex positions scaled to width, length & height
        mcub_vtx_sr@double matrix = zeros(8,3); % vertex positions scaled and set to the given orientation
        mcub_mg_s@struct          = struct('xx', [], 'yy', [], 'zz', []); % 3D-coordinates arrays of the scaled cuboid
        mcub_mg_sr@struct         = struct('xx', [], 'yy', [], 'zz', []); % 3D-coordinates arrays of the scaled and rotated cuboid
        mcub_mgC_s@double  matrix = [];         % 3D-meshgrid coordinates matrix (x,y,z) of the scaled cuboid
        mcub_msz@double    scalar = 0.01;       % mesh size of the internal 3D-grid (default: 0.01)
    end

    properties(Access = private, Constant)
        %           x:    y:    z:
        mverts = [-0.5  -0.5  -0.5;
                  -0.5   0.5  -0.5;
                   0.5   0.5  -0.5;
                   0.5  -0.5  -0.5;
                  -0.5  -0.5   0.5;
                  -0.5   0.5   0.5;
                   0.5   0.5   0.5;
                   0.5  -0.5   0.5];

        mfaces = uint8([1 2 3 4;
                        1 4 8 5;
                        2 6 5 1;
                        3 7 8 4;
                        5 8 7 6;
                        7 3 2 6]);
    end

    methods
        function obj = vbCuboid(varargin)
            switch nargin
                case 6 % rectangular cuboid:
                    % l_x      = varargin{1} ... length along the x direction (width)
                    % l_y      = varargin{2} ... length along the y direction (length)
                    % l_z      = varargin{3} ... length along the z direction (height)
                    % orig     = varargin{4} ... origin of the cuboid
                    % rotm     = varargin{5} ... orientation of the cuboid
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
                    % l        = varargin{1} ... length of all sides of the cube
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

        function hgo = getGObj(obj)
            hgo = patch('Vertices', obj.vertices, 'LineWidth', obj.line_width, 'EdgeColor', obj.edge_color, ...
                        'Faces', obj.mfaces, 'FaceColor', obj.face_color, 'FaceAlpha', obj.face_alpha);
        end

        function hgo = updGObj(obj, hgo)
            hgo.Vertices = obj.vertices;
            % set(hgo, 'Vertices', obj.vertices);
        end

        function hmg = drawMGrid(obj, pt_color)
            if ~obj.isobstacle
                error('vbCuboid::drawMGrid: %s', WBM.wbmErrorMsg.OBJ_NOT_OBSTACLE);
            end

            if (nargin == 1)
                pt_color = 'green';
            end
            hmg = scatter3(obj.mgrid.X(:), obj.mgrid.Y(:), obj.mgrid.Z(:), ...
                           'Marker', '.', 'MarkerEdgeColor', pt_color);
        end

        function result = ptInObj(obj, pos)
            % half length of each side of the cuboid ...
            lx_h = obj.mcub_vtx_s(7,1);
            ly_h = obj.mcub_vtx_s(7,2);
            lz_h = obj.mcub_vtx_s(7,3);

            if isvector(pos)
                WBM.utilities.chkfun.checkVecLen(pos, 3, 'vbCuboid::ptInObj')
                pos = pos(:); % make sure that pos is a column vector

                % get the distance of the given point to the CoM and rotate this
                % point back to the initial condition of the cuboid (rect. pos.
                % with CoM at origin 0):
                d_r = obj.mcub_rotm.' * (pos - obj.com); % back-rotated distance
                d_x = d_r(1,1);
                d_y = d_r(2,1);
                d_z = d_r(3,1);

                % check if the back-rotated point position is in the cuboid, i.e.
                % the point is below the surface of the cuboid:
                res_x = (d_x > -lx_h) && (d_x < lx_h);
                res_y = (d_y > -ly_h) && (d_y < ly_h);
                res_z = (d_z > -lz_h) && (d_z < lz_h);

                result = (res_x && res_y && res_z);
            elseif ismatrix(pos)
                [m,n] = size(pos);
                if (n ~= 3)
                    error('vbCuboid::ptInObj: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
                end
                % get the distances of all points to the CoM:
                ds = zeros(m,3);
                cm = obj.com.';
                for i = 1:m
                    ds(i,1:3) = pos(i,1:3) - cm;
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

                result = (res_x && res_y && res_z);
            else
                error('vbCuboid::ptInObj: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
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
            obj.mcub_rotm = rotm;
            setCuboidAtPosRotm(obj);
        end

        function tform = get.tform(obj)
            tform = eye(4,4);
            tform(1:3,1:3) = obj.mcub_rotm;
            tform(1:3,4)   = obj.mcub_orig;
        end

        function set.tform(obj, tform)
            [obj.mcub_orig, obj.mcub_rotm] = WBM.utilities.tfms.tform2posRotm(tform);
            setCuboidAtPosRotm(obj);
        end

        function vqT = get.frame(obj)
            vqT = zeros(7,1);
            vqT(1:3,1) = obj.mcub_orig;
            vqT(4:7,1) = WBM.utilities.tfms.rotm2quat(obj.mcub_rotm);
        end

        function set.frame(obj, vqT)
            [obj.mcub_orig, obj.mcub_rotm] = WBM.utilities.tfms.frame2posRotm(vqT);
            setCuboidAtPosRotm(obj);
        end

    end

    methods(Access = private)
        function setObjData(obj, orig, rotm, obj_prop)
            WBM.utilities.chkfun.checkCVecDim(orig, 3, 'vbCuboid::setObjData');
            WBM.utilities.chkfun.checkMatDim(rotm, 3, 3, 'vbCuboid::setObjData');

            obj.mcub_orig = orig;
            obj.mcub_rotm = rotm;

            if (nargin == 4)
                % set the object properties of the cuboid ...
                if ~isstruct(obj_prop)
                    error('vbCuboid::setObjData: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
                end
                obj.line_width  = obj_prop.line_width;
                obj.edge_color  = obj_prop.edge_color;
                obj.face_color  = obj_prop.face_color;
                obj.face_alpha  = obj_prop.face_alpha;
                obj.description = obj_prop.description;
                obj.ismovable   = obj_prop.ismovable;

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
                            [obj.m_rb, obj.I_cm] = WBM.utilities.rb.massbInertiaGObj('scub', obj.rho, l);
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
