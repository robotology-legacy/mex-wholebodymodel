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
    properties(Dependent)
        origin@double vector % position of the origin (x,y,z) of the cylinder
        rotm@double   matrix % orientation (rotation matrix) of the cylinder
        tform@double  matrix % transformation matrix of the cylinder
        frame@double  vector % VQ-transformation (pos. & orientation) of the cylinder
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
        obj_type@char             = 'bdy';      % type of object: obstacle (obs) or volume body (bdy) (default: bdy)
        isobstacle@logical scalar = false;      % defines if the cylinder is an obstacle (default: false)
        issolid@logical    scalar = false;      % defines if the cylinder is solid (default: false)
        istool@logical     scalar = false;      % defines if the cylinder is a tool (default: false)
        init_frame@double  vector = WBM.vbCylinder.DF_FRAME; % initial frame (pos. & orientation) of the cylinder
        dimension@double   vector = zeros(1,3); % dimension (inner/outer radius & height) of the cylinder
        vertices@struct           = struct('X', [], 'Y', [], 'Z', []); % vertex positions of the cylinder at the given origin and orientation
        com@double         vector = zeros(3,1); % position of the center of mass (CoM)
        rho@double         scalar = 0;          % volumetric mass density of the cylinder
        m_rb@double        scalar = 0;          % mass of the cylinder
        I_cm@double        matrix = zeros(3,3); % inertia of the cylinder at CoM (default: identity matrix)
        mgrid@struct              = struct('X', [], 'Y', [], 'Z', []); % internal 3D-meshgrid of the cuboid (only for obstacles)
    end

    properties(Access = private)
        mcyl_orig@double   vector = zeros(3,1); % initial position of the origin (x,y,z)
        mcyl_rotm@double   matrix = eye(3,3);   % initial rotation matrix (x,y,z)
        mcyl_vtx_s@struct         = struct('xx', [], 'yy', [], 'zz', []); % vertex positions scaled to the cylinder height
        mcyl_vtx_sr@struct        = struct('xx', [], 'yy', [], 'zz', []); % vertex positions scaled and set to the given orientation
        mcyl_vtxC_s@double matrix = [];         % vertex coordinates matrix (x,y,z) of the scaled cylinder
        mcyl_nfcs@int16    scalar = 20;         % number of faces around the circumference of the (linearized) cylinder (default: 20)
        mcyl_mg_s@struct          = struct('xx', [], 'yy', [], 'zz', []); % 3D-coordinates arrays of the scaled cylinder
        mcyl_mg_sr@struct         = struct('xx', [], 'yy', [], 'zz', []); % 3D-coordinates arrays of the scaled and rotated cylinder
        mcyl_mgC_s@double  matrix = [];         % 3D-meshgrid coordinates matrix (x,y,z) of the scaled cylinder
        mcyl_msz@double    scalar = 0.01;       % mesh size of the internal 3D-grid (default: 0.01)
    end

    methods
        function obj = vbCylinder(varargin)
            switch nargin
                case 6 % circular cylindrical tube:
                    % ri       = varargin{1} ... inner radius of the cylindrical tube
                    % ro       = varargin{2} ... outer radius of the cylindrical tube
                    % h        = varargin{3} ... height of the cylinder along the z direction
                    % orig     = varargin{4} ... origin of the cylinder
                    % rotm     = varargin{5} ... orientation of the cylinder
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
                        % r        = varargin{1} ... radius of the solid cylinder
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
            % set the cylinder to the given initial pos. and orientation:
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
            hgo = surf(obj.vertices.X, obj.vertices.Y, obj.vertices.Z, 'LineWidth', obj.line_width, ...
                       'EdgeColor', obj.edge_color, 'FaceColor', obj.face_color, 'FaceAlpha', obj.face_alpha);
        end

        function hgo = updGObj(obj, hgo)
            hgo.XData = obj.vertices.X;
            hgo.YData = obj.vertices.Y;
            hgo.ZData = obj.vertices.Z;
        end

        function hmg = drawMGrid(obj, pt_color)
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

                result = (res1 && res2);
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

                result = (res1 && res2);
            else
                error('vbCylinder::ptInObj: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
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
