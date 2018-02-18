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
    properties(Dependent)
        origin@double vector % position of the origin (x,y,z) of the sphere
        rotm@double   matrix % orientation (rotation matrix) of the sphere
        tform@double  matrix % transformation matrix of the sphere
        frame@double  vector % VQ-transformation (pos. & orientation) of the sphere
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
        obj_type@char             = 'bdy';      % type of object: obstacle (obs) or volume body (bdy) (default: bdy)
        isobstacle@logical scalar = false;      % defines if the sphere is an obstacle (default: false)
        issolid@logical    scalar = false;      % defines if the sphere is solid (default: false)
        istool@logical     scalar = false;      % defines if the sphere is a tool (default: false)
        init_frame@double  vector = WBM.vbSphere.DF_FRAME; % initial frame (pos. & orientation) of the sphere
        dimension@double   vector = zeros(1,2); % dimension (inner/outer radius) of the sphere
        vertices@struct           = struct('X', [], 'Y', [], 'Z', []); % vertex positions of the sphere at the given origin and orientation
        com@double         vector = zeros(3,1); % position of the center of mass (CoM)
        rho@double         scalar = 0;          % volumetric mass density of the sphere
        m_rb@double        scalar = 0;          % mass of the sphere
        I_cm@double        matrix = zeros(3,3); % inertia of the sphere at CoM (default: identity matrix)
        mgrid@struct              = struct('X', [], 'Y', [], 'Z', []); % internal 3D-meshgrid of the sphere (only for obstacles)
    end

    properties(Access = private)
        msph_orig@double   vector = zeros(3,1); % initial position of the origin (x,y,z)
        msph_rotm@double   matrix = eye(3,3);   % initial rotation matrix (x,y,z)
        msph_vtx_s@struct         = struct('xx', [], 'yy', [], 'zz', []); % vertex positions scaled to the sphere radius
        msph_vtx_sr@struct        = struct('xx', [], 'yy', [], 'zz', []); % vertex positions scaled and set to the given orientation
        msph_vtxC_s@double matrix = [];         % vertex coordinates matrix (x,y,z) of the scaled sphere
        msph_nfcs@double   scalar = 20;         % number of horizontal and vertical faces to generate a n-by-n sphere (default: 20)
        msph_mg_s@struct          = struct('xx', [], 'yy', [], 'zz', []); % 3D-coordinates arrays of the scaled sphere
        msph_mg_sr@struct         = struct('xx', [], 'yy', [], 'zz', []); % 3D-coordinates arrays of the scaled and rotated sphere
        msph_mgC_s@double  matrix = [];         % 3D-meshgrid coordinates matrix (x,y,z) of the scaled sphere
        msph_msz@double    scalar = 0.01;       % mesh size of the internal 3D-grid (default: 0.01)
    end

    methods
        function obj = vbSphere(varargin)
            switch nargin
                case 5 % spherical shell:
                    % ri       = varargin{1} ... inner radius of the spherical shell
                    % ro       = varargin{2} ... outer radius of the spherical shell
                    % orig     = varargin{3} ... origin of the sphere
                    % rotm     = varargin{4} ... orientation of the sphere
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
                        % r        = varargin{1} ... radius of the solid sphere
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
            % set the sphere to the given initial pos. and orientation:
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
