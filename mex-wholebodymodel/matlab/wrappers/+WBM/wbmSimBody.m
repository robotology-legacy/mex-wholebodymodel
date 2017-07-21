classdef wbmSimBody < handle
    properties(Dependent)
        % public properties for fast get/set methods:
        shape_geom@struct
        shape_size_sf@double matrix
        shape_faces@uint8    matrix

        foot_geom@struct
        foot_joints@uint8    vector
        foot_base_sz
        foot_shape_ds@double matrix
    end

    properties
        draw_prop@WBM.wbmRobotDrawProp
    end

    properties(SetAccess = private, GetAccess = public)
        joint_lnk_names@cell vector
        joint_pair_idx@uint8 matrix
        nJoints@uint8        scalar = 0;
        nLinks@uint8         scalar = 0;
        nFeets@uint8         scalar = 0;
    end

    properties(Access = private)
        mshape_geom = struct( 'size_sf',  [], ...
                              'faces',    [] );

        mfoot_geom  = struct( 'joints',   [], ...
                              'base_sz',  struct( 'width',  0, ...
                                                  'height', 0 ), ...
                              'shape_ds', [] );
    end

    methods
        function obj = wbmSimBody(joint_lnk_names, joint_pair_idx, draw_prop)
            if ( (nargin < 2) || (nargin > 3) )
                error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            if ( ~iscellstr(joint_lnk_names) || ~ismatrix(joint_pair_idx) )
                error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            if ~iscolumn(joint_lnk_names)
                error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end

            obj.joint_lnk_names = joint_lnk_names;
            obj.nJoints = size(obj.joint_lnk_names,1);
            obj.nLinks  = obj.nJoints - 2; % a tree has n-1 edges (links) + without 'CoM' ...

            [m,n] = size(joint_pair_idx);
            if ( (m ~= obj.nLinks) || (n ~= 6) )
                error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end
            obj.joint_pair_idx = joint_pair_idx;

            % initialize the draw properties for the body of the animated robot ...
            if exist('draw_prop', 'var')
                obj.draw_prop = draw_prop;
                return
            end
            % else, use the default draw values ...
            obj.draw_prop = WBM.wbmRobotDrawProp;
            obj.draw_prop.joints.marker     = '.';
            obj.draw_prop.joints.marker_sz  = 9;
            obj.draw_prop.joints.color      = 'blue';

            obj.draw_prop.links.line_width  = 1.6;
            obj.draw_prop.links.color       = 'black';

            obj.draw_prop.com.marker        = '*';
            obj.draw_prop.com.marker_sz     = 4;
            obj.draw_prop.com.color         = 'red';

            obj.draw_prop.shape.line_width  = 0.4;
            obj.draw_prop.shape.edge_color  = 'black';
            obj.draw_prop.shape.face_color  = 'black';
            obj.draw_prop.shape.face_alpha  = 0.2;
        end

        function set.shape_geom(obj, shape_geom)
            if ~isstruct(shape_geom)
                error('wbmSimBody::set.shape_geom: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            obj.shape_size_sf = shape_geom.size_sf;
            obj.shape_faces   = shape_geom.faces;
        end

        function shape_geom = get.shape_geom(obj)
            shape_geom = obj.mshape_geom;
        end

        function set.shape_size_sf(obj, size_sf)
            [m,n] = size(size_sf);
            if ( (m ~= obj.nLinks) || (n ~= 2) )
                error('wbmSimBody::set.shape_size_sf: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end
            obj.mshape_geom.size_sf = size_sf;
        end

        function shape_size_sf = get.shape_size_sf(obj)
            shape_size_sf = obj.mshape_geom.size_sf;
        end

        function set.shape_faces(obj, shape_faces)
            [m,n] = size(shape_faces);
            if ( (m ~= 6) || (n ~= 4) )
                error('wbmSimBody::set.shape_faces: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end
            obj.mshape_geom.faces = shape_faces;
        end

        function shape_faces = get.shape_faces(obj)
            shape_faces = obj.mshape_geom.faces;
        end

        function set.foot_geom(obj, foot_geom)
            if ~isstruct(foot_geom)
                error('wbmSimBody::set.foot_geom: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            obj.foot_joints   = foot_geom.joints;
            obj.foot_base_sz  = foot_geom.base_sz;
            obj.foot_shape_ds = foot_geom.shape_ds;
        end

        function foot_geom = get.foot_geom(obj)
            foot_geom = obj.mfoot_geom;
        end

        function set.foot_joints(obj, foot_jnts)
            if ~isvector(foot_jnts)
                error('wbmSimBody::set.foot_joints: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            obj.mfoot_geom.joints = foot_jnts;
            obj.nFeets = length(obj.mfoot_geom.joints);
        end

        function foot_jnts = get.foot_joints(obj)
            foot_jnts = obj.mfoot_geom.joints;
        end

        function set.foot_base_sz(obj, base_sz)
            if isstruct(base_sz)
                obj.mfoot_geom.base_sz.width  = base_sz.width;
                obj.mfoot_geom.base_sz.height = base_sz.height;
            elseif ( isvector(base_sz) && (size(base_sz,2) == 2) )
                obj.mfoot_geom.base_sz.width  = base_sz(1,1);
                obj.mfoot_geom.base_sz.height = base_sz(1,2);
            else
                error('wbmSimBody::set.foot_base_sz: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
        end

        function foot_base_sz = get.foot_base_sz(obj)
            foot_base_sz = obj.mfoot_geom.base_sz;
        end

        function set.foot_shape_ds(obj, foot_ds)
            [m,n] = size(foot_ds);
            if ( (m ~= 8) || (n ~= 3) )
                error('wbmSimBody::set.foot_shape_ds: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end
            obj.mfoot_geom.shape_ds = foot_ds;
        end

        function foot_shape_ds = get.foot_shape_ds(obj)
            foot_shape_ds = obj.mfoot_geom.shape_ds;
        end

    end
end
