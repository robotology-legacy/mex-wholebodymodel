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
        jnt_lnk_names@cell vector
        jnt_pair_idx@uint8 matrix
        nJnts@uint8        scalar = 0;
        nLnks@uint8        scalar = 0;
        nFeet@uint8        scalar = 0;
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
        function obj = wbmSimBody(jnt_lnk_names, jnt_pair_idx, draw_prop)
            if (nargin < 2)
                error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            if ( ~iscellstr(jnt_lnk_names) || ~ismatrix(jnt_pair_idx) )
                error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            if ~iscolumn(jnt_lnk_names)
                error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end

            obj.jnt_lnk_names = jnt_lnk_names;
            obj.nJnts = size(jnt_lnk_names,1);
            obj.nLnks = obj.nJnts - 2; % a tree has n-1 edges (links) + without 'CoM' ...

            WBM.utilities.chkfun.checkMatDim(jnt_pair_idx, obj.nLnks, 6, 'wbmSimBody::wbmSimBody');
            obj.jnt_pair_idx = jnt_pair_idx;

            % initialize the draw properties for the body of the animated robot ...
            if (nargin == 3)
                obj.draw_prop = draw_prop;
                return
            end
            % else, use the default draw values ...
            obj.draw_prop = WBM.wbmRobotDrawProp;
            obj.draw_prop.joints.line_width = 0.5;
            obj.draw_prop.joints.marker     = '.';
            obj.draw_prop.joints.marker_sz  = 10;
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
            WBM.utilities.chkfun.checkMatDim(size_sf, obj.nLnks, 2, 'wbmSimBody::set.shape_size_sf');
            obj.mshape_geom.size_sf = size_sf;
        end

        function shape_size_sf = get.shape_size_sf(obj)
            shape_size_sf = obj.mshape_geom.size_sf;
        end

        function set.shape_faces(obj, shape_faces)
            WBM.utilities.chkfun.checkMatDim(shape_faces, 6, 4, 'wbmSimBody::set.shape_faces');
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
            obj.nFeet = uint8(length(obj.mfoot_geom.joints));
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
            WBM.utilities.chkfun.checkMatDim(foot_ds, 8, 3, 'wbmSimBody::set.foot_shape_ds');
            obj.mfoot_geom.shape_ds = foot_ds;
        end

        function foot_shape_ds = get.foot_shape_ds(obj)
            foot_shape_ds = obj.mfoot_geom.shape_ds;
        end

    end
end
