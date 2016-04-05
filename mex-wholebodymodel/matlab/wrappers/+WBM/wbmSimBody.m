classdef wbmSimBody < handle
    properties(Dependent)
        % public properties for fast get/set methods:
        hull_geometry@struct
        hull_size_sf@double  matrix
        hull_faces@uint8     matrix

        foot_geometry@struct
        foot_joints@uint8    vector
        foot_base_sz
        foot_shape_ds@double matrix
    end

    properties(Access = private)
        % mHull_geom.size_sf@double        matrix
        % mHull_geom.faces@uint8           matrix
        mHull_geom = struct( 'size_sf',  [], ...
                             'faces',    [] );  

        mFoot_geom = struct( 'joints',   [], ...
                             'base_sz',  struct( 'width',  0, ...
                                                 'height', 0 ), ...
                             'shape_ds', [] );
        % mFoot_geom.joints@uint8          vector
        % mFoot_geom.base_sz.width@double  scalar
        % mFoot_geom.base_sz.height@double scalar
        % mFoot_geom.facess@uint8       matrix
        % mFoot_geom.shape_ds@double       matrix
    end

    properties(SetAccess = private, GetAccess = public)
        joint_names@cell 	vector = {};
        joint_pair_idx@cell vector = {};
        nJoints@uint8       scalar = 0;
        nLinks@uint8        scalar = 0;
        nFeets@uint8 		scalar = 0;
    end

    properties
        draw_prop@WBM.wbmDrawProp
	end

    methods
        function obj = wbmSimBody(robot_joint_names, joint_pair_idx, draw_prop)
            if ( (nargin < 2) || (nargin > 3) )
                error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            if ( ~iscell(robot_joint_names) || ~iscell(joint_pair_idx) )
                error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrMsg.WRONG_DATA_TYPE);
            end
            if ~iscolumn(robot_joint_names)
                error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrMsg.WRONG_VEC_DIM);     
            end
            if (size(joint_pair_idx,1) ~= 2)
                error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrMsg.WRONG_ARR_SIZE); 
            end

            obj.joint_names = robot_joint_names;
            obj.nJoints = size(obj.joint_names,1);
            obj.nLinks  = obj.nJoints - 2; % a tree has n-1 edges (links) + without 'CoM' ...

            % check the dimensions of the joint pair matrices ...
            for i = 1:2
                [nRows,nCols] = size(joint_pair_idx{i,1});
                if ( (nRows ~= obj.nLinks) || (nCols ~= 6) )
                    error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrMsg.WRONG_MAT_DIM);     
                end
            end
            obj.joint_pair_idx = joint_pair_idx;

            % initialize the draw properties for the body of the animated robot ...
            obj.draw_prop = WBM.wbmDrawProp;
            if ~exist('draw_prop', 'var')
            	% use default values ...
				obj.draw_prop.joints.marker    = '.';
				obj.draw_prop.joints.marker_sz = 9;
				obj.draw_prop.joints.color     = 'blue';

				obj.draw_prop.links.line_width = 1.6;
				obj.draw_prop.links.color      = 'black';
				
                obj.draw_prop.com.marker       = '*';
                obj.draw_prop.com.marker_sz    = 4;
				obj.draw_prop.com.color        = 'red';

                obj.draw_prop.hull.line_width  = 0.4;
                obj.draw_prop.hull.edge_color  = 'black';
				obj.draw_prop.hull.face_color  = 'black';
                obj.draw_prop.hull.face_alpha  = 0.2;

				obj.draw_prop.ground_color     = [201 220 222] ./ 255; % hex: #c9dcde
                return
            end
            % else ...
            obj.draw_prop.joints.marker    = draw_prop.joint.marker;
            obj.draw_prop.joints.marker_sz = draw_prop.joint.marker_sz;
            obj.draw_prop.joints.color     = draw_prop.joint.color;

            obj.draw_prop.links.line_width = draw_prop.link.line_width;
            obj.draw_prop.links.color      = draw_prop.link.color;

            obj.draw_prop.com.marker       = draw_prop.com.marker;
            obj.draw_prop.com.marker_sz    = draw_prop.com.marker_sz;
            obj.draw_prop.com.color        = draw_prop.com.color;

            obj.draw_prop.hull.line_width  = draw_prop.hull.line_width;
            obj.draw_prop.hull.edge_color  = draw_prop.hull.edge_color;
            obj.draw_prop.hull.face_color  = draw_prop.hull.face_color;
            obj.draw_prop.hull.face_alpha  = draw_prop.hull.face_alpha;

            obj.draw_prop.ground_color     = draw_prop.ground_color;                    
        end

        function set.hull_geometry(obj, hull_geom)
            if ~isstruct(hull_geom)
                error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrMsg.WRONG_DATA_TYPE);                
            end
            obj.hull_size_sf = hull_geom.size_sf;
            obj.hull_faces = hull_geom.faces;
        end

        function hull_geom = get.hull_geometry(obj)
            hull_geom = obj.mHull_geom;
        end

        function set.hull_size_sf(obj, hull_size_sf)
            [nRows,nCols] = size(hull_size_sf);
            if ( (nRows ~= obj.nLinks) || (nCols ~= 2) )
                error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrMsg.WRONG_MAT_DIM);
            end
            obj.mHull_geom.size_sf = hull_size_sf;
        end

    	function hull_size_sf = get.hull_size_sf(obj)
    		hull_size_sf = obj.mHull_geom.size_sf;
    	end

        function set.hull_faces(obj, hull_faces)
            [nRows,nCols] = size(hull_faces);
            if ( (nRows ~= 6) || (nCols ~= 4) )
                error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrMsg.WRONG_MAT_DIM);
            end
            obj.mHull_geom.faces = hull_faces;
        end

    	function hull_faces = get.hull_faces(obj)
    		hull_faces = obj.mHull_geom.faces;
    	end

        function set.foot_geometry(obj, foot_geom)
            if ~isstruct(foot_geom)
                error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrMsg.WRONG_DATA_TYPE);                
            end
            obj.foot_joints   = foot_geom.joints;
            obj.foot_base_sz  = foot_geom.base_sz;
            obj.foot_shape_ds = foot_geom.shape_ds;
        end

        function foot_geom = get.foot_geometry(obj)
            foot_geom = obj.mFoot_geom;
        end

    	function set.foot_joints(obj, foot_joints)
    		if ~isvector(foot_joints)
                error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrMsg.WRONG_DATA_TYPE);
    		end
    		obj.mFoot_geom.joints = foot_joints;
            obj.nFeets = length(obj.mFoot_geom.joints);
    	end

    	function foot_joints = get.foot_joints(obj)
    		foot_joints = obj.mFoot_geom.joints;
    	end

        function set.foot_base_sz(obj, base_sz)
            if isstruct(base_sz)
                obj.mFoot_geom.base_sz.width  = base_sz.width;
                obj.mFoot_geom.base_sz.height = base_sz.height;
            elseif ( isvector(base_sz) && (size(base_sz,2) == 2) )
                obj.mFoot_geom.base_sz.width  = base_sz(1,1);
                obj.mFoot_geom.base_sz.height = base_sz(1,2);
            else
                error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrMsg.WRONG_DATA_TYPE);
            end
        end

        function foot_base_sz = get.foot_base_sz(obj)
            foot_base_sz = obj.mFoot_geom.base_sz;
        end

        function set.foot_shape_ds(obj, foot_ds)
            [nRows,nCols] = size(foot_ds);
            if ( (nRows ~= 8) || (nCols ~= 3) )
                error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrMsg.WRONG_MAT_DIM);
            end
            obj.mFoot_geom.shape_ds = foot_ds;
        end

    	function foot_shape_ds = get.foot_shape_ds(obj)
    		foot_shape_ds = obj.mFoot_geom.shape_ds;
    	end

	end
end