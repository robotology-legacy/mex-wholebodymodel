classdef wbmSimBody < handle
    properties(Dependent)%, GetAccess = public)
        % public properties for fast get/set methods:
        hull_size_sf@double matrix
        hull_faces@uint8    matrix
        foot_joints@uint8   vector
        foot_ds@double      matrix
    end

    properties(Access = private)
        mHull_size_sf@double matrix = [];
        mHull_faces@uint8    matrix %= uint8([]);
        mFoot_joints@uint8   vector %= uint8([]);
        mFoot_ds@double      matrix = [];
    end

    properties(SetAccess = private, GetAccess = public)
        joint_names@cell 	vector = {};
        joint_pair_idx@cell vector = {};
        nJoints@uint8       scalar = 0;
        nLinks@uint8        scalar = 0;
        nFeets@uint8 		scalar = 0;
    end

    methods
        function obj = wbmSimBody(robot_joint_names, joint_pair_idx)
            if (nargin ~= 2)
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

            for i = 1:2
                [nRows,nCols] = size(joint_pair_idx{i,1});
                if ( (nRows ~= obj.nLinks) || (nCols ~= 6) )
                    error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrMsg.WRONG_MAT_DIM);     
                end
            end
            obj.joint_pair_idx = joint_pair_idx;
        end

        function set.hull_size_sf(obj, hull_size_sf)
            [nRows,nCols] = size(hull_size_sf);
            if ( (nRows ~= obj.nLinks) || (nCols ~= 2) )
                error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrMsg.WRONG_MAT_DIM);
            end
            obj.mHull_size_sf = hull_size_sf;
        end

    	function hull_size_sf = get.hull_size_sf(obj)
    		hull_size_sf = obj.mHull_size_sf;
    	end

        function set.hull_faces(obj, hull_faces)
            [nRows,nCols] = size(hull_faces);
            if ( (nRows ~= 6) || (nCols ~= 4) )
                error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrMsg.WRONG_MAT_DIM);
            end
            obj.mHull_faces = hull_faces;
        end

    	function hull_faces = get.hull_faces(obj)
    		hull_faces = obj.mHull_faces;
    	end

    	function set.foot_joints(obj, foot_joints)
    		if ~isvector(foot_joints)
                error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrMsg.WRONG_DATA_TYPE);
    		end
    		obj.mFoot_joints = foot_joints;
            obj.nFeets = length(obj.mFoot_joints);
    	end

    	function foot_joints = get.foot_joints(obj)
    		foot_joints = obj.mFoot_joints;
    	end

        function set.foot_ds(obj, foot_ds)
            [nRows,nCols] = size(foot_ds);
            if ( (nRows ~= 8) || (nCols ~= 3) )
                error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrMsg.WRONG_MAT_DIM);
            end
            obj.mFoot_ds = foot_ds;
        end

    	function foot_ds = get.foot_ds(obj)
    		foot_ds = obj.mFoot_ds;
    	end

	end
end