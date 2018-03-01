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

classdef wbmSimBody < handle
    % :class:`!wbmSimBody` is a *data type* (class) that defines the *body model*,
    % i.e. the geometric shape, of the simulated robot.
    %
    % Attributes:
    %   shape_geom             (struct): Data structure to define the shape and
    %                                    the size of all *body parts* (patches)
    %                                    for the links and the feet of the robot's
    %                                    skeleton.
    %
    %                                    **Note:** The structure has two data fields,
    %                                    ``size_sf`` and ``faces``. The field ``size_sf``
    %                                    represents a (*nLnks* x 2) data matrix with
    %                                    *constant scale factors* of the form
    %                                    :math:`[sf_{width}, sf_{height}]`, to define
    %                                    the shape sizes for the body parts of all
    %                                    links of the robot. The row index of the
    %                                    matrix is equal to the joint index number of
    %                                    the robot. The second field ``faces`` denotes
    %                                    a (6 x 4) *vertex connection matrix* to define
    %                                    which vertices are to connect for the polygons
    %                                    of the patches. The patches are defining the
    %                                    body parts of the robot.
    %   shape_size_sf  (double, matrix): (*nLnks* x 2) matrix to set only the *scale
    %                                    factors* :math:`[sf_{width}, sf_{height}]`
    %                                    for the shape sizes of the body parts of
    %                                    the robot (see :attr:`shape_geom`).
    %   shape_faces     (uint8, matrix): (6 x 4) vertex connection matrix to set
    %                                    only the *vertex connections* for the
    %                                    polygons of the patches that are forming
    %                                    the body parts for the robot (see
    %                                    :attr:`shape_geom`).
    %   foot_geom              (struct): Data structure to define the *geometric
    %                                    form* of each foot of the robot.
    %
    %                                    **Note:** The data structure is a nested
    %                                    structure and contains following fields:
    %
    %                                       - ``joints`` (uint8, vector): (2 x 1) vector to specify the
    %                                                                     joint indices where the left
    %                                                                     and right foot is connected.
    %                                       - ``base_sz`` (struct): Structure with the fields ``width``
    %                                                               and ``height``, to specify the *base
    %                                                               size values* (double) for the feet
    %                                                               of the robot. Optional, instead of
    %                                                               the structure, a row-vector of the
    %                                                               form :math:`[width, height]` can
    %                                                               also be used to set the size values.
    %                                       - ``shape_ds`` (double, matrix): (n x 3) matrix with *size values* of
    %                                                                        the form :math:`[length, width, height]`
    %                                                                        to define the *foot dimensions* in the
    %                                                                        x, y and z directions.
    %
    %   foot_joints     (uint8, vector): (2 x 1) vector to set the joint indices
    %                                    for the left and the right foot (see
    %                                    :attr:`foot_geom`).
    %   foot_base_sz    (struct/vector): Data structure, or optional a vector, to
    %                                    set the *base size values*, i.e. the
    %                                    ``width`` and the ``height``, for the
    %                                    feet of the robot (see :attr:`foot_geom`).
    %   foot_shape_ds  (double, matrix): (n x 3) matrix with size values of the
    %                                    form :math:`[length, width, height]`
    %                                    to set the *dimensions* for the feet
    %                                    (see :attr:`foot_geom`).
    %
    %   draw_prop (:class:`~WBM.wbmRobotDrawProp`): Data object to control the draw properties
    %                                               for the body of the simulated robot.
    %
    %   jnt_lnk_names  (cell, vector): Column-array of *link* and *frame names* that are
    %                                  related to specific parent joints or parent links
    %                                  of the robot model [#f1]_.
    %   jnt_pair_idx  (uint8, matrix): (:math:`n` x 6) matrix of *joint pair indices* for the
    %                                  x, y and z-positions to describe the configuration, i.e.
    %                                  the *connectivity graph*, of the robot's skeleton in
    %                                  3D-space. In general :math:`n = nJnts`, but if the given
    %                                  robot also has a *root link* and/or a *com link* that
    %                                  have no parent joint, then either :math:`n = nJnts - 1`
    %                                  or :math:`n = nJnts - 2`.
    %
    %                                  **Note:** Each joint pair is connected with a rigid
    %                                  link to form a kinematic chain of the robot. The
    %                                  joint index pairs in the matrix denoting the index
    %                                  positions of the given joint names in
    %                                  :attr:`jnt_lnk_names`. The index pairs are specified
    %                                  in the matrix as row-vectors of the form
    %                                  :math:`[x_1, x_2, y_1, y_2, z_1, z_2]`.
    %                                  The matrix will be used to create a set of position
    %                                  parameters that describes the skeleton configuration
    %                                  of the robot.
    %   nJnts         (uint8, scalar): Number of joints of the robot model [#f2]_.
    %   nLnks         (uint8, scalar): Number of links of the robot model [#f2]_.
    %   nFeet         (uint8, scalar): Number of feet of the robot model [#f2]_.
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
        % internal structure to store the shapes and
        % sizes of all body parts of the robot:
        mshape_geom = struct( 'size_sf',  [], ...
                              'faces',    [] );

        % internal structure to store the geometric data
        % for the feet of the robot:
        mfoot_geom  = struct( 'joints',   [], ...
                              'base_sz',  struct( 'width',  0, ...
                                                  'height', 0 ), ...
                              'shape_ds', [] );
    end

    methods
        function obj = wbmSimBody(jnt_lnk_names, jnt_pair_idx, draw_prop)
            % Constructor.
            %
            % Arguments:
            %   jnt_lnk_names (cellstr, vector): Column-array of *link* and *frame names*
            %                                    that are deduced from their parent joints
            %                                    or parent links of the robot model [#f1]_
            %                                    (see :attr:`jnt_lnk_names`).
            %   jnt_pair_idx    (uint8, matrix): (n x 6) matrix of *joint pair indices* of
            %                                    the form :math:`[x_1, x_2, y_1, y_2, z_1, z_2]`,
            %                                    to describe the configuration, i.e. the
            %                                    *connectivity graph*, of the robot's
            %                                    skeleton in 3D-space (see :attr:`jnt_pair_idx`).
            %   draw_prop (:class:`~WBM.wbmRobotDrawProp`): The draw properties for the body of
            %                                               the simulated robot.
            % Returns:
            %   obj: An instance of the :class:`!wbmSimBody` data type.
            if (nargin < 2)
                error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            if ( ~iscellstr(jnt_lnk_names) || ~ismatrix(jnt_pair_idx) )
                error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            if ~iscolumn(jnt_lnk_names)
                error('wbmSimBody::wbmSimBody: %s', WBM.wbmErrorMsg.WRONG_ARR_DIM);
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
