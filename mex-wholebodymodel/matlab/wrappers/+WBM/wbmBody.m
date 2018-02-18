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

classdef wbmBody < handle
    % :class:`!wbmBody` is a *data type* (class) to define and configure the
    % body components of a given floating base robot.
    %
    % Attributes:
    %   chains (cell, matrix): (*nChns* x 3) list of *chain names* (effectors) with
    %                          their corresponding *joint index pair numbers*,
    %                          specified in the form
    %                          :math:`\{chain\_name, idx_{start}, idx_{end}\}`.
    %                          The joint index pairs are defining the start and
    %                          end positions of each chain in the joint name array.
    %
    %                          **Note:** A "chain" represents a subset of joints
    %                          of a robot that are linked together. The joint
    %                          names of the robot must be named as defined in the
    %                          `iCub Model Naming Conventions`_ and must be set in
    %                          the same order as specified in the configuration
    %                          file `yarpWholeBodyInterface.ini`_.
    %   nChns (uint8, scalar): Number of chains that are defining the full body
    %                          of the robot (default 0, if the robot is undefined).
    %   joints (cell, matrix): (*nJnts* x 2) list of *joint names* with the
    %                          corresponding *joint index number*, specified in
    %                          the form :math:`\{jnt\_name, jnt_idx \}`.
    %
    %                          **Note:** The order of the joint names must be set
    %                          as defined in the list ``ROBOT_MEX_WBI_TOOLBOX`` of
    %                          the configuration file `yarpWholeBodyInterface.ini`_.
    %   nJnts (uint8, scalar): Number of joints of the given robot (default 0, if
    %                          the robot is undefined).
    properties(SetAccess = private, GetAccess = public)
       chains@cell matrix
       nChns@uint8 scalar = 0;
       joints@cell matrix
       nJnts@uint8 scalar = 0;
    end

    methods
        function obj = wbmBody(chn_names, chn_idx, jnt_names, jnt_idx)
            % Constructor.
            %
            % Args:
            %   chn_names (cellstr, vector): Column-array with the *chain names* of
            %                                each chain element of the robot model.
            %   chn_idx   (integer, matrix): (*nChns* x 2) matrix of the form
            %                                :math:`[idx_{start}, idx_{end}]`, with
            %                                start and end positions of each chain
            %                                of the robot.
            %   jnt_names (cellstr, vector): Column-array with the *joint names*
            %                                of the robot model as specified in
            %                                the `iCub Model Naming Conventions`_.
            %   jnt_idx   (integer, vector): (*nJnts* x 1) vector of ascending
            %                                index numbers for the joints of the
            %                                robot.
            % Returns:
            %   obj: An instance of the body components object of the given robot.
            if (nargin ~= 4)
                error('wbmBody::wbmBody: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end

            %% Chains:
            % verify the input types ...
            if ( ~iscellstr(chn_names) || ~ismatrix(chn_idx) )
                error('wbmBody::wbmBody: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            % check the dimensions ...
            [nRows, nCols] = size(chn_idx);
            if (nCols ~= 2)
                error('wbmBody::wbmBody: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end

            if (size(chn_names,1) ~= nRows)
                error('wbmBody::wbmBody: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
            end
            % store the chain data into the cell array ...
            obj.nChns  = nRows;
            obj.chains = horzcat(chn_names, num2cell(chn_idx));

            %% Joints:
            % check input types ...
            if ( ~iscellstr(jnt_names) || ~isvector(jnt_idx) )
                error('wbmBody::wbmBody: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            % check dimensions ...
            nJnts = size(jnt_names,1);
            if (nJnts ~= size(jnt_idx,1))
                error('wbmBody::wbmBody: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
            end
            % store the joint data ...
            obj.nJnts  = nJnts;
            obj.joints = horzcat(jnt_names, num2cell(jnt_idx));
        end

        function jnt_idx = getChainIndices(obj, chn_name)
            % Gets the index positions of all joint members of a given chain.
            %
            % Args:
            %   chn_name (char, vector): String matching name of the chain
            %                            element of the robot.
            % Returns:
            %   jnt_idx (integer, vector): Row vector with joint index positions.
            pos = find(strcmp(obj.chains(1:obj.nChns,1), chn_name));
            if isempty(pos)
                error('wbmBody::getChainIndices: %s', WBM.wbmErrorMsg.NAME_NOT_EXIST);
            end

            start_idx = obj.chains{pos,2};
            end_idx   = obj.chains{pos,3};
            jnt_idx   = start_idx:end_idx;
        end

        function jnt_idx = getJointIndex(obj, jnt_name)
            % Gets the index position of a specific joint.
            %
            % Args:
            %   jnt_name (char, vector): String matching name of a specific joint
            %                            of the robot.
            % Returns:
            %   jnt_idx (integer): Index position of the given joint.
            jnt_idx = find(strcmp(obj.joints(1:obj.nJnts,1), jnt_name));
        end

        function jnt_names = getJointNames(obj, jnt_idx)
            % Gets the joint names of a given list of index positions.
            %
            % Args:
            %   chn_idx (integer, vector): Row-vector with the index positions
            %                              of specific joints in ascending or
            %                              descending order.
            % Returns:
            %   jnt_names (cellstr, vector): Column-vector with the joint names
            %                                that are corresponding to the index
            %                                list.

            % check ranges ...
            if isscalar(jnt_idx)
                if ( (jnt_idx > obj.nJnts) || (jnt_idx < 1) )
                    error('wbmBody::getJointNames: %s', WBM.wbmErrorMsg.IDX_OUT_OF_BOUNDS);
                end

                jnt_names = obj.joints{jnt_idx, 1};
                return
            end
            % else ...
            if ~isrow(jnt_idx)
                error('wbmBody::getJointNames: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end
            % we assume that the elements are in ascending or descending order ...
            n = size(jnt_idx,2);
            if ( (jnt_idx(1,1) > obj.nJnts) || (jnt_idx(1,1) < 1) || ...
                 (jnt_idx(1,n) > obj.nJnts) || (jnt_idx(1,n) < 1) )
                error('wbmBody::getJointNames: %s', WBM.wbmErrorMsg.IDX_OUT_OF_BOUNDS);
            end

            jnt_names = obj.joints(jnt_idx,1);
        end

        function chn_tbl = getChainTable(obj)
            % Gets the table of all chain elements of the robot body.
            %
            % Args:
            %   obj: The body components object of the given robot.
            %
            % Returns:
            %   chn_tbl (table): Table object with the variables *'chain_name'*,
            %                    *'start_idx'* and *'end_idx'* as column names.
            chn_tbl = cell2table(obj.chains, 'VariableNames', {'chain_name', 'start_idx', 'end_idx'});
        end

        function jnt_tbl = getJointTable(obj)
            % Gets the table of all joints of the robot body.
            %
            % Args:
            %   obj: The body components object of the given robot.
            %
            % Returns:
            %   jnt_tbl (table): Table object with the variables *'joint_name'*
            %                    and *'idx'* as column names.
            jnt_tbl = cell2table(obj.joints, 'VariableNames', {'joint_name', 'idx'});
        end

    end
end
