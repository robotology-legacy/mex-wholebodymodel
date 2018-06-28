% Copyright (C) 2015-2018, by Martin Neururer
% Author: Martin Neururer
% E-mail: martin.neururer@student.tuwien.ac.at / martin.neururer@gmail.com
% Date:   January-May, 2018
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
% FP7 EU-project CoDyCo (No. 600716, ICT-2011.2.1 Cognitive Systems and
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

function lnk_traj = setTrajectoriesData(obj, lnk_traj, stmPos, start_idx, end_idx)
    % Sets the positions for the data points of the given link trajectories at
    % each evaluation point of the defined time interval or a specific time
    % period of it.
    %
    % The method calculates the data points for the trajectory curves of some
    % certain links of the robot. The trajectories will be shown in the
    % visualization of the robot simulation.
    %
    % Arguments:
    %   lnk_traj (:class:`~WBM.wbmLinkTrajectory`, vector): Array of trajectory objects to
    %                                                       show the trajectory curves of
    %                                                       specific links of the robot.
    %   stmPos    (double, matrix): Position and orientation data from the *forward
    %                               dynamics states* of the integration output matrix
    %                               :math:`\mathcal{X}` of the given robot model.
    %   start_idx    (int, scalar): Time index (row) of the position and orientation
    %                               data matrix ``stmPos``, where the interval starts.
    %   end_idx      (int, scalar): Time index (row) of the position and orientation
    %                               data matrix ``stmPos``, where the interval ends.
    % Returns:
    %   lnk_traj (:class:`~WBM.wbmLinkTrajectory`, vector): Array of trajectory objects
    %   with the calculated data points of each link trajectory.
    %
    % See Also:
    %   :class:`~WBM.wbmLinkTrajectory` and :meth:`WBM.getPositionsData`.
    if ~isa(lnk_traj, 'WBM.wbmLinkTrajectory')
        error('WBM::setTrajectoriesData: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
    end
    lnk_traj = lnk_traj(:); % make sure that lnk_traj is a column vector ...

    ndof = obj.mwbm_model.ndof;
    vlen = ndof + 7;

    [noi, len] = size(stmPos);
    if (len ~= vlen)
        error('WBM::setTrajectoriesData: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
    end
    nTraj = size(lnk_traj,1);

    % get the positions, orientations and the joint positions from the output vector
    % "stmPos" of the integration part of the forward dynamics function:
    switch nargin
        case 5
            start_idx = start_idx(:);
            end_idx   = end_idx(:);
            WBM.utilities.chkfun.checkCVecDs(start_idx, end_idx, nTraj, nTraj, 'WBM::setTrajectoriesData');
            if ( ~isempty(find(start_idx( (start_idx < 1) | (start_idx > noi) | (start_idx > end_idx) ),1)) || ...
                 ~isempty(find(end_idx( (end_idx < 1) | (end_idx > noi) ),1)) )
                error('WBM::setTrajectoriesData: %s', WBM.wbmErrorMsg.VAL_OUT_OF_BOUNDS);
            end

            % calculate and set the data points of each link trajectory to a
            % specific time section [is, ie] of the given iteration:
            for i = 1:nTraj
                is = start_idx(i,1);
                ie = end_idx(i,1);

                vqT_b = stmPos(is:ie,1:7);
                q_j   = stmPos(is:ie,8:vlen);
                q_j   = q_j.';

                nSteps = ie - is + 1; % = size(vqT_b,1)
                lnk_traj(i,1) = setTrajectoryDPts(obj, lnk_traj(i,1), vqT_b, q_j, nSteps);
            end
        case 3
            vqT_b = stmPos(1:noi,1:7);
            q_j   = stmPos(1:noi,8:vlen);
            q_j   = q_j.';

            % calculate and set the data points of each link trajectory
            % for all iteration steps (noi):
            for i = 1:nTraj
                lnk_traj(i,1) = setTrajectoryDPts(obj, lnk_traj(i,1), vqT_b, q_j, noi);
            end
        otherwise
            error('WBM::setTrajectoriesData: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
end
