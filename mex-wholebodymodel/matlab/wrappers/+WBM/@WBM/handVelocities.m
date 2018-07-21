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

function [vc_h, v_prms] = handVelocities(obj, hand_conf, varargin)
    % Calculates the *mixed hand velocities* (end-effector velocities) :math:`v_{c_h}`
    % of a given humanoid robot in contact (operational) space :math:`\mathrm{C}_h`.
    %
    % The formula to calculate the *mixed velocities* :math:`c_{c_h}` at the
    % specified *contact links* of the hands (end-effectors) can be expressed
    % as follows:
    %
    %   .. math::
    %      :label: mixed_hand_velocities
    %
    %      v_{c_h} = \dot{x}_h = J_{c_h}\dot{q}_j\:,
    %
    % where the matrix :math:`J_{c_h}` denotes the contact Jacobian of the hands.
    %
    % The method can be called in two different modes:
    %
    % **Normal mode** -- Computes the hand accelerations, specified by
    % the base orientation, the positions and the velocities:
    %
    %   .. py:method:: handVelocities(hand_conf, wf_R_b, wf_p_b, q_j, dq_j, v_b)
    %
    % **Optimized mode** -- Computes the hand accelerations at the current
    % state of the robot system, in dependency of the given contact Jacobian
    % and the joint velocities:
    %
    %   .. py:method:: handVelocities(hand_conf[, Jc_h], dq_j)
    %
    % Arguments:
    %   hand_conf         (struct): Configuration structure to specify the
    %                               *qualitative state* of the hands.
    %
    %                               The data structure specifies which hand is
    %                               currently in contact with the ground, or an
    %                               object, or a wall.
    %
    %                               **Note:** In dependency of the situation, the
    %                               configuration structure can also specify the
    %                               *desired poses*, *angular velocities* and
    %                               *control gains* for the position-regulation
    %                               system of the hands.
    %   wf_R_b    (double, matrix): :math:`(3 \times 3)` rotation matrix (orientation)
    %                               from the base frame *b* to world frame *wf*
    %                               (*optional*).
    %   wf_p_b    (double, vector): :math:`(3 \times 1)` position vector from the base
    %                               frame *b* to the world frame *wf* (*optional*).
    %   q_j       (double, vector): :math:`(n_{dof} \times 1)` joint positions
    %                               vector in :math:`[\si{\radian}]` (*optional*).
    %   dq_j      (double, vector): :math:`(n_{dof} \times 1)` joint angle velocity
    %                               vector in :math:`[\si{\radian/s}]`.
    %   v_b       (double, vector): :math:`(6 \times 1)` generalized base velocity
    %                               vector (*optional*).
    % Other Parameters:
    %   Jc_h    (double, matrix): :math:`(6 m \times n)` Jacobian of the
    %                             *hand contact constraints* in contact space
    %                             :math:`\mathrm{C}_h` with either :math:`m = 1`
    %                             or :math:`m = 2` and :math:`n = n_{dof} + 6`
    %                             (*optional*).
    % Returns:
    %   [vc_h[, v_prms]]: 2-element tuple containing:
    %
    %      - **vc_h** (*double, vector*) -- :math:`(6 m \times 1)` mixed velocity
    %        vector of the hands in :math:`[\si{\radian/{s^2}}]` with either
    %        :math:`m = 1` or :math:`m = 2`.
    %      - **v_prms**       (*struct*) -- Data structure of the calculated
    %        hand velocities parameters with the fields ``Jc_h`` and ``djcdq_h``.
    %
    %        **Note:** The second output argument can be used for further calculations
    %        or for data logging.
    %
    % See Also:
    %   :meth:`WBM.jointAccelerations` and :meth:`WBM.handVelocities`.
    %
    % References:
    %   :cite:`Lilly1992`, p. 82, eq. (5.7).

    % References:
    %   [Lil92] Lilly, Kathryn: Efficient Dynamic Simulation of Robotic Mechanisms.
    %           Springer, 1992, p. 82, eq. (5.7).

    % check the contact state (CS) of the hands ...
    hand_idx = getContactIdx(obj, hand_conf);
    if ~hand_idx
        % no contacts:
        vc_h = obj.ZERO_CVEC_12;
        if (nargout == 2)
            v_prms = struct(); % empty structure ...
        end
        return
    end

    switch nargin
        case 7
            % normal mode:
            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            % v_b    = varargin{5}
            dq_j = varargin{1,4};

            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            [Jc_h, djcdq_h] = contactJacobians(obj, wf_R_b_arr, varargin{1,2}, varargin{1,3}, ...
                                               dq_j, varargin{1,5}, hand_idx);
        case 4 % optimized modes:
            Jc_h = varargin{1,1};
            dq_j = varargin{1,2};
        case 3
            dq_j = varargin{1,1};
            [Jc_h, djcdq_h] = contactJacobians(obj, hand_idx);
        otherwise
            error('WBM::handVelocities: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    % calculate the mixed velocity of the hand(s) at the contact link(s):
    vc_h = Jc_h * dq_j;

    if (nargout == 2)
        if (nargin == 4)
            error('WBM::handVelocities: %s', WBM.wbmErrorMsg.WRONG_NARGOUT);
        end
        % data structure of the calculated velocity parameters ...
        v_prms = struct('Jc_h', Jc_h, 'djcdq_h', djcdq_h);
    end
end
