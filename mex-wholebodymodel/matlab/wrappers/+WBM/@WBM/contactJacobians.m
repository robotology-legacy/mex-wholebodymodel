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

function [Jc, djcdq] = contactJacobians(obj, varargin)
    % Computes the *contact constraint Jacobian* and the *bias acceleration*,
    % i.e. the *product* of the contact Jacobian derivative with the joint
    % velocities of the floating-base robot.
    %
    % The method can be called in two different modes:
    %
    % **Normal mode** -- Computes the contact Jacobian and the bias acceleration
    % of the given contact links configuration, specified by the base orientation,
    % the positions and the velocities:
    %
    %   .. py:method:: contactJacobians(wf_R_b_arr, wf_p_b, q_j, dq_j, v_b[, idx_list])
    %
    % **Optimized mode** -- Computes the contact Jacobian and the bias acceleration
    % of the given contact links configuration at the current state of the robot
    % system:
    %
    %   .. py:method:: contactJacobians(clnk_conf)
    %
    % Arguments:
    %   wf_R_b_arr (double, vector): :math:`(9 \times 1)` rotation matrix reshaped
    %                                in vector form from the base frame *b* to the
    %                                world frame *wf* (*optional*).
    %   wf_p_b     (double, vector): :math:`(3 \times 1)` position vector from the
    %                                base frame *b* to the world frame *wf* (*optional*).
    %   q_j        (double, vector): :math:`(n_{dof} \times 1)` joint positions vector
    %                                in :math:`[\si{\radian}]` (*optional*).
    %   dq_j       (double, vector): :math:`(n_{dof} \times 1)` joint angle velocity
    %                                vector in :math:`[\si{\radian/s}]` (*optional*).
    %   v_b        (double, vector): :math:`(6 \times 1)` generalized base velocity
    %                                vector (*optional*).
    %   idx_list      (int, vector): :math:`(1 \times k)` vector with the index numbers
    %                                of specific contact constraints in ascending order
    %                                with :math:`1 \leq k \leq n_{cstrs}` (*optional*).
    %
    %                                **Note:** If the index list for the contact
    %                                constraints is undefined, then the method uses
    %                                automatically all defined contact constraints
    %                                of the robot.
    % Returns:
    %   [Jc, djcdq]: 2-element tuple containing:
    %
    %      - **Jc**    (*double, vector*) -- :math:`(6 m \times n)` Jacobian
    %        of the *contact constraints* in contact space
    %        :math:`\mathrm{C = \{C_1,\ldots,C_k\}}`.
    %      - **djcdq** (*double, vector*) -- :math:`(6 m \times 1)` bias
    %        acceleration vector :math:`\dot{J}_c\dot{q}_j` in contact space
    %        :math:`\mathrm{C}`.
    %
    %   The variable :math:`m` denotes the *number of contact constraints*
    %   of the robot and :math:`n = n_{dof} + 6`.
    %
    % Note:
    %   If both contact links that are defined in the given configuration
    %   structure are (currently) not in contact with the ground or an
    %   object, then the arrays of the contact Jacobian :math:`J_c` and
    %   of the product :math:`\dot{J}_c\dot{q}_j` will be set to zero.
    %   Furthermore, the bias acceleration :math:`\dot{J}_c\dot{q}_j` is
    %   an acceleration that is not due to a robot acceleration.
    n = obj.mwbm_model.ndof + 6;

    switch nargin
        case 7 % normal modes:
            % use only specific contact constraints:
            wf_R_b_arr = varargin{1,1};
            wf_p_b     = varargin{1,2};
            q_j        = varargin{1,3};
            dq_j       = varargin{1,4};
            v_b        = varargin{1,5};
            idx_list   = varargin{1,6};

            nCstrs = getNCstrs(idx_list);
        case 6
            % use all contact constraints:
            wf_R_b_arr = varargin{1,1};
            wf_p_b     = varargin{1,2};
            q_j        = varargin{1,3};
            dq_j       = varargin{1,4};
            v_b        = varargin{1,5};

            nCstrs = obj.mwbm_config.nCstrs;
            if (nCstrs == 0)
                % constraint list is empty ...
                Jc    = zeros(6,n);
                djcdq = obj.ZERO_CVEC_6;
                return
            end
            idx_list = 1:nCstrs;
        case 2 % optimized modes:
            % use only specific constraints:
            idx_list = varargin{1,1};
            nCstrs = getNCstrs(idx_list);
        case 1
            % use all constraints:
            nCstrs = obj.mwbm_config.nCstrs;
            if (nCstrs == 0)
                Jc    = zeros(6,n);
                djcdq = obj.ZERO_CVEC_6;
                return
            end
            idx_list = 1:nCstrs;
        otherwise
            error('WBM::contactJacobians: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    m     = 6*nCstrs;
    Jc    = zeros(m,n);
    djcdq = zeros(m,1);

    % compute for each contact constraint the Jacobian and the product
    % of the Jacobian derivative with the joint velocities:
    if (nargin > 2)
        % normal mode:
        for i = 1:nCstrs
            idx = idx_list(1,i);
            ccstr_link = obj.mwbm_config.ccstr_link_names{1,idx};

            Jc(6*i-5:6*i,1:n)  = mexWholeBodyModel('jacobian', wf_R_b_arr, wf_p_b, q_j, ccstr_link); % 6*(i-1)+1 = 6*i-5
            djcdq(6*i-5:6*i,1) = mexWholeBodyModel('dJdq', wf_R_b_arr, wf_p_b, q_j, dq_j, v_b, ccstr_link);
        end
    else
        % optimized mode:
        for i = 1:nCstrs
            idx = idx_list(1,i);
            ccstr_link = obj.mwbm_config.ccstr_link_names{1,idx};

            Jc(6*i-5:6*i,1:n)  = mexWholeBodyModel('jacobian', ccstr_link);
            djcdq(6*i-5:6*i,1) = mexWholeBodyModel('dJdq', ccstr_link);
        end
    end
end
%% END of contactJacobians.


%% NUMBER OF CONSTRAINTS:

function nCstrs = getNCstrs(idx_list)
    if isrow(idx_list) % (including scalar & not empty)
        nCstrs = size(idx_list,2);
        if (nCstrs > 1)
            WBM.utilities.chkfun.checkNumListAscOrder(idx_list, 'WBM::contactJacobians');
        end
    else
        error('WBM::contactJacobians: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
    end
end
