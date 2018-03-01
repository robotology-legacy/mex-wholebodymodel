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

function tau_j = inverseDynamics(obj, varargin)
    % wf_R_b = varargin{1}
    % wf_p_b = varargin{2}
    % q_j    = varargin{3}
    % dq_j   = varargin{4}
    % v_b    = varargin{5}
    % ddq_j  = varargin{6}
    % dv_b   = varargin{7}
    switch nargin
        case 8 % normal modes:
            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            tau    = mexWholeBodyModel('inverse-dynamics', wf_R_b_arr, varargin{1,2}, varargin{1,3}, ...
                                       varargin{1,4}, varargin{1,5}, varargin{1,6},  varargin{1,7});
            tau_fr = frictionForces(obj, varargin{1,4}); % friction torques (negative values)
            tau_fr = vertcat(zeros(6,1), tau_fr);

            tau_j = tau + tau_fr;
            return
        case 7
            % if dv_b, the acceleration (linear & angular acceleration)
            % of the robot base is not given or unknown ...
            wf_p_b = varargin{1,2};
            q_j    = varargin{1,3};
            dq_j   = varargin{1,4};
            ddq_j  = varargin{1,6};

            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            M    = mexWholeBodyModel('mass-matrix', wf_R_b_arr, wf_p_b, q_j);
            c_qv = mexWholeBodyModel('generalized-forces', wf_R_b_arr, wf_p_b, q_j, dq_j, varargin{1,5});
        case 4 % optimized modes:
            % dq_j  = varargin{1}
            % ddq_j = varargin{2}
            % dv_b  = varargin{3}

            % Note: The same vector dq_j is already stored inside of the mex-subroutine. Because
            %       before any function can be used in optimized mode, the function "setState"
            %       must be called previously to update the state parameters q_j, dq_j and v_b.
            tau    = mexWholeBodyModel('inverse-dynamics', varargin{1,2}, varargin{1,3});
            tau_fr = frictionForces(obj, varargin{1,1});
            tau_fr = vertcat(zeros(6,1), tau_fr);

            tau_j = tau + tau_fr;
            return
        case 3
            % if dv_b is not given or unknown ...
            dq_j  = varargin{1,1};
            ddq_j = varargin{1,2};

            M    = mexWholeBodyModel('mass-matrix');
            c_qv = mexWholeBodyModel('generalized-forces');
    otherwise
        error('WBMBase::inverseDynamics: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end

    tau_fr = frictionForces(obj, dq_j);
    tau_fr = vertcat(zeros(6,1), tau_fr);

    %% Generalized floating base acceleration for a hybrid-dynamic system:
    %
    %  In general the equation of motion is given as follows:
    %
    %       tau_j = M * ddq_j + C(q_j, dq_j) + tau_fr,
    %
    %  where C(q_j, dq_j) denotes the generalized bias force.
    %  In contrast to the fixed-based system, the equation of motion for a
    %  floating base system is given as follows:
    %
    %       |   0   |   | M_00      M_01 |   | dv_b  |   | h_0 |   |   0    |
    %       |       | = |                | * |       | + |     | + |        |
    %       | tau_j |   | M_01^T    M_11 |   | ddq_j |   | h_1 |   | tau_fr |
    %
    %  where M is the (n+6)x(n+6) mass matrix, h = (h_0, h_1)^T is a (n+6)x1 vector
    %  representing the generalized bias forces (Coriolis, centrifugal and gravity forces)
    %  and dv_b = -(M_00)^(-1) * (M_01 * ddq_j + h_0) denotes the base acceleration.
    %
    % Further details about the formulas are available at:
    %   [1] Rigid Body Dynamics Algorithms, Roy Featherstone, Springer, 2008,
    %       chapter 9.3-9.5, pp. 180-184, eq. (9.13) & (9.24).
    %   [2] Informatics in Control, Automation and Robotics, J. A. Cetto & J. Ferrier & J. Filipe,
    %       Lecture Notes in Electrical Engineering, Volume 89, Springer, 2011, p. 14, eq. (36) & (37).
    %   [3] Dynamics of Tree-Type Robotic Systems, S. V. Shah & S. K. Saha & J. K. Dutt,
    %       Intelligent Systems, Control and Automation: Science and Engineering, Volume 62, Springer, 2012,
    %       p. 119, eq. (7.1) & (7.2).
    dv_b  = generalizedBaseAcc(obj, M, c_qv, ddq_j);
    ddq_j = vertcat(dv_b, ddq_j); % mixed generalized acceleration

    tau_j = M*ddq_j + c_qv + tau_fr;
end
