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

function [t, stmChi] = intForwardDynamics(obj, tspan, stvChi_0, fhTrqControl, ode_opt, varargin)
    % Setups and integrates the *forward dynamics differential equation* of the
    % form :math:`\dot{\chi} = FD(t, \chi)` in dependency of the given interval
    % of integration :math:`[t_0, t_f]` and the initial condition :math:`\chi_0`.
    %
    % The method uses for the specified ODE-function the Matlab ODE solver ``ode15s``
    % for solving stiff differential equations and differential-algebraic systems
    % :cite:`Shampine1997`.
    %
    % Following *pose correction types* for the position-regulation system of
    % the forward dynamics will be supported:
    %
    %   - ``none`` -- No pose corrections.
    %   - ``nfb``  -- No floating base and without pose corrections.
    %   - ``fpc``  -- Foot pose correction.
    %   - ``hpc``  -- Hand pose correction.
    %   - ``fhpc`` -- Foot and hand pose correction.
    %
    % In dependency of the specified *pose correction*, the method can be
    % called as follows:
    %
    %   *none, nfb:*
    %      .. py:method:: intForwardDynamics(tspan, stvChi_0, fhTrqControl, ode_opt[, pc_type])
    %
    %   *none, fpc:*
    %      .. py:method:: intForwardDynamics(tspan, stvChi_0, fhTrqControl, ode_opt, foot_conf, clnk_conf, fe_c, ac, ac_f[, pc_type])
    %
    %   *none, fpc, fhpc:*
    %      .. py:method:: intForwardDynamics(tspan, stvChi_0, fhTrqControl, ode_opt, fhTotCWrench, foot_conf, hand_conf, f_cp, ac_f[, pc_type])
    %
    %   *nfb:*
    %      .. py:method:: intForwardDynamics(tspan, stvChi_0, fhTrqControl, ode_opt, clnk_conf, fe_c, ac[, pc_type])
    %
    %   *nfb:*
    %      .. py:method:: intForwardDynamics(tspan, stvChi_0, fhTrqControl, ode_opt, fhTotCWrench, hand_conf, f_cp[, pc_type])
    %
    %   *fpc:*
    %      .. py:method:: intForwardDynamics(tspan, stvChi_0, fhTrqControl, ode_opt, foot_conf, ac_f[, pc_type])
    %
    %   *hpc:*
    %      .. py:method:: intForwardDynamics(tspan, stvChi_0, fhTrqControl, ode_opt, hand_conf, fe_h, ac_h[, pc_type])
    %
    %   *fhpc:*
    %      .. py:method:: intForwardDynamics(tspan, stvChi_0, fhTrqControl, ode_opt, foot_conf, hand_conf, fe_h, ac_h, ac_f[, pc_type])
    %
    % Arguments:
    %   tspan         (double, vector): :math:(1 \times n) time interval of integration
    %                                   vector with a minimum size of two elements
    %                                   :math:`[t_0, t_f]`, specifying the initial and
    %                                   final times.
    %
    %                                   The variable :math:`n` denotes the *number of
    %                                   evaluation points* of the given interval vector
    %                                   specified by a time step size :math:`t_{step}`,
    %                                   such that :math:`t_f = t_0 + n\cdot t_{step}`.
    %
    %                                   **Note:** All elements of the interval vector
    %                                   ``tspan`` must be *increasing* or *decreasing*.
    %                                   If ``tspan`` has two elements, then the
    %                                   ODE-solver returns a solution at each internal
    %                                   time step within the interval. If ``tspan``
    %                                   has more than two elements, then the solver
    %                                   returns a solution at each given time point.
    %   stvChi_0      (double, vector): :math:`((2 n_{dof} + 13) \times 1)` initial state
    %                                   vector :math:`\chi_0` of the robot model to
    %                                   specify the initial condition for the given
    %                                   forward dynamics ODE-function.
    %   fhTrqControl (function_handle): Function handle to a specified time-dependent
    %                                   *torque control function* that controls the
    %                                   dynamics of the robot system.
    %   ode_opt               (struct): Option structure for fine-tuning, adjusting
    %                                   error tolerances, or passing additional
    %                                   information to the ODE-solver.
    %
    %                                   Use the ``odeset`` function of Matlab to
    %                                   create or modify the options structure.
    % Other Parameters:
    %   fhTotCWrench (function_handle): Function handle to a specific *total contact
    %                                   wrench function* in contact space
    %                                   :math:`\mathrm{C_h = \{C_1,\ldots,C_n\}}`
    %                                   of the hands that will be applied by the
    %                                   robot model (*optional*).
    %   foot_conf     (struct): Configuration structure to specify the *qualitative
    %                           state* of the feet (*optional*).
    %   hand_conf     (struct): Configuration structure to specify the *qualitative
    %                           state* of the hands (*optional*).
    %   clnk_conf     (struct): Configuration structure to specify the *qualitative
    %                           state* of at most two *contact links* (*optional*).
    %   fe_c  (double, vector): :math:`(k \times 1)` vector of external forces (in
    %                           contact space) that are acting on the specified
    %                           contact links (*optional*).
    %   fe_h  (double, vector): :math:`(k \times 1)` vector of external forces (in
    %                           contact space) that are acting on the specified
    %                           *contact points* of the hands (*optional*).
    %   f_cp  (double, vector): Force vector or scalar applied to a grasped object at
    %                           the *contact points* :math:`{}^{\small O}p_{\small C_i}`
    %                           from the contact frames :math:`\mathrm{\{C_1,\ldots,C_n\}}`
    %                           of the hands to the origin frame :math:`\mathrm{O}` at
    %                           the CoM of the object (*optional*).
    %
    %                           The vector length :math:`l` of the applied forces depends
    %                           on the chosen *contact model* and if only one hand or both
    %                           hands are involved in grasping an object, such that
    %                           :math:`l = h\cdot s` with size :math:`s \in \{1,3,4\}` and
    %                           the number of hands :math:`h \in \{1,2\}`.
    %
    %                           **Note:** The z-axis of a contact frame
    %                           :math:`\mathrm{C_{i \in \{1,\ldots,n\}}}` points in the
    %                           direction of the inward surface normal at the point of
    %                           contact :math:`{}^{\small O}p_{\small C_i}`. If the
    %                           chosen contact model is *frictionless*, then each applied
    %                           force to the object is a scalar, otherwise a vector.
    %   ac    (double, vector): :math:`(k \times 1)` mixed acceleration vector
    %                           for the *contact points* of the specified
    %                           contact links (*optional*).
    %   ac_f  (double, vector): :math:`(k \times 1)` mixed acceleration vector
    %                           for the *foot contact points* (*optional*).
    %   ac_h  (double, vector): :math:`(k \times 1)` mixed acceleration vector
    %                           for the *hand contact points* (*optional*).
    %   pc_type (char, vector): Specifies the *pose correction type* for the robot model
    %                           that will be applied by the position-regulation system of
    %                           the given forward dynamics method.
    %
    % The variable :math:`k` indicates the *size* of the given *force* and *acceleration
    % vectors* in dependency of the specified contact links, feet and hands:
    %
    %   - :math:`k = 6`  -- only one link/foot/hand is defined.
    %   - :math:`k = 12` -- both links/feet/hands are defined.
    %
    % The specified *external forces* and *accelerations* are either *constant* or *zero*.
    %
    % Returns:
    %   [t, stmChi]: 2-element tuple containing:
    %
    %      - **t** (*double, vector*) -- :math:`(n \times 1)` evaluation points vector
    %        of the given time interval to perform the integration.
    %      - **stmChi**    (*struct*) -- :math:`(n \times (2 n_{dof} + 13)` integration
    %        output matrix :math:`\mathcal{X}` with the next *forward dynamics states*
    %        (solutions) of the robot model. Each row in ``stmChi`` corresponds to a
    %        solution at the value in the corresponding row of ``t``.
    %
    %   The variable :math:`n` denotes the *number of evaluation points* of the given
    %   time interval.
    %
    % See Also:
    %   :meth:`WBM.getFDynVisData`, :meth:`WBM.getFDynVisDataSect` and
    %   :meth:`WBM.visualizeForwardDynamics`.
    %
    % References:
    %   :cite:`Shampine1997`

    % References:
    %   [LR97] Shampine, L. F.; Reichelt, M. W.: The Matlab ODE Suite.
    %          In: SIAM Journal on Scientific Computing, Volume 18, Issue 1, 1997,
    %          URL: <https://www.mathworks.com/help/pdf_doc/otherdocs/ode_suite.pdf>.
    if (nargin < 5)
        error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    if ~isa(fhTrqControl, 'function_handle')
        error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE)
    end
    if (obj.mwbm_config.nCstrs == 0)
        error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.VALUE_LTE_ZERO);
    end

    % check the last element of the argument list ...
    n = size(varargin,2);
    if ( (n ~= 0) && ischar(varargin{1,n}) )
        pc_type = varargin{1,n};
        narg = nargin;
    else
        % use no pose corrections (default) ...
        pc_type = 'none';
        narg = nargin + 1;
    end

    switch pc_type
        case 'none'
            % no pose corrections:
            switch narg
                case 11
                    % pc_type = varargin{6}
                    if isstruct(varargin{1,1})
                        % simple function with external forces:
                        % fe_c = varargin{3}
                        % ac   = varargin{4}
                        % ac_f = varargin{5} (must be either zero or constant)
                        foot_conf = varargin{1,1};
                        clnk_conf = varargin{1,2};

                        WBM.utilities.chkfun.checkCLinkConfigs(foot_conf, clnk_conf, 'WBM::intForwardDynamics');

                        fhFwdDyn = @(t, chi)forwardDynamicsEF(obj, t, chi, fhTrqControl, foot_conf, ...
                                                              clnk_conf, varargin{3:5});
                    else
                        % simple function with payloads at the hands:
                        % f_cp = varargin{4}
                        % ac_f = varargin{5}
                        fhTotCWrench = varargin{1,1};
                        foot_conf    = varargin{1,2};
                        hand_conf    = varargin{1,3};

                        checkInputTypes(fhTotCWrench, foot_conf, hand_conf);

                        fhFwdDyn = @(t, chi)forwardDynamicsPL(obj, t, chi, fhTrqControl, fhTotCWrench, ...
                                                              foot_conf, hand_conf, varargin{1,4}, varargin{1,5});
                    end
                case 6
                    % simple function without any pose corrections:
                    % nargin = 6: pc_type = varargin{1}
                    fhFwdDyn = @(t, chi)forwardDynamics(obj, t, chi, fhTrqControl);
                otherwise
                    error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
        case 'nfb'
            % no floating base and no pose corrections:
            switch narg
                case 9
                    % pc_type = varargin{4}
                    if isstruct(varargin{1,1})
                        % function with external forces:
                        % fe_c = varargin{2}
                        % ac   = varargin{3}
                        clnk_conf = varargin{1,1};

                        WBM.utilities.chkfun.checkCLinkConfig(clnk_conf, 'WBM::intForwardDynamics');

                        fhFwdDyn = @(t, chi)forwardDynamicsNFBEF(obj, t, chi, fhTrqControl, clnk_conf, ...
                                                                 varargin{1,2}, varargin{1,3});
                    else
                        % function with payloads at the hands:
                        % f_cp = varargin{3}
                        fhTotCWrench = varargin{1,1};
                        hand_conf    = varargin{1,2};

                        if ~isa(fhTotCWrench, 'function_handle')
                            error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
                        end
                        WBM.utilities.chkfun.checkCLinkConfig(hand_conf, 'WBM::intForwardDynamics');

                        fhFwdDyn = @(t, chi)forwardDynamicsNFBPL(obj, t, chi, fhTrqControl, fhTotCWrench, ...
                                                                 hand_conf, varargin{1,3});
                    end
                case 6
                    % function without any pose corrections:
                    % nargin = 6: pc_type = varargin{1}
                    fhFwdDyn = @(t, chi)forwardDynamicsNFB(obj, t, chi, fhTrqControl);
                otherwise
                    error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
        case 'fpc'
            % only foot pose corrections:
            switch narg
                case 11
                    % pc_type = varargin{6}
                    if isstruct(varargin{1,1})
                        % extended function with external forces:
                        % fe_c = varargin{3}
                        % ac   = varargin{4}
                        % ac_f = varargin{5} (must be either zero or constant)
                        foot_conf = varargin{1,1};
                        clnk_conf = varargin{1,2};

                        WBM.utilities.chkfun.checkCLinkConfigs(foot_conf, clnk_conf, 'WBM::intForwardDynamics');

                        fhFwdDyn = @(t, chi)forwardDynamicsFPCEF(obj, t, chi, fhTrqControl, foot_conf, ...
                                                                 clnk_conf, varargin{3:5});
                    else
                        % extended function with payloads at the hands:
                        % f_cp = varargin{4}
                        % ac_f = varargin{5}
                        fhTotCWrench = varargin{1,1};
                        foot_conf    = varargin{1,2};
                        hand_conf    = varargin{1,3};

                        checkInputTypes(fhTotCWrench, foot_conf, hand_conf);

                        fhFwdDyn = @(t, chi)forwardDynamicsFPCPL(obj, t, chi, fhTrqControl, fhTotCWrench, ...
                                                                 foot_conf, hand_conf, varargin{1,4}, varargin{1,5});
                    end
                case 8
                    % extended function with only foot pose corrections:
                    % ac_f    = varargin{2}
                    % pc_type = varargin{3}
                    foot_conf = varargin{1,1};

                    WBM.utilities.chkfun.checkCLinkConfig(foot_conf, 'WBM::intForwardDynamics');

                    fhFwdDyn = @(t, chi)forwardDynamicsFPC(obj, t, chi, fhTrqControl, foot_conf, varargin{1,2});
                otherwise
                    error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
        case 'hpc'
            % only hand pose corrections:
            if (narg ~= 9)
                error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            % fe_h    = varargin{2}
            % ac_h    = varargin{3}
            % pc_type = varargin{4}
            hand_conf = varargin{1,1};

            WBM.utilities.chkfun.checkCLinkConfig(hand_conf, 'WBM::intForwardDynamics');

            fhFwdDyn = @(t, chi)forwardDynamicsHPCEF(obj, t, chi, fhTrqControl, hand_conf, ...
                                                     varargin{1,2}, varargin{1,3});
        case 'fhpc'
            % foot and hand pose corrections:
            if (narg ~= 11)
                error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            % pc_type = varargin{6}
            if isstruct(varargin{1,1})
                % extended function with external forces:
                % fe_h = varargin{3}
                % ac_h = varargin{4}
                % ac_f = varargin{5} (must be either zero or constant)
                foot_conf = varargin{1,1};
                hand_conf = varargin{1,2};

                WBM.utilities.chkfun.checkCLinkConfigs(foot_conf, hand_conf, 'WBM::intForwardDynamics');

                fhFwdDyn = @(t, chi)forwardDynamicsFHPCEF(obj, t, chi, fhTrqControl, foot_conf, ...
                                                          hand_conf, varargin{3:5});
            else
                % extended function with payloads at the hands:
                % f_cp = varargin{4}
                % ac_f = varargin{5}
                fhTotCWrench = varargin{1,1};
                foot_conf    = varargin{1,2};
                hand_conf    = varargin{1,3};

                checkInputTypes(fhTotCWrench, foot_conf, hand_conf);

                fhFwdDyn = @(t, chi)forwardDynamicsFPCPL(obj, t, chi, fhTrqControl, fhTotCWrench, ...
                                                         foot_conf, hand_conf, varargin{1,4}, varargin{1,5});
            end
        otherwise
            error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
    end
    % call the ODE-Solver ...
    [t, stmChi] = ode15s(fhFwdDyn, tspan, stvChi_0, ode_opt);
end
%% END of intForwardDynamics.


%% INPUT VERIFICATION:

function checkInputTypes(fh, clink_conf1, clink_conf2)
    if ~isa(fh, 'function_handle')
        error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
    end
    WBM.utilities.chkfun.checkCLinkConfigs(clink_conf1, clink_conf2, 'WBM::intForwardDynamics');
end
