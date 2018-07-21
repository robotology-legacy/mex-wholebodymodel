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

function vis_data = getFDynVisData(obj, stmChi, fhTrqControl, varargin)
    % Returns the *visualization data parameters* from from the forward dynamics
    % of the given robot model.
    %
    % The method returns a *dynamic data structure* with parameter fields of the
    % computed *forward dynamics states* :math:`\mathcal{X}`, in dependency of
    % the given forward dynamics method and the torque controller. The calculated
    % parameter values of the controller and the forward dynamics can be used for
    % the visualization of certain parameters in a plot or for further analysis
    % purposes.
    %
    % Note:
    %   To obtain the correct parameter values of the robot's forward dynamics,
    %   the *torque controller* and the *forward dynamics method* must be the
    %   same, as used in the computation of the integration output matrix
    %   :math:`\mathcal{X}`.
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
    %      .. py:method:: getFDynVisData(stmChi, fhTrqControl[, pc_type])
    %
    %   *none, fpc:*
    %      .. py:method:: getFDynVisData(stmChi, fhTrqControl, foot_conf, clnk_conf, fe_c, ac, ac_f[, pc_type])
    %
    %   *none, fpc, fhpc:*
    %      .. py:method:: getFDynVisData(stmChi, fhTrqControl, fhTotCWrench, foot_conf, hand_conf, f_cp, ac_f[, pc_type])
    %
    %   *nfb:*
    %      .. py:method:: getFDynVisData(stmChi, fhTrqControl, clnk_conf, fe_c, ac[, pc_type])
    %
    %   *nfb:*
    %      .. py:method:: getFDynVisData(stmChi, fhTrqControl, fhTotCWrench, hand_conf, f_cp[, pc_type])
    %
    %   *fpc:*
    %      .. py:method:: getFDynVisData(stmChi, fhTrqControl, foot_conf, ac_f[, pc_type])
    %
    %   *hpc:*
    %      .. py:method:: getFDynVisData(stmChi, fhTrqControl, hand_conf, fe_h, ac_h[, pc_type])
    %
    %   *fhpc:*
    %      .. py:method:: getFDynVisData(stmChi, fhTrqControl, foot_conf, hand_conf, fe_h, ac_h, ac_f[, pc_type])
    %
    % Arguments:
    %   stmChi        (double, matrix): Integration output matrix :math:`\mathcal{X}`
    %                                   with the calculated *forward dynamics states*
    %                                   (row-vectors) of the given robot model.
    %   fhTrqControl (function_handle): Function handle to a specified time-dependent
    %                                   *torque control function* that controls the
    %                                   dynamics of the robot system.
    %   varargin        (cell, vector): Variable-length input argument list to pass extra
    %                                   parameters for the specified forward dynamics
    %                                   function to be integrated by the ODE-solver
    %                                   (*optional*).
    %
    %                                   The *extra parameters* to be passed are defined
    %                                   in the respective forward dynamics function
    %                                   (see :meth:`WBM.intForwardDynamics` -- *Other
    %                                   Parameters*).
    %   pc_type         (char, vector): String and last element of ``varargin`` to
    %                                   indicate the *pose correction type* to be used
    %                                   for the forward dynamics ODE-function, specified
    %                                   by one of the given string values: ``'none'``,
    %                                   ``'nfb'``, ``'fpc'``, ``'hpc'``, ``'fhpc'``.
    %
    %                                   The default string is ``'none'`` (*optional*).
    % Returns:
    %   vis_data (struct): Dynamic structure with parameter data of the torque
    %   controller and the forward dynamics of the given robot model for
    %   visualization and analysis purposes.
    %
    % See Also:
    %   :meth:`WBM.getFDynVisDataSect`, :meth:`WBM.intForwardDynamics` and
    %   :meth:`WBM.visualizeForwardDynamics`.
    [noi, len] = size(stmChi);
    if (len ~= obj.mwbm_config.stvLen)
        error('WBM::getFDynVisData: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
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

    % add all data to the fields of the generated data container:
    switch pc_type
        case 'none'
            % no pose corrections:
            switch narg
                case 9
                    % pc_type = varargin{6}
                    if isstruct(varargin{1,1})
                        % simple forward dynamics with external forces:
                        % foot_conf = varargin{1}
                        % clnk_conf = varargin{2}
                        % fe_c      = varargin{3} ... external forces that are acting on the contact links
                        % ac        = varargin{4} ... mixed accelerations of the contact points
                        % ac_f      = varargin{5} (must be either zero or constant)
                        vis_data = getVisDataEF(obj, stmChi, fhTrqControl, varargin{1:5}, noi);
                    else
                        % simple forward dynamics with payloads at the hands:
                        % fhTotCWrench = varargin{1}
                        % foot_conf    = varargin{2}
                        % hand_conf    = varargin{3}
                        % f_cp         = varargin{4} ... applied forces at the contact points pc_i
                        % ac_f         = varargin{5} ... mixed accelerations of the foot contact points
                        vis_data = getVisDataPL(obj, stmChi, fhTrqControl, varargin{1:5}, noi);
                    end
                case 4
                    % simple forward dynamics without any pose corrections:
                    % pc_type = varargin{1}
                    vis_data = getVisData(obj, stmChi, fhTrqControl, noi);
                otherwise
                    error('WBM::getFDynVisData: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
        case 'nfb'
            % no floating base (without pose corrections):
            switch narg
                case 7
                    % pc_type = varargin{4}
                    if isstruct(varargin{1,1})
                        % forward dynamics with external forces:
                        % clnk_conf = varargin{1}
                        % fe_c      = varargin{2}
                        % ac        = varargin{3}
                        vis_data = getVisDataNFBEF(obj, stmChi, fhTrqControl, varargin{1:3}, noi);
                    else
                        % forward dynamics with payloads at the hands:
                        % fhTotCWrench = varargin{1}
                        % hand_conf    = varargin{2}
                        % f_cp         = varargin{3}
                        vis_data = getVisDataNFBPL(obj, stmChi, fhTrqControl, varargin{1:3}, noi);
                    end
                case 4
                    % forward dynamics without any pose corrections:
                    % pc_type = varargin{1}
                    vis_data = getVisDataNFB(obj, stmChi, fhTrqControl, noi);
                otherwise
                    error('WBM::getFDynVisData: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
        case 'fpc'
            % only foot pose corrections:
            switch narg
                case 9
                    % pc_type = varargin{6}
                    if isstruct(varargin{1,1})
                        % extended forward dynamics with foot pose corrections
                        % and external forces at the given contact links:
                        % foot_conf = varargin{1}
                        % clnk_conf = varargin{2}
                        % fe_c      = varargin{3}
                        % ac        = varargin{4}
                        % ac_f      = varargin{5} (must be either zero or constant)
                        vis_data = getVisDataFPCEF(obj, stmChi, fhTrqControl, varargin{1:5}, noi);
                    else
                        % extended forward dynamics with foot pose corrections
                        % and payloads at the hands:
                        % fhTotCWrench = varargin{1}
                        % foot_conf    = varargin{2}
                        % hand_conf    = varargin{3}
                        % f_cp         = varargin{4}
                        % ac_f         = varargin{5}
                        vis_data = getVisDataFPCPL(obj, stmChi, fhTrqControl, varargin{1:5}, noi);
                    end
                case 6
                    % extended forward dynamics with only foot pose corrections:
                    % pc_type = varargin{3}
                    % ---------------------
                    % ac_f    = varargin{2}
                    foot_conf = varargin{1,1};

                    fe_0 = zeroExtForces(obj, foot_conf);
                    vis_data = getVisDataCLPCEF(obj, stmChi, fhTrqControl, foot_conf, fe_0, varargin{1,2}, noi);
                otherwise
                    error('WBM::getFDynVisData: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
        case 'hpc'
            % only hand pose corrections:
            if (narg ~= 7)
                error('WBM::getFDynVisData: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            % extended forward dynamics with hand pose corrections
            % and external forces at the hands:
            % pc_type   = varargin{4}
            % -----------------------
            % hand_conf = varargin{1}
            % fe_h      = varargin{2} ... external forces at the hands
            % ac_h      = varargin{3} ... mixed accelerations of the hand contact points
            vis_data = getVisDataCLPCEF(obj, stmChi, fhTrqControl, varargin{1:3}, noi);
        case 'fhpc'
            % foot and hand pose corrections:
            if (narg ~= 9)
                error('WBM::getFDynVisData: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            % pc_type = varargin{6}
            if isstruct(varargin{1,1})
                % extended forward dynamics with foot and hand pose corrections
                % and external forces at the hands:
                % foot_conf = varargin{1}
                % hand_conf = varargin{2}
                % fe_h      = varargin{3}
                % ac_h      = varargin{4}
                % ac_f      = varargin{5} (must be either zero or constant)
                vis_data = getVisDataFHPCEF(obj, stmChi, fhTrqControl, varargin{1:5}, noi);
            else
                % extended function with payloads at the hands:
                % fhTotCWrench = varargin{1}
                % foot_conf    = varargin{2}
                % hand_conf    = varargin{3}
                % f_cp         = varargin{4}
                % ac_f         = varargin{5}
                vis_data = getVisDataFHPCPL(obj, stmChi, fhTrqControl, varargin{1:5}, noi);
            end
        otherwise
            error('WBM::getFDynVisData: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
    end
end
%% END of getFDynVisData.


%% INITIALIZATION, GET & SET VIS-DATA FUNCTIONS:

function [v_1, vlen, is_const] = getFirstVector(vlist, noi)
    % check if the given vector list has only a single vector or
    % a time list of row vectors (matrix):
    is_const = false;
    if iscolumn(vlist)
        % vector does not change in time ...
        v_1  = vlist;
        vlen = size(vlist,1);
        is_const = true;
    elseif ismatrix(vlist)
        % get the first vector ...
        [m, n] = size(vlist);
        if (m ~= noi)
            error('getFirstVector: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
        end
        v_1  = vlist(1,1:n).';
        vlen = n;
    else
        error('getFirstVector: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
    end
end

function fd_prms = getFDynParams(obj, t, stvChi, fhTrqControl)
    % get the current state parameters ...
    stp = WBM.utilities.ffun.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);
    v_b = vertcat(stp.dx_b, stp.omega_b); % generalized base velocity

    % update the state for the optimized mode ...
    setState(obj, stp.q_j, stp.dq_j, v_b);

    % get the current torque forces and the corresponding
    % visualization data structure from the controller:
    [tau, ctrl_prms] = fhTrqControl(t);
    % get the fdyn-parameters from the joint acceleration computation:
    [~,fd_prms] = jointAccelerations(obj, tau, stp.dq_j); % optimized mode

    % add the controller data to the data structure ...
    fd_prms.ctrl_prms = ctrl_prms;
end

function fd_prms = getFDynParamsEF(obj, t, stvChi, fhTrqControl, foot_conf, clnk_conf, fe_c, ac)
    % get the current state parameters ...
    stp = WBM.utilities.ffun.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);
    v_b = vertcat(stp.dx_b, stp.omega_b); % generalized base velocity

    % update the state for the optimized mode ...
    setState(obj, stp.q_j, stp.dq_j, v_b);

    % get the current torque forces and the corresponding
    % visualization data structure from the controller:
    [tau, ctrl_prms] = fhTrqControl(t);
    % get the fdyn-parameters from the joint acceleration computation:
    [~,fd_prms] = jointAccelerationsEF(obj, foot_conf, clnk_conf, tau, fe_c, ac, stp.dq_j); % optimized mode

    % add the controller data to the data structure ...
    fd_prms.ctrl_prms = ctrl_prms;
end

function fd_prms = getFDynParamsPL(obj, t, stvChi, fhTrqControl, fhTotCWrench, foot_conf, hand_conf, f_cp, ac_f)
    % get the current state parameters ...
    stp = WBM.utilities.ffun.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);
    v_b = vertcat(stp.dx_b, stp.omega_b); % generalized base velocity

    % update the state for the optimized mode ...
    setState(obj, stp.q_j, stp.dq_j, v_b);

    % get the current torque forces and the corresponding
    % visualization data structure from the controller:
    [tau, ctrl_prms] = fhTrqControl(t);
    % get the fdyn-parameters from the joint acceleration computation (optimized mode):
    [~,fd_prms] = jointAccelerationsPL(obj, foot_conf, hand_conf, tau, fhTotCWrench, f_cp, ac_f, stp.dq_j);

    % add the controller data to the data structure ...
    fd_prms.ctrl_prms = ctrl_prms;
end

function fd_prms = getFDynParamsNFB(obj, t, stvChi, fhTrqControl)
    % get the current state parameters ...
    stp = WBM.utilities.ffun.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);
    v_b = vertcat(stp.dx_b, stp.omega_b); % generalized base velocity

    % update the state for the optimized mode ...
    setState(obj, stp.q_j, stp.dq_j, v_b);

    [M, c_qv] = wholeBodyDyn(obj); % optimized mode

    % get the current torque forces and the corresponding
    % visualization data structure from the controller:
    [tau, ctrl_prms] = fhTrqControl(t, M, c_qv, stp);
    % get the fdyn-parameters from the joint acceleration computation:
    [~,fd_prms] = jointAccelerationsNFB(obj, tau, M, c_qv, stp.dq_j); % optimized mode

    % add the controller data to the data structure ...
    fd_prms.ctrl_prms = ctrl_prms;
end

function fd_prms = getFDynParamsNFBEF(obj, t, stvChi, fhTrqControl, clnk_conf, fe_c, ac)
    % get the current state parameters ...
    stp = WBM.utilities.ffun.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);
    v_b = vertcat(stp.dx_b, stp.omega_b); % generalized base velocity

    % update the state for the optimized mode ...
    setState(obj, stp.q_j, stp.dq_j, v_b);

    [M, c_qv] = wholeBodyDyn(obj); % optimized mode

    % get the current torque forces and the corresponding
    % visualization data structure from the controller:
    [tau, ctrl_prms] = fhTrqControl(t, M, c_qv, stp);
    % get the fdyn-parameters from the joint acceleration computation (optimized mode):
    [~,fd_prms] = jointAccelerationsNFBEF(obj, clnk_conf, tau, fe_c, ac, M, c_qv, stp.dq_j);

    % add the controller data to the data structure ...
    fd_prms.ctrl_prms = ctrl_prms;
end

function fd_prms = getFDynParamsNFBPL(obj, t, stvChi, fhTrqControl, fhTotCWrench, hand_conf, f_cp)
    % get the current state parameters ...
    stp = WBM.utilities.ffun.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);
    v_b = vertcat(stp.dx_b, stp.omega_b); % generalized base velocity

    % update the state for the optimized mode ...
    setState(obj, stp.q_j, stp.dq_j, v_b);

    [M, c_qv] = wholeBodyDyn(obj); % optimized mode

    % get the current torque forces and the corresponding
    % visualization data structure from the controller:
    [tau, ctrl_prms] = fhTrqControl(t, M, c_qv, stp);
    % get the fdyn-parameters from the joint acceleration computation (optimized mode):
    [~,fd_prms] = jointAccelerationsPL(obj, foot_conf, hand_conf, tau, fhTotCWrench, f_cp, ...
                                       ac_f, M, c_qv, stp.dq_j);
    % add the controller data to the data structure ...
    fd_prms.ctrl_prms = ctrl_prms;
end

function fd_prms = getFDynParamsCLPCEF(obj, t, stvChi, fhTrqControl, clnk_conf, fe_c, ac)
    % get the state parameters ...
    stp  = WBM.utilities.ffun.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);
    v_b  = vertcat(stp.dx_b, stp.omega_b); % generalized base velocity
    nu_s = vertcat(v_b, stp.dq_j);         % mixed generalized velocity of the current state

    % update state ...
    setState(obj, stp.q_j, stp.dq_j, v_b);

    [M, c_qv, Jc_lnk, djcdq_lnk] = wholeBodyDynamicsCS(obj, clnk_conf); % optimized mode

    % get the torque forces and the controller parameters:
    [tau, ctrl_prms] = fhTrqControl(t, M, c_qv, stp, nu_s, Jc_lnk, djcdq_lnk, clnk_conf);
    % get the fdyn-parameters from the acceleration computation (optimized mode):
    [~,fd_prms] = jointAccelerationsCLPCEF(obj, clnk_conf, tau, fe_c, ac, Jc_lnk, djcdq_lnk, ...
                                           M, c_qv, stp.dq_j, nu_s);
    % add the controller data to the fdyn-data ...
    fd_prms.ctrl_prms = ctrl_prms;
end

function fd_prms = getFDynParamsFPCEF(obj, t, stvChi, fhTrqControl, foot_conf, clnk_conf, fe_c, ac, ac_f)
    % get the state parameters ...
    stp  = WBM.utilities.ffun.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);
    v_b  = vertcat(stp.dx_b, stp.omega_b); % generalized base velocity
    nu_s = vertcat(v_b, stp.dq_j);         % mixed generalized velocity of the current state

    % update state ...
    setState(obj, stp.q_j, stp.dq_j, v_b);

    [M, c_qv, Jc_f, djcdq_f] = wholeBodyDynamicsCS(obj, foot_conf); % optimized mode

    % get the torque forces and the controller parameters:
    [tau, ctrl_prms] = fhTrqControl(t, M, c_qv, stp, nu_s, Jc_f, djcdq_f, foot_conf);
    % get the fdyn-parameters of the joint accelerations (optimized mode):
    [~,fd_prms] = jointAccelerationsFPCEF(obj, foot_conf, clnk_conf, tau, fe_c, ac, ac_f, ...
                                          Jc_f, djcdq_f, M, c_qv, stp.dq_j, nu_s);
    % add the controller data to the fdyn-data ...
    fd_prms.ctrl_prms = ctrl_prms;
end

function fd_prms = getFDynParamsFPCPL(obj, t, stvChi, fhTrqControl, fhTotCWrench, foot_conf, hand_conf, f_cp, ac_f)
    % get the state parameters ...
    stp  = WBM.utilities.ffun.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);
    v_b  = vertcat(stp.dx_b, stp.omega_b); % generalized base velocity
    nu_s = vertcat(v_b, stp.dq_j);         % mixed generalized velocity of the current state

    % update state ...
    setState(obj, stp.q_j, stp.dq_j, v_b);

    [M, c_qv, Jc_f, djcdq_f] = wholeBodyDynamicsCS(obj, foot_conf); % optimized mode

    % get the torque forces and the controller parameters:
    [tau, ctrl_prms] = fhTrqControl(t, M, c_qv, stp, nu_s, Jc_f, djcdq_f, foot_conf);
    % get the fdyn-parameters of the joint accelerations (optimized mode):
    [~,fd_prms] = jointAccelerationsFPCPL(obj, foot_conf, hand_conf, tau, fhTotCWrench, f_cp, ...
                                          ac_f, Jc_f, djcdq_f, M, c_qv, stp.dq_j, nu_s);
    % add the controller data to the fdyn-data ...
    fd_prms.ctrl_prms = ctrl_prms;
end

function fd_prms = getFDynParamsFHPCEF(obj, t, stvChi, fhTrqControl, foot_conf, hand_conf, fe_h, ac_h, ac_f)
    % get the state parameters ...
    stp  = WBM.utilities.ffun.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);
    v_b  = vertcat(stp.dx_b, stp.omega_b); % generalized base velocity
    nu_s = vertcat(v_b, stp.dq_j);         % mixed generalized velocity of the current state

    % update state ...
    setState(obj, stp.q_j, stp.dq_j, v_b);

    [M, c_qv, Jc_f, djcdq_f] = wholeBodyDynamicsCS(obj, foot_conf); % optimized mode

    % get the torque forces and the controller parameters:
    [tau, ctrl_prms] = fhTrqControl(t, M, c_qv, stp, nu_s, Jc_f, djcdq_f, foot_conf);
    % get the fdyn-parameters of the joint accelerations (optimized mode):
    [~,fd_prms] = jointAccelerationsFHPCEF(obj, foot_conf, hand_conf, tau, fe_h, ac_h, ...
                                           ac_f, Jc_f, djcdq_f, M, c_qv, stp.dq_j, nu_s);
    % add the controller data to the fdyn-data ...
    fd_prms.ctrl_prms = ctrl_prms;
end

function fd_prms = getFDynParamsFHPCPL(obj, t, stvChi, fhTrqControl, fhTotCWrench, foot_conf, hand_conf, f_cp, ac_f)
    % get the state parameters ...
    stp  = WBM.utilities.ffun.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);
    v_b  = vertcat(stp.dx_b, stp.omega_b); % generalized base velocity
    nu_s = vertcat(v_b, stp.dq_j);         % mixed generalized velocity of the current state

    % update state ...
    setState(obj, stp.q_j, stp.dq_j, v_b);

    [M, c_qv, Jc_f, djcdq_f] = wholeBodyDynamicsCS(obj, foot_conf); % optimized mode

    % get the torque forces and the controller parameters:
    [tau, ctrl_prms] = fhTrqControl(t, M, c_qv, stp, nu_s, Jc_f, djcdq_f, foot_conf);
    % get the fdyn-parameters of the joint accelerations (optimized mode):
    [~,fd_prms] = jointAccelerationsFHPCPL(obj, foot_conf, hand_conf, tau, fhTotCWrench, f_cp, ...
                                           ac_f, Jc_f, djcdq_f, M, c_qv, stp.dq_j, nu_s);
    % add the controller data to the fdyn-data ...
    fd_prms.ctrl_prms = ctrl_prms;
end

function [vis_data, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl] = initVisDataStruct(fd_prms, noi)
    % Basic fields:
    fnames = fieldnames(fd_prms); % get the field names ...
    nflds  = size(fnames,1);
    if isempty(find(strcmp(fnames, 'ctrl_prms'), 1))
        error('initVisDataStruct: %s', WBM.wbmErrorMsg.NAME_NOT_EXIST);
    end
    % initialize all data fields for the (dynamic) visualization data structure:
    vis_data = struct();
    vis_data = initVisDataFields(vis_data, fd_prms, fnames, nflds, noi);

    % remove "ctrl_prms" from the list (is always the last element):
    nflds_base  = nflds - 1;
    fnames_base = fnames(1:nflds_base,1);

    % Controller fields:
    % get the field names of the controller data structure ...
    fnames_ctrl = fieldnames(fd_prms.ctrl_prms);
    nflds_ctrl  = size(fnames_ctrl,1);
    if (nflds_ctrl == 0)
        % the data structure is empty ...
        return
    end
    % add all data fields of the controller structure to the
    % dynamic visualization data field and initialize them:
    vis_data.ctrl_prms = initVisDataFields(vis_data.ctrl_prms, fd_prms.ctrl_prms, ...
                                           fnames_ctrl, nflds_ctrl, noi);
end

function data_flds = initVisDataFields(data_flds, param_flds, fnames, nflds, noi)
    for i = 1:nflds
        fname  = fnames{i,1};
        fld    = param_flds.(fname);
        [m, n] = size(fld); % field dimension

        if isstruct(fld)
            data_flds.(fname) = struct();
        elseif isscalar(fld)
            data_flds.(fname) = zeros(noi,1);
        elseif iscolumn(fld)
            data_flds.(fname) = zeros(m,noi);
        elseif isrow(fld)
            data_flds.(fname) = zeros(noi,n);
        elseif ismatrix(fld)
            data_flds.(fname) = cell(noi,1);
        else
            error('initVisDataFields: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
        end
    end
end

function vis_data = setVisData(idx, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl)
    % add all base data to the corresponding base fields (except to the ctrl_prms field):
    vis_data = addData2Fields(idx, vis_data, fd_prms, fnames_base, nflds_base);

    % add all controller data to the controller sub-field of the visualization data:
    vis_data.ctrl_prms = addData2Fields(idx, vis_data.ctrl_prms, fd_prms.ctrl_prms, ...
                                        fnames_ctrl, nflds_ctrl);
end

function data_flds = addData2Fields(idx, data_flds, param_flds, fnames, nflds)
    % add the data to the specified data fields:
    for i = 1:nflds
        fname  = fnames{i,1};
        fld    = param_flds.(fname);
        [m, n] = size(fld);

        if isstruct(fld)
            data_flds.(fname) = fld;
        elseif isscalar(fld)
            data_flds.(fname)(idx,1) = fld;
        elseif iscolumn(fld)
            data_flds.(fname)(1:m,idx) = fld;
        elseif isrow(fld)
            data_flds.(fname)(idx,1:n) = fld;
        elseif ismatrix(fld)
            data_flds.(fname){idx,1} = fld;
        else
            error('addData2Fields: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
        end
    end
end

function vis_data = getVisData(obj, stmChi, fhTrqControl, noi)
    len = obj.mwbm_config.stvLen;

    % create and initialize the visualization data container:
    stvChi  = stmChi(1,1:len).'; % get first data element ...
    fd_prms = getFDynParams(obj, 1, stvChi, fhTrqControl);
    [vis_data, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl] = initVisDataStruct(fd_prms, noi);

    % put all data into the data container:
    for i = 1:noi
        stvChi   = stmChi(i,1:len).';
        fd_prms  = getFDynParams(obj, i, stvChi, fhTrqControl);
        vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
    end
end

function vis_data = getVisDataEF(obj, stmChi, fhTrqControl, foot_conf, clnk_conf, fe_c, ac, noi)
    len = obj.mwbm_config.stvLen;

    % read the first data-structure s.t. the complete dynamic structure can be
    % generated for the visualization data:
    [fe_1, fe_len, fe_const] = getFirstVector(fe_c, noi);
    [ac_1, ac_len, ac_const] = getFirstVector(ac, noi);
    stvChi  = stmChi(1,1:len).';
    fd_prms = getFDynParamsEF(obj, 1, stvChi, fhTrqControl, foot_conf, clnk_conf, fe_1, ac_1);
    % create and initialize the fields for the visualization data container ...
    [vis_data, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl] = initVisDataStruct(fd_prms, noi);

    % put all data into the data container:
    if (fe_const && ac_const)
        % ext. forces and foot contact accelerations are constant:
        for i = 1:noi
            stvChi   = stmChi(i,1:len).';
            fd_prms  = getFDynParamsEF(obj, i, stvChi, fhTrqControl, foot_conf, clnk_conf, fe_1, ac_1);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    elseif fe_const
        % only the ext. forces are constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            a_c    = ac(i,1:ac_len).';

            fd_prms  = getFDynParamsEF(obj, i, stvChi, fhTrqControl, foot_conf, clnk_conf, fe_1, a_c);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    elseif ac_const
        % only the contact accelerations are constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            fe     = fe_c(i,1:fe_len).';

            fd_prms  = getFDynParamsEF(obj, i, stvChi, fhTrqControl, foot_conf, clnk_conf, fe, ac_1);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    else
        % both vectors are row vector lists (matrices):
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            fe     = fe_c(i,1:feh_len).';
            a_c    = ac(i,1:acf_len).';

            fd_prms  = getFDynParamsEF(obj, i, stvChi, fhTrqControl, foot_conf, clnk_conf, fe, a_c);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    end
end

function vis_data = getVisDataPL(obj, stmChi, fhTrqControl, fhTotCWrench, foot_conf, hand_conf, f_cp, ac_f, noi)
    len = obj.mwbm_config.stvLen;

    % initialization:
    [fcp_1, fcp_len, fcp_const] = getFirstVector(f_cp, noi);
    [acf_1, acf_len, acf_const] = getFirstVector(ac_f, noi);

    stvChi  = stmChi(1,1:len).';
    fd_prms = getFDynParamsPL(obj, 1, stvChi, fhTrqControl, fhTotCWrench, foot_conf, hand_conf, fcp_1, acf_1);
    [vis_data, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl] = initVisDataStruct(fd_prms, noi);

    % put all data into the data container:
    if (fcp_const && acf_const)
        % payload forces and foot contact accelerations are constant:
        for i = 1:noi
            stvChi   = stmChi(i,1:len).';
            fd_prms  = getFDynParamsPL(obj, i, stvChi, fhTrqControl, fhTotCWrench, foot_conf, hand_conf, fcp_1, acf_1);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    elseif fcp_const
        % only the payload forces are constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            acf    = ac_f(i,1:acf_len).';

            fd_prms  = getFDynParamsPL(obj, i, stvChi, fhTrqControl, fhTotCWrench, foot_conf, hand_conf, fcp_1, acf);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    elseif acf_const
        % only the contact accelerations are constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            fcp    = f_cp(i,1:fcp_len).';

            fd_prms  = getFDynParamsPL(obj, i, stvChi, fhTrqControl, fhTotCWrench, foot_conf, hand_conf, fcp, acf_1);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    else
        % both vectors are row vector lists (matrices):
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            fcp    = f_cp(i,1:fcp_len).';
            acf    = ac_f(i,1:acf_len).';

            fd_prms  = getFDynParamsPL(obj, i, stvChi, fhTrqControl, fhTotCWrench, foot_conf, hand_conf, fcp, acf);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    end
end

function vis_data = getVisDataNFB(obj, stmChi, fhTrqControl, noi)
    len = obj.mwbm_config.stvLen;

    % create and initialize the visualization data container:
    stvChi  = stmChi(1,1:len).'; % first element
    fd_prms = getFDynParamsNFB(obj, 1, stvChi, fhTrqControl);
    [vis_data, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl] = initVisDataStruct(fd_prms, noi);

    % put all data into the data container:
    for i = 1:noi
        stvChi   = stmChi(i,1:len).';
        fd_prms  = getFDynParamsNFB(obj, i, stvChi, fhTrqControl);
        vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
    end
end

function vis_data = getVisDataNFBEF(obj, stmChi, fhTrqControl, clnk_conf, fe_c, ac, noi)
    len = obj.mwbm_config.stvLen;

    % initialization:
    [fe_1, fe_len, fe_const] = getFirstVector(fe_c, noi);
    [ac_1, ac_len, ac_const] = getFirstVector(ac, noi);

    stvChi  = stmChi(1,1:len).';
    fd_prms = getFDynParamsNFBEF(obj, 1, stvChi, fhTrqControl, clnk_conf, fe_1, ac_1);
    [vis_data, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl] = initVisDataStruct(fd_prms, noi);

    % put all data into the data container:
    if (fe_const && ac_const)
        % ext. forces and contact accelerations are constant:
        for i = 1:noi
            stvChi   = stmChi(i,1:len).';
            fd_prms  = getFDynParamsNFBEF(obj, i, stvChi, fhTrqControl, clnk_conf, fe_1, ac_1);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    elseif fe_const
        % only the ext. forces are constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            a_c    = ac(i,1:ac_len).';

            fd_prms  = getFDynParamsNFBEF(obj, i, stvChi, fhTrqControl, clnk_conf, fe_1, a_c);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    elseif ac_const
        % only the contact accelerations are constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            fe     = fe_c(i,1:fe_len).';

            fd_prms  = getFDynParamsNFBEF(obj, i, stvChi, fhTrqControl, clnk_conf, fe, ac_1);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    else
        % both vectors are row vector lists (matrices):
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            fe     = fe_c(i,1:feh_len).';
            a_c    = ac(i,1:acf_len).';

            fd_prms  = getFDynParamsNFBEF(obj, i, stvChi, fhTrqControl, clnk_conf, fe, a_c);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    end
end

function vis_data = getVisDataNFBPL(obj, stmChi, fhTrqControl, fhTotCWrench, hand_conf, f_cp, noi)
    len = obj.mwbm_config.stvLen;

    % initialization:
    [fcp_1, fcp_len, fcp_const] = getFirstVector(f_cp, noi);
    stvChi  = stmChi(1,1:len).';
    fd_prms = getFDynParamsNFBPL(obj, 1, stvChi, fhTrqControl, fhTotCWrench, hand_conf, fcp_1);
    [vis_data, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl] = initVisDataStruct(fd_prms, noi);

    % put all data into the data container:
    if fcp_const
        % the payload forces are constant:
        for i = 1:noi
            stvChi   = stmChi(i,1:len).';
            fd_prms  = getFDynParamsNFBPL(obj, i, stvChi, fhTrqControl, fhTotCWrench, foot_conf, hand_conf, fcp_1);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    else
        % the payload force is a row vector list (matrix):
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            fcp    = f_cp(i,1:fcp_len).';

            fd_prms  = getFDynParamsNFBPL(obj, i, stvChi, fhTrqControl, fhTotCWrench, hand_conf, fcp);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    end
end

function vis_data = getVisDataCLPCEF(obj, stmChi, fhTrqControl, clnk_conf, fe_c, ac, noi)
    len = obj.mwbm_config.stvLen;

    % initialization:
    [fe_1, fe_len, fe_const] = getFirstVector(fe_c, noi);
    [ac_1, ac_len, ac_const] = getFirstVector(ac, noi);

    stvChi  = stmChi(1,1:len).';
    fd_prms = getFDynParamsCLPCEF(obj, 1, stvChi, fhTrqControl, clnk_conf, fe_1, ac_1);
    [vis_data, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl] = initVisDataStruct(fd_prms, noi);

    % put all data into the data container:
    if (fe_const && ac_const)
        % ext. forces and contact accelerations are constant:
        for i = 1:noi
            stvChi   = stmChi(i,1:len).';
            fd_prms  = getFDynParamsCLPCEF(obj, i, stvChi, fhTrqControl, clnk_conf, fe_1, ac_1);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    elseif fe_const
        % only the ext. force vector is constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            a_c    = ac(i,1:ac_len).';

            fd_prms  = getFDynParamsCLPCEF(obj, i, stvChi, fhTrqControl, clnk_conf, fe_1, a_c);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    elseif ac_const
        % only the acceleration vector is constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            fe     = fe_c(i,1:fe_len).';

            fd_prms  = getFDynParamsCLPCEF(obj, i, stvChi, fhTrqControl, clnk_conf, fe, ac_1);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    else
        % both vectors are row vector lists (matrices):
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            fe     = fe_c(i,1:fe_len).';
            a_c    = ac(i,1:ac_len).';

            fd_prms  = getFDynParamsCLPCEF(obj, i, stvChi, fhTrqControl, clnk_conf, fe, a_c);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    end
end

function vis_data = getVisDataFPCEF(obj, stmChi, fhTrqControl, foot_conf, clnk_conf, fe_c, ac, ac_f, noi)
    if ~iscolumn(ac_f)
        error('getVisDataFPCEF: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
    end
    len = obj.mwbm_config.stvLen;

    % initialization:
    [fe_1, fe_len, fe_const] = getFirstVector(fe_c, noi);
    [ac_1, ac_len, ac_const] = getFirstVector(ac, noi);

    stvChi  = stmChi(1,1:len).';
    fd_prms = getFDynParamsFPCEF(obj, 1, stvChi, fhTrqControl, foot_conf, clnk_conf, fe_1, ac_1, ac_f);
    [vis_data, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl] = initVisDataStruct(fd_prms, noi);

    % put all data into the data container:
    if (fe_const && ac_const)
        % ext. forces and foot contact accelerations are constant:
        for i = 1:noi
            stvChi   = stmChi(i,1:len).';
            fd_prms  = getFDynParamsFPCEF(obj, i, stvChi, fhTrqControl, foot_conf, clnk_conf, fe_1, ac_1, ac_f);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    elseif feh_const
        % only the ext. forces are constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            a_c    = ac(i,1:ac_len).';

            fd_prms  = getFDynParamsFPCEF(obj, i, stvChi, fhTrqControl, foot_conf, clnk_conf, fe_1, a_c, ac_f);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    elseif acf_const
        % only the contact accelerations are constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            fe    = fe_c(i,1:fe_len).';

            fd_prms  = getFDynParamsFPCEF(obj, i, stvChi, fhTrqControl, foot_conf, clnk_conf, fe, ac_1, ac_f);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    else
        % both vectors are row vector lists (matrices):
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            fe    = fe_c(i,1:fe_len).';
            a_c   = ac(i,1:ac_len).';

            fd_prms  = getFDynParamsFPCEF(obj, i, stvChi, fhTrqControl, foot_conf, clnk_conf, fe, a_c, ac_f);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    end
end

function vis_data = getVisDataFPCPL(obj, stmChi, fhTrqControl, fhTotCWrench, foot_conf, hand_conf, f_cp, ac_f, noi)
    len = obj.mwbm_config.stvLen;

    % initialization:
    [fcp_1, fcp_len, fcp_const] = getFirstVector(f_cp, noi);
    [acf_1, acf_len, acf_const] = getFirstVector(ac_f, noi);

    stvChi  = stmChi(1,1:len).';
    fd_prms = getFDynParamsFPCPL(obj, 1, stvChi, fhTrqControl, fhTotCWrench, foot_conf, hand_conf, fcp_1, acf_1);
    [vis_data, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl] = initVisDataStruct(fd_prms, noi);

    % put all data into the data container:
    if (fcp_const && acf_const)
        % payload forces and foot contact accelerations are constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';

            fd_prms  = getFDynParamsFPCPL(obj, i, stvChi, fhTrqControl, fhTotCWrench, foot_conf, hand_conf, fcp_1, acf_1);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    elseif fcp_const
        % only the payload forces are constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            acf    = ac_f(i,1:acf_len).';

            fd_prms  = getFDynParamsFPCPL(obj, i, stvChi, fhTrqControl, fhTotCWrench, foot_conf, hand_conf, fcp_1, acf);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    elseif acf_const
        % only the contact accelerations are constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            fcp    = f_cp(i,1:fcp_len).';

            fd_prms  = getFDynParamsFPCPL(obj, i, stvChi, fhTrqControl, fhTotCWrench, foot_conf, hand_conf, fcp, acf_1);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    else
        % both vectors are row vector lists (matrices):
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            fcp    = f_cp(i,1:fcp_len).';
            acf    = ac_f(i,1:acf_len).';

            fd_prms  = getFDynParamsFPCPL(obj, i, stvChi, fhTrqControl, fhTotCWrench, foot_conf, hand_conf, fcp, acf);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    end
end

function vis_data = getVisDataFHPCEF(obj, stmChi, fhTrqControl, foot_conf, hand_conf, fe_h, ac_h, ac_f, noi)
    if ~iscolumn(ac_f)
        error('getVisDataFHPCEF: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
    end
    len = obj.mwbm_config.stvLen;

    % initialization:
    [feh_1, feh_len, feh_const] = getFirstVector(fe_h, noi);
    [ach_1, ach_len, ach_const] = getFirstVector(ac_h, noi);

    stvChi  = stmChi(1,1:len).';
    fd_prms = getFDynParamsFHPCEF(obj, 1, stvChi, fhTrqControl, foot_conf, hand_conf, feh_1, ach_1, ac_f);
    [vis_data, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl] = initVisDataStruct(fd_prms, noi);

    % put all data into the data container:
    if (feh_const && ach_const)
        % ext. forces (hands) and foot contact accelerations are constant:
        for i = 1:noi
            stvChi   = stmChi(i,1:len).';
            fd_prms  = getFDynParamsFHPCEF(obj, i, stvChi, fhTrqControl, foot_conf, hand_conf, feh_1, ach_1, ac_f);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    elseif feh_const
        % only the ext. forces are constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            ach    = ac_h(i,1:ach_len).';

            fd_prms  = getFDynParamsFHPCEF(obj, i, stvChi, fhTrqControl, foot_conf, hand_conf, feh_1, ach, ac_f);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    elseif acf_const
        % only the contact accelerations are constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            feh    = fe_h(i,1:feh_len).';

            fd_prms  = getFDynParamsFHPCEF(obj, i, stvChi, fhTrqControl, foot_conf, hand_conf, feh, ach_1, ac_f);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    else
        % both vectors are row vector lists (matrices):
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            feh    = fe_h(i,1:feh_len).';
            ach    = ac_h(i,1:ach_len).';

            fd_prms  = getFDynParamsFHPCEF(obj, i, stvChi, fhTrqControl, foot_conf, hand_conf, feh, ach, ac_f);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    end
end

function vis_data = getVisDataFHPCPL(obj, stmChi, fhTrqControl, fhTotCWrench, foot_conf, hand_conf, f_cp, ac_f, noi)
    len = obj.mwbm_config.stvLen;

    % initialization:
    [fcp_1, fcp_len, fcp_const] = getFirstVector(f_cp, noi);
    [acf_1, acf_len, acf_const] = getFirstVector(ac_f, noi);

    stvChi  = stmChi(1,1:len).';
    fd_prms = getFDynParamsFHPCPL(obj, 1, stvChi, fhTrqControl, fhTotCWrench, foot_conf, hand_conf, fcp_1, acf_1);
    [vis_data, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl] = initVisDataStruct(fd_prms, noi);

    % put all data into the data container:
    if (fcp_const && acf_const)
        % payload forces and foot contact accelerations are constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';

            fd_prms  = getFDynParamsFHPCPL(obj, i, stvChi, fhTrqControl, fhTotCWrench, foot_conf, hand_conf, fcp_1, acf_1);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    elseif fcp_const
        % only the payload forces are constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            acf    = ac_f(i,1:acf_len).';

            fd_prms  = getFDynParamsFHPCPL(obj, i, stvChi, fhTrqControl, fhTotCWrench, foot_conf, hand_conf, fcp_1, acf);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    elseif acf_const
        % only the contact accelerations are constant:
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            fcp    = f_cp(i,1:fcp_len).';

            fd_prms  = getFDynParamsFHPCPL(obj, i, stvChi, fhTrqControl, fhTotCWrench, foot_conf, hand_conf, fcp, acf_1);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    else
        % both vectors are row vector lists (matrices):
        for i = 1:noi
            stvChi = stmChi(i,1:len).';
            fcp    = f_cp(i,1:fcp_len).';
            acf    = ac_f(i,1:acf_len).';

            fd_prms  = getFDynParamsFHPCPL(obj, i, stvChi, fhTrqControl, fhTotCWrench, foot_conf, hand_conf, fcp, acf);
            vis_data = setVisData(i, vis_data, fd_prms, fnames_base, nflds_base, fnames_ctrl, nflds_ctrl);
        end
    end
end
