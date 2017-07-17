function [t, stmChi] = intForwardDynamics(obj, tspan, stvChi_0, fhTrqControl, ode_opt, varargin)
    if ~isa(fhTrqControl, 'function_handle')
        error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE)
    end
    if (obj.mwbm_config.nCstrs == 0)
        error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.VALUE_LTE_ZERO);
    end

    switch nargin
        case 10
            % extended function with feet and hand pose correction and payload:
            % f_cp = varargin{4}
            % ac_f = varargin{5}
            fhTotCWrench = varargin{1,1};
            feet_conf    = varargin{1,2};
            hand_conf    = varargin{1,3};

            if ~isa(fhTotCWrench, 'function_handle')
                error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE)
            end
            WBM.utilities.chkfun.checkCLinkConfigs(feet_conf, hand_conf, 'WBM::intForwardDynamics');

            fhFwdDyn = @(t, chi)forwardDynamicsFHPCPL(obj, t, chi, fhTrqControl, fhTotCWrench, feet_conf, ...
                                                      hand_conf, varargin{1,4}, varargin{1,5});
        case 9
            % extended function with feet and hand pose correction:
            % fe_h = varargin{3}
            % ac_h = varargin{4}
            feet_conf = varargin{1,1};
            hand_conf = varargin{1,2};
            WBM.utilities.chkfun.checkCLinkConfigs(feet_conf, hand_conf, 'WBM::intForwardDynamics');

            fhFwdDyn = @(t, chi)forwardDynamicsFHPC(obj, t, chi, fhTrqControl, feet_conf, hand_conf, ...
                                                    varargin{1,3}, varargin{1,4});
        case 8
            % extended function with only hand pose correction:
            % (i.e if the robot is mounted on a rod and the feet have no contact to the floor.)
            % fe_h = varargin{2}
            % ac_h = varargin{3}
            hand_conf = varargin{1,1};
            WBM.utilities.chkfun.checkCLinkConfig(hand_conf, 'WBM::intForwardDynamics');

            fhFwdDyn = @(t, chi)forwardDynamicsHPC(obj, t, chi, fhTrqControl, hand_conf, ...
                                                   varargin{1,2}, varargin{1,3});
        case 7
            % extended function with only feet pose correction:
            % ac_f = varargin{2}
            feet_conf = varargin{1,1};
            WBM.utilities.chkfun.checkCLinkConfig(feet_conf, 'WBM::intForwardDynamics');

            fhFwdDyn = @(t, chi)forwardDynamicsFPC(obj, t, chi, fhTrqControl, feet_conf, varargin{1,2});
        case 5
            % simple function without any pose corrections:
            fhFwdDyn = @(t, chi)forwardDynamics(obj, t, chi, fhTrqControl);
        otherwise
            error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    % call the ODE-Solver ...
    [t, stmChi] = ode15s(fhFwdDyn, tspan, stvChi_0, ode_opt);
end
