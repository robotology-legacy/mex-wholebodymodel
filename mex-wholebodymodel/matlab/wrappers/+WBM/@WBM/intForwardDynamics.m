function [t, stmChi] = intForwardDynamics(obj, tspan, stvChi_0, fhTrqControl, ode_opt, varargin)
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
                        % simple function with payload at the hands:
                        % f_cp = varargin{4}
                        % ac_f = varargin{5}
                        fhTotCWrench = varargin{1,1};
                        feet_conf    = varargin{1,2};
                        hand_conf    = varargin{1,3};

                        checkInputTypes(fhTotCWrench, feet_conf, hand_conf);

                        fhFwdDyn = @(t, chi)forwardDynamicsPL(obj, t, stvChi, fhTrqControl, fhTotCWrench, ...
                                                              feet_conf, hand_conf, varargin{1,4}, varargin{1,5});
                    else
                        % simple function with external forces:
                        % fe_c = varargin{3}
                        % ac   = varargin{4}
                        % ac_f = varargin{5} (must be either zero or constant)
                        feet_conf  = varargin{1,1};
                        clink_conf = varargin{1,2};

                        WBM.utilities.chkfun.checkCLinkConfigs(feet_conf, clink_conf, 'WBM::intForwardDynamics');

                        fhFwdDyn = @(t, chi)forwardDynamicsEF(obj, t, chi, fhTrqControl, feet_conf, ...
                                                              clink_conf, varargin{3:5});
                    end
                case 6
                    % simple function without any pose corrections:
                    % nargin = 6: pc_type = varargin{1}
                    fhFwdDyn = @(t, chi)forwardDynamics(obj, t, chi, fhTrqControl);
                otherwise
                    error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
        case 'fpc'
            % only feet pose correction:
            switch narg
                case 11
                    % pc_type = varargin{6}
                    if isstruct(varargin{1,1})
                        % extended function with external forces:
                        % fe_c = varargin{3}
                        % ac   = varargin{4}
                        % ac_f = varargin{5} (must be either zero or constant)
                        feet_conf  = varargin{1,1};
                        clink_conf = varargin{1,2};

                        WBM.utilities.chkfun.checkCLinkConfigs(feet_conf, clink_conf, 'WBM::intForwardDynamics');

                        fhFwdDyn = @(t, chi)forwardDynamicsFPCEF(obj, t, stvChi, fhTrqControl, feet_conf, ...
                                                                 clink_conf, varargin{3:5});
                    else
                        % extended function with payload at the hands:
                        % f_cp = varargin{4}
                        % ac_f = varargin{5}
                        fhTotCWrench = varargin{1,1};
                        feet_conf    = varargin{1,2};
                        hand_conf    = varargin{1,3};

                        checkInputTypes(fhTotCWrench, feet_conf, hand_conf);

                        fhFwdDyn = @(t, chi)forwardDynamicsFPCPL(obj, t, stvChi, fhTrqControl, fhTotCWrench, ...
                                                                 feet_conf, hand_conf, varargin{1,4}, varargin{1,5});
                    end
                case 8
                    % extended function with only feet pose correction:
                    % ac_f    = varargin{2}
                    % pc_type = varargin{3}
                    feet_conf = varargin{1,1};

                    WBM.utilities.chkfun.checkCLinkConfig(feet_conf, 'WBM::intForwardDynamics');

                    fhFwdDyn = @(t, chi)forwardDynamicsFPC(obj, t, chi, fhTrqControl, feet_conf, varargin{1,2});
                otherwise
                    error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
        case 'hpc'
            % only hand pose correction:
            if (narg ~= 9)
                error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            % fe_h    = varargin{2}
            % ac_h    = varargin{3}
            % pc_type = varargin{4}
            hand_conf = varargin{1,1};

            WBM.utilities.chkfun.checkCLinkConfig(hand_conf, 'WBM::intForwardDynamics');

            fhFwdDyn = @(t, chi)forwardDynamicsHPCEF(obj, t, stvChi, fhTrqControl, hand_conf, ...
                                                     varargin{1,2}, varargin{1,3});
        case 'fhpc'
            % feet and hand pose corrections:
            if (narg ~= 11)
                error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            % pc_type = varargin{6}
            if isstruct(varargin{1,1})
                % extended function with external forces:
                % fe_h = varargin{3}
                % ac_h = varargin{4}
                % ac_f = varargin{5} (must be either zero or constant)
                feet_conf = varargin{1,1};
                hand_conf = varargin{1,2};

                WBM.utilities.chkfun.checkCLinkConfigs(feet_conf, hand_conf, 'WBM::intForwardDynamics');

                fhFwdDyn = @(t, chi)forwardDynamicsFHPCEF(obj, t, stvChi, fhTrqControl, feet_conf, ...
                                                          hand_conf, varargin{3:5});
            else
                % extended function with payload at the hands:
                % f_cp = varargin{4}
                % ac_f = varargin{5}
                fhTotCWrench = varargin{1,1};
                feet_conf    = varargin{1,2};
                hand_conf    = varargin{1,3};

                checkInputTypes(fhTotCWrench, feet_conf, hand_conf);

                fhFwdDyn = @(t, chi)forwardDynamicsFPCPL(obj, t, stvChi, fhTrqControl, fhTotCWrench, ...
                                                         feet_conf, hand_conf, varargin{1,4}, varargin{1,5});
            end
        otherwise
            error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
    end
    % call the ODE-Solver ...
    [t, stmChi] = ode15s(fhFwdDyn, tspan, stvChi_0, ode_opt);
end
%% END of intForwardDynamics.


%% INPUT VERIFICATION:

function checkInputTypes(fhTotCWrench, feet_conf, hand_conf)
    if ~isa(fhTotCWrench, 'function_handle')
        error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
    end
    WBM.utilities.chkfun.checkCLinkConfigs(feet_conf, hand_conf, 'WBM::intForwardDynamics');
end
