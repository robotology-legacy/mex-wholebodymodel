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
                        foot_conf    = varargin{1,2};
                        hand_conf    = varargin{1,3};

                        checkInputTypes(fhTotCWrench, foot_conf, hand_conf);

                        fhFwdDyn = @(t, chi)forwardDynamicsPL(obj, t, chi, fhTrqControl, fhTotCWrench, ...
                                                              foot_conf, hand_conf, varargin{1,4}, varargin{1,5});
                    else
                        % simple function with external forces:
                        % fe_c = varargin{3}
                        % ac   = varargin{4}
                        % ac_f = varargin{5} (must be either zero or constant)
                        foot_conf = varargin{1,1};
                        clnk_conf = varargin{1,2};

                        WBM.utilities.chkfun.checkCLinkConfigs(foot_conf, clnk_conf, 'WBM::intForwardDynamics');

                        fhFwdDyn = @(t, chi)forwardDynamicsEF(obj, t, chi, fhTrqControl, foot_conf, ...
                                                              clnk_conf, varargin{3:5});
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
                        % function with payload at the hands:
                        % f_cp = varargin{3}
                        fhTotCWrench = varargin{1,1};
                        hand_conf    = varargin{1,2};

                        if ~isa(fhTotCWrench, 'function_handle')
                            error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
                        end
                        WBM.utilities.chkfun.checkCLinkConfig(hand_conf, 'WBM::intForwardDynamics');

                        fhFwdDyn = @(t, chi)forwardDynamicsNFBPL(obj, t, chi, fhTrqControl, fhTotCWrench, ...
                                                                 hand_conf, varargin{1,3});
                    else
                        % function with external forces:
                        % fe_c = varargin{2}
                        % ac   = varargin{3}
                        clnk_conf = varargin{1,1};

                        WBM.utilities.chkfun.checkCLinkConfig(clnk_conf, 'WBM::intForwardDynamics');

                        fhFwdDyn = @(t, chi)forwardDynamicsNFBEF(obj, t, chi, fhTrqControl, clnk_conf, ...
                                                                 varargin{1,2}, varargin{1,3});
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
                        % extended function with payload at the hands:
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
                % extended function with payload at the hands:
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
