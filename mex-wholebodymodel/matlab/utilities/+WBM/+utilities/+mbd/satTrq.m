function tau = satTrq(tau, varargin)
    eps_t = 1e-5; % default 0-torque threshold

    switch nargin
        case 1
            % default torque saturation value
            % for the iCub humanoid robot:
            max_t = 1e5; % in [ksps] (kilosample(s) per second)
        case 2
            max_t = varargin{1,1};
        case 3
            max_t = varargin{1,1};
            eps_t = varargin{1,2};
        otherwise
            error('satTrq: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    tau(abs(tau) <= eps_t) = 0;
    tau(tau >  max_t) =  max_t;
    tau(tau < -max_t) = -max_t;
end
