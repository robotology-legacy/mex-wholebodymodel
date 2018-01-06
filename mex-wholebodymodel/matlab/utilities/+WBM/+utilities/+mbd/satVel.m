function v = satVel(v, varargin)
    eps_v = 1e-5; % default 0-velocity threshold

    switch nargin
        case 1
            % default velocity saturation value
            % for the iCub humanoid robot:
            % Source: <http://wiki.icub.org/brain/velControlThread_8cpp.html>
            max_v = 250; % in [ksps] (kilosample(s) per second)
        case 2
            max_v = varargin{1,1};
        case 3
            max_v = varargin{1,1};
            eps_v = varargin{1,2};
        otherwise
            error('satVel: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    v(abs(v) <= eps_v) = 0;
    v(v >  max_v) =  max_v;
    v(v < -max_v) = -max_v;
end
