function a = satAcc(a, varargin)
    eps_a = 1e-5; % default 0-acceleration threshold

    switch nargin
        case 1
            % default acceleration saturation value
            % for the iCub humanoid robot:
            % Source: <http://wiki.icub.org/brain/velControlThread_8cpp.html>
            max_a = WBM.WBM.MAX_JNT_ACC; % in [Ms/s^2] (Megasample(s) per second squared)
        case 2
            max_a = varargin{1,1};
        case 3
            max_a = varargin{1,1};
            eps_a = varargin{1,2};
        otherwise
            error('satAcc: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    a(abs(a) <= eps_a) = 0;
    a(a >  max_a) =  max_a;
    a(a < -max_a) = -max_a;
end
