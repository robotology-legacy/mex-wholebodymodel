function dq = rotm2dquat(rotm, omega, varargin)
    WBM.utilities.checkMatCVecDs(rotm, omega, 3, 3, 'rotm2dquat');

    q = WBM.utilities.rotm2quat(rotm);
    if (nargin > 2)
        dq = WBM.utilities.dquat(q, omega, varargin);
        return
    end
    % else, nargin == 2 ...
    dq = WBM.utilities.dquat(q, omega);
end
