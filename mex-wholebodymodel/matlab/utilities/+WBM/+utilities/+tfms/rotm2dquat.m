function dq = rotm2dquat(rotm, omega, varargin)
    WBM.utilities.chkfun.checkMatCVecDs(rotm, omega, 3, 3, 'rotm2dquat');

    q  = WBM.utilities.tfms.rotm2quat(rotm);
    dq = WBM.utilities.tfms.dquat(q, omega, varargin{:});
end
