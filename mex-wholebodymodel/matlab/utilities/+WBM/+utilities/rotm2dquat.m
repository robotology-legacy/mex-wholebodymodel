function dq = rotm2dquat(rotm, omega, varargin)
    WBM.utilities.checkMatCVecDs(rotm, omega, 3, 3, 'rotm2dquat');

    q  = WBM.utilities.rotm2quat(rotm);
    dq = WBM.utilities.dquat(q, omega, varargin{:});
end
