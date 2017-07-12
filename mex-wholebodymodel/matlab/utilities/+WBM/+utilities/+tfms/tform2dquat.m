function dq = tform2dquat(tform, omega, varargin)
    WBM.utilities.chkfun.checkMatDim(tform, 4, 4, 'tform2dquat');

    R  = tform(1:3,1:3);
    dq = WBM.utilities.tfms.rotm2dquat(R, omega, varargin{:});
end
