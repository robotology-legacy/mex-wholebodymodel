function da = rotm2daxang(rotm, omega)
    WBM.utilities.chkfun.checkMatCVecDs(rotm, omega, 3, 3, 'rotm2daxang');

    a  = WBM.utilities.tfms.rotm2axang(rotm);
    da = WBM.utilities.tfms.dquat(a, omega);
end
