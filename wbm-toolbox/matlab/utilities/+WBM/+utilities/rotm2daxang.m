function da = rotm2daxang(rotm, omega)
    WBM.utilities.checkMatCVecDs(rotm, omega, 3, 3, 'rotm2daxang');

    a  = WBM.utilities.rotm2axang(rotm);
    da = WBM.utilities.dquat(a, omega);
end
