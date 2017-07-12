function da = daxang(axang, omega)
    WBM.utilities.chkfun.checkCVecDim(omega, 3, 'daxang');

    B_inv = WBM.utilities.tfms.axang2angRateTF(axang);
    da    = B_inv*omega;
end
