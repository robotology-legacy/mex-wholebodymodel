function da = daxang(axang, omega)
    WBM.utilities.checkCVecDim(omega, 3, 'daxang');

    B_inv = WBM.utilities.axang2angRateTF(axang);
    da    = B_inv*omega;
end
