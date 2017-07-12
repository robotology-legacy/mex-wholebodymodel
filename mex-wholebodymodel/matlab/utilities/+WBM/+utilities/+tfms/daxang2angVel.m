function omega = daxang2angVel(daxang, axang)
    WBM.utilities.chkfun.checkCVecDim(daxang, 4, 'daxang2angVel');

    B = WBM.utilities.tfms.axang2angVelTF(axang);
    omega = B*daxang;
end
