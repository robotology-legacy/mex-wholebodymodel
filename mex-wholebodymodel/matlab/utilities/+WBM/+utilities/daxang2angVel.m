function omega = daxang2angVel(daxang, axang)
    WBM.utilities.checkCVecDim(daxang, 4, 'daxang2angVel');

    B = WBM.utilities.axang2angVelTF(axang);
    omega = B*daxang;
end
