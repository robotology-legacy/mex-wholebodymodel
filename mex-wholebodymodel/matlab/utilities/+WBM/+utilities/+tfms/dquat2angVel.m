function omega = dquat2angVel(dquat, quat)
    WBM.utilities.chkfun.checkCVecDim(dquat, 4, 'dquat2angVel');

    B = WBM.utilities.tfms.quat2angVelTF(quat);
    omega = 2*B*dquat;
end
