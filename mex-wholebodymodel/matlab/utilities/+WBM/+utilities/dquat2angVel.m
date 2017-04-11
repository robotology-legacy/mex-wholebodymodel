function omega = dquat2angVel(dquat, quat)
    WBM.utilities.checkCVecDim(dquat, 4, 'dquat2angVel');

    B = WBM.utilities.quat2angVelTF(quat);
    omega = 2*B*dquat;
end
