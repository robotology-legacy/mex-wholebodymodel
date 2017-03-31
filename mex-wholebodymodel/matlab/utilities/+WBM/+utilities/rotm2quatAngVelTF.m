function [quat, B] = rotm2quatAngVelTF(rotm)
    quat = WBM.utilities.rotm2quat(rotm);
    B    = WBM.utilities.quat2angVelTF(quat);
end
