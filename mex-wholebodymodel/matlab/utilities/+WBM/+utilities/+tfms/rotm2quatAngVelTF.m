function [quat, B] = rotm2quatAngVelTF(rotm)
    quat = WBM.utilities.tfms.rotm2quat(rotm);
    B    = WBM.utilities.tfms.quat2angVelTF(quat);
end
