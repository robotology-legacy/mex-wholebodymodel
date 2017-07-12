function [axang, B] = rotm2axangAngVelTF(rotm)
    axang = WBM.utilities.tfms.rotm2axang(rotm);
    B     = WBM.utilities.tfms.axang2angVelTF(axang);
end
