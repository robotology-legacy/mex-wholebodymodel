function [axang, B] = rotm2axangAngVelTF(rotm)
    axang = WBM.utilities.rotm2axang(rotm);
    B     = WBM.utilities.axang2angVelTF(axang);
end
