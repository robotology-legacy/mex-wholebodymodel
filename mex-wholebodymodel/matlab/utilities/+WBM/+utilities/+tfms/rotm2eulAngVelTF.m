function [eul, B] = rotm2eulAngVelTF(rotm, sequence)
    if (nargin ~= 2)
        % use the default sequence ...
        sequence = 'ZYX';
    end
    eul = WBM.utilities.tfms.rotm2eul(rotm, sequence);
    B   = WBM.utilities.tfms.eul2angVelTF(eul, sequence);
end
