function omega = deul2angVel(deul, eul, sequence)
    WBM.utilities.chkfun.checkCVecDim(deul, 3, 'deul2angVel');

    if (nargin ~= 3)
        % use the default sequence ...
        sequence = 'ZYX';
    end

    B = WBM.utilities.tfms.eul2angVelTF(eul, sequence);
    omega = B*deul;
end
