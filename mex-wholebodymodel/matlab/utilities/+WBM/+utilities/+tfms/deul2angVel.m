function omega = deul2angVel(deul, eul, sequence)
    WBM.utilities.chkfun.checkCVecDim(deul, 3, 'deul2angVel');

    if ~exist('sequence', 'var')
        % use the default sequence ...
        sequence = 'ZYX';
    end

    B = WBM.utilities.tfms.eul2angVelTF(eul, sequence);
    omega = B*deul;
end
