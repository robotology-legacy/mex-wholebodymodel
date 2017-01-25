function omega = dEuler2angVel(deul, eul, sequence)
    if (size(deul,1) ~= 3)
        error('dEuler2angVel: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
    end

    if ~exist('sequence', 'var')
        % use the default sequence ...
        sequence = 'ZYX';
    end

    B = WBM.utilities.eul2angVelTF(eul, sequence);
    omega = B*deul;
end
