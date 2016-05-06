function deul = dEuler(eul, omega, sequence)
    if (size(omega,1) ~= 3)
        error('dEuler: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
    end

    if ~exist('sequence', 'var')
        % use the default sequence ...
        sequence = 'ZYX';
    end

    B_inv = WBM.utilities.eul2angRateTF(eul, sequence);
    deul = B_inv*omega;
end
