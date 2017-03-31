function de = deul(eul, omega, sequence)
    WBM.utilities.checkCVecDim(omega, 3, 'deul');

    if ~exist('sequence', 'var')
        % use the default sequence ...
        sequence = 'ZYX';
    end

    B_inv = WBM.utilities.eul2angRateTF(eul, sequence);
    de    = B_inv*omega;
end
