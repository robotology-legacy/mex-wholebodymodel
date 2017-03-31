function de = rotm2deul(rotm, omega, sequence)
    if ~exist('sequence', 'var')
        % use the default sequence ...
        sequence = 'ZYX';
    end
    eul = WBM.utilities.rotm2eul(rotm, sequence);
    de  = WBM.utilities.deul(eul, omega, sequence);
end
