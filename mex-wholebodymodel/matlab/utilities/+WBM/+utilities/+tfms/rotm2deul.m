function de = rotm2deul(rotm, omega, sequence)
    if (nargin ~= 3)
        % use the default sequence ...
        sequence = 'ZYX';
    end
    eul = WBM.utilities.tfms.rotm2eul(rotm, sequence);
    de  = WBM.utilities.tfms.deul(eul, omega, sequence);
end
