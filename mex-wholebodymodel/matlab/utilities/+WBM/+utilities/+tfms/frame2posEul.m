function [pos, eul] = frame2posEul(vqT, sequence)
    if (nargin ~= 2)
        % use the default sequence ...
        sequence = 'ZYX';
    end
    [pos, R] = WBM.utilities.tfms.frame2posRotm(vqT);
    eul      = WBM.utilities.tfms.rotm2eul(R, sequence);
end
