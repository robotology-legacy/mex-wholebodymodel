function de = tform2deul(tform, omega)
    WBM.utilities.checkMatDim(tform, 4, 4, 'tform2deul');

    if ~exist('sequence', 'var')
        % use the default axis sequence ...
        sequence = 'ZYX';
    end

    R  = tform(1:3,1:3);
    de = WBM.utilities.rotm2deul(R, omega, sequence);
end
