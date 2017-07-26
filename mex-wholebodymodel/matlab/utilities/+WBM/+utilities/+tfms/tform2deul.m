function de = tform2deul(tform, omega, sequence)
    WBM.utilities.chkfun.checkMatDim(tform, 4, 4, 'tform2deul');

    if (nargin ~= 3)
        % use the default axis sequence ...
        sequence = 'ZYX';
    end

    R  = tform(1:3,1:3);
    de = WBM.utilities.tfms.rotm2deul(R, omega, sequence);
end
