function eul = tform2eul(tform, sequence)
    WBM.utilities.chkfun.checkMatDim(tform, 4, 4, 'tform2eul');

    if (nargin ~= 2)
        % use the default axis sequence ZYX ...
        sequence = 'ZYX';
    end
    % extract the rotation matrix and transform it ...
    R = tform(1:3,1:3);
    eul = WBM.utilities.tfms.rotm2eul(R, sequence);
end
