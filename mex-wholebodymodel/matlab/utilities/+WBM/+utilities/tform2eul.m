function eul = tform2eul(tform, sequence)
    WBM.utilities.checkMatDim(tform, 4, 4, 'tform2eul');

    % extract the rotation matrix and transform it ...
    R = tform(1:3,1:3);
    if ~exist('sequence', 'var')
        % use the default axis sequence ZYX ...
        eul = WBM.utilities.rotm2eul(R);
        return
    end
    % else ...
    eul = WBM.utilities.rotm2eul(R, sequence);
end
