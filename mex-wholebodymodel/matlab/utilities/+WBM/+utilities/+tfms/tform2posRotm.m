function [pos, rotm] = tform2posRotm(tform)
    WBM.utilities.chkfun.checkMatDim(tform, 4, 4, 'tform2posRotm');

    % extract the translation vector and the rotation matrix ...
    pos  = tform(1:3,4);
    rotm = tform(1:3,1:3);
end
