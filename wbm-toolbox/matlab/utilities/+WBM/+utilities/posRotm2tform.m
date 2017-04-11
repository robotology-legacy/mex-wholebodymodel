function tform = posRotm2tform(pos, rotm)
    WBM.utilities.checkMatCVecDs(rotm, pos, 3, 3, 'posRotm2tform');

    % create the homogeneous transformation matrix:
    tform = eye(4,4);
    tform(1:3,1:3) = rotm; % rotation
    tform(1:3,4)   = pos;  % translation
end
