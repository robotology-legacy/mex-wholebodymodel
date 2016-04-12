function tform = posRotm2tform(pos, rotm)
    if ( (size(pos,1) ~= 3) || ~isequal(size(rotm), [3,3]) )
        error('posRotm2tform: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
    end
    % create the homogeneous transformation matrix:
    tform = zeros(4,4);
    tform(1:3,1:3) = rotm; % rotation
    tform(1:3,4)   = pos;  % translation
    tform(4,1:3)   = zeros(1,3);
    tform(4,4)     = 1;
end
