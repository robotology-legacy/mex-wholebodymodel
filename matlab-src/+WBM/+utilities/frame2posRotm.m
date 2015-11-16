function [pos, rotm] = frame2posRotm(qT)
    if (length(qT) ~= 7)
        error('frame2posRotm: %s', wbmErrMsg.WRONG_VEC_SIZE);
    end

    pos  = qT(1:3);
    quat = qT(4:end);
    % compute the orthonormal rotation matrix ...
    rotm = WBM.utilities.quat2dcm(quat);
end
