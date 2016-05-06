function [pos, rotm] = frame2posRotm(vqT)
    if (length(vqT) ~= 7)
        error('frame2posRotm: %s', WBM.wbmErrorMsg.WRONG_VEC_SIZE);
    end

    pos  = vqT(1:3);
    quat = vqT(4:7);
    % compute the orthonormal rotation matrix R ...
    rotm = WBM.utilities.quat2dcm(quat);
end
