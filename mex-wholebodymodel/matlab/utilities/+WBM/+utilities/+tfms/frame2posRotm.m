function [pos, rotm] = frame2posRotm(vqT)
    WBM.utilities.chkfun.checkCVecDim(vqT, 7, 'frame2posRotm');

    pos  = vqT(1:3,1);
    quat = vqT(4:7,1);
    % compute the orthonormal rotation matrix R ...
    rotm = WBM.utilities.tfms.quat2rotm(quat);
end
