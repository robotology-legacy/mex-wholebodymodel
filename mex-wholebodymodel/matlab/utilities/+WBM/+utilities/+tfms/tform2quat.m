function quat = tform2quat(tform)
    WBM.utilities.chkfun.checkMatDim(tform, 4, 4, 'tform2quat');

    % extract the rotation matrix and transform it ...
    R = tform(1:3,1:3);
    quat = WBM.utilities.tfms.rotm2quat(R);
end
