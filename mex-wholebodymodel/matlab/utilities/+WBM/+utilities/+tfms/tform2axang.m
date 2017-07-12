function axang = tform2axang(tform)
    WBM.utilities.chkfun.checkMatDim(tform, 4, 4, 'tform2axang');

    % extract the rotation matrix and transform it ...
    R = tform(1:3,1:3);
    axang = WBM.utilities.tfms.rotm2axang(R);
end
