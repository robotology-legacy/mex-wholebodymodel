function axang = tform2axang(tform)
    WBM.utilities.checkMatDim(tform, 4, 4, 'tform2axang');

    % extract the rotation matrix and transform it ...
    R = tform(1:3,1:3);
    axang = WBM.utilities.rotm2axang(R);
end
