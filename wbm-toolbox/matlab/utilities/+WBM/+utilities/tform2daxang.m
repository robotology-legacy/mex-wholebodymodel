function da = tform2daxang(tform, omega)
    WBM.utilities.checkMatDim(tform, 4, 4, 'tform2daxang');

    R  = tform(1:3,1:3);
    da = WBM.utilities.rotm2daxang(R, omega);
end
