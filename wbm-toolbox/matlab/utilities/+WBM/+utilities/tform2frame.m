function vqT = tform2frame(tform)
    WBM.utilities.checkMatDim(tform, 4, 4, 'tform2frame');

    vqT = zeros(7,1);
    R   = tform(1:3,1:3); % extract the rotation matrix ...

    vqT(1:3,1) = tform(1:3,4);               % translation
    vqT(4:7,1) = WBM.utilities.rotm2quat(R); % orientation
end
