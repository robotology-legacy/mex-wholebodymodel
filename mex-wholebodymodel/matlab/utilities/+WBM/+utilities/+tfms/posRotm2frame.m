function vqT = posRotm2frame(pos, rotm)
    WBM.utilities.chkfun.checkCVecDim(pos, 3, 'posRotm2frame');

    % create the VQ-transformation (frame):
    vqT = zeros(7,1);
    vqT(1:3,1) = pos;
    vqT(4:7,1) = WBM.utilities.tfms.rotm2quat(rotm);
end
