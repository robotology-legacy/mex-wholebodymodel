function tform = dQuat2tform(quat, omega)
    % build the homogeneous transformation matrix:
    tform = eye(4,4);
    dq = WBM.utilities.dQuat(quat, omega);
    tform(1:3,1:3) = WBM.utilities.quat2rotm(dq);
end
