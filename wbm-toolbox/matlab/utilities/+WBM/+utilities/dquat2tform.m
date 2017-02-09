function tform = dquat2tform(quat, omega)
    % build the homogeneous transformation matrix:
    tform = eye(4,4);
    dq = WBM.utilities.dquat(quat, omega);
    tform(1:3,1:3) = WBM.utilities.quat2rotm(dq);
end
