function tform = quat2tform(quat)
    tform = zeros(4,4);
    p0    = zeros(1,3);

    % build the homogeneous transformation matrix ...
    tform(1:3,1:3) = WBM.utilities.quat2rotm(quat);
    tform(1:3,4)   = p0';
    tform(4,1:3)   = p0;
    tform(4,4)     = 1;
end
