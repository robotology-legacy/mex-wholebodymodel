function quat = axisAngle2quat(axang)
    if (size(axang,1) ~= 4)
        error('axisAngle2quat: %s', wbmErrMsg.WRONG_VEC_DIM);
    end

    q = zeros(4,1);
    theta_half = axang(4,1) * 0.5; % rotation angle in radians / 2
    sAngle = sin(theta_half);
    % the direct assignment is much faster than an array-concatenation ...
    q(1,1) = cos(theta_half);
    q(2,1) = axang(1,1) * sAngle;        
    q(3,1) = axang(2,1) * sAngle;
    q(4,1) = axang(3,1) * sAngle;

    quat = q./norm(q); % normalized (unit) quaternion
end
