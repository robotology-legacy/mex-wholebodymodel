function qnorm = axisAngle2NormQuat(axang)
    try
        % Matlab R2015a and newer: if the "Robotics System Toolbox" is installed,
        % use the internal function ...
        qnorm = axang2quat(axang);
    catch
        % else, for Matlab R2014b and earlier ...
        if (length(axang) ~= 4)
            error('axisAngle2NormQuat: %s', wbmErrMsg.WRONG_VEC_SIZE);
        end
        
        theta = axang(4); % rotation angle in radians
        q = [cos(theta/2); sin(theta/2).*axang(1:3)];
        qnorm = q./norm(q);
    end
end
