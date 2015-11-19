function quat = axisAngle2quat(axang)
    try
        % Matlab R2015a and newer: if the "Robotics System Toolbox" (RST) is installed,
        % use the internal function.
        quat = axang2quat(axang);
    catch
        % else, for Matlab R2014b and earlier, or if the RST is not available ...
        if (size(axang,1) ~= 4)
            error('axisAngle2quat: %s', wbmErrMsg.WRONG_VEC_DIM);
        end
        
        %theta = axang(4,1); % rotation angle in radians
        %q = [cos(theta/2); sin(theta/2).*axang(1:3)]; % easy to read, but slow ...
        
        % direct assignment is much faster than an array-concatenation ...
        q = zeros(4,1);
        theta_half = axang(4,1) * 0.5; % rotation angle in radians / 2
        sAngle = sin(theta_half);
        
        q(1,1) = cos(theta_half);
        q(2,1) = axang(1,1) * sAngle;        
        q(3,1) = axang(2,1) * sAngle;
        q(4,1) = axang(3,1) * sAngle;
        
        quat = q./norm(q); % normalized (unit) quaternion
    end
end
