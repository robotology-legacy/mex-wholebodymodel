function [pos, dcm] = frame2posRotm(qT)
    if (length(qT) ~= 7)
        error('frame2posRot: Wrong vector size!');
    end

    pos = qT(1:3);
    try
        % Matlab R2015a and newer: if the "Robotics System Toolbox" is installed,
        % use the internal function ...
        q = qT(4:end);
        dcm = quat2rotm(q);
    catch
        % else ...
        qt_b_mod_s = qT(4);     % scalar part
        qt_b_mod_r = qT(5:end); % (imaginary) vector part

        % calculate the Direction Cosine Matrix (DCM):
        dcm = eye(3) - 2*qt_b_mod_s*skew(qt_b_mod_r) + 2*skew(qt_b_mod_r)^2;
    end
end
