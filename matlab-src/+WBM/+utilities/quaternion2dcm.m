function dcm = quaternion2dcm(quat)
    try
        % Matlab R2015a and newer: if the "Robotics System Toolbox" is installed,
        % use the internal function ...
        dcm = quat2rotm(quat);
    catch
        % else, for R2014b and earlier ...
        qt_b_mod_s = quat(1);     % scalar part
        qt_b_mod_r = quat(2:end); % (imaginary) vector part

        % calculate the Direction Cosine Matrix (DCM):
        % Note: For more background on this formula, please check
        %       https://www-sop.inria.fr/act_recherche/formulaire/uploads/phd-425.pdf,
        %       page 101, formula 3.8.
        dcm = eye(3) + 2*qt_b_mod_s*skew(qt_b_mod_r) + 2*skew(qt_b_mod_r)^2;
    end
end
