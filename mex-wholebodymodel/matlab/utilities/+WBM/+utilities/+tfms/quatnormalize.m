function uquat = quatnormalize(quat)
    qmag = WBM.utilities.tfms.quatmagnitude(quat);

    %% Compute the normalized (unit) quaternion q_u of a quaternion q:
    % Further details can be found in:
    %   [1] MathWorks, Documentation - Aerospace Toolbox: <http://mathworks.com/help/aerotbx/ug/quatnormalize.html>.
    uquat = quat/qmag;
end
