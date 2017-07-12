function qmag = quatmagnitude(quat)
    WBM.utilities.chkfun.checkCVecDim(quat, 4, 'quatmagnitude');

    %% Compute the magnitude (length) of a quaternion q:
    % Further details can be found in:
    %   [1] GPGPU Programming for Games and Science, David H. Eberly, CRC Press, 2014, p. 296.
    %   [2] MathWorks, Documentation - Aerospace Toolbox: <http://mathworks.com/help/aerotbx/ug/quatmod.html>.
    %   [3] Theory of Applied Robotics: Kinematics, Dynamics, and Control, Reza N. Jazar, 2nd Edition, Springer, 2010, p. 113, eq. (3.172).
    qnorm = quat.'*quat;
    qmag  = sqrt(qnorm);
end
