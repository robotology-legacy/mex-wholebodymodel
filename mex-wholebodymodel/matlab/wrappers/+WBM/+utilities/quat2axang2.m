function axang = quat2axang2(quat)
    if (size(quat,1) ~= 4)
        error('quat2axang2: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
    end
    axang = zeros(4,1);

    %% Translate the given quaternion to the axis-angle representation (u, theta):
    % For further details, see:
    %   [1] Geometric Tools Engine, Documentation: <http://www.geometrictools.com/Documentation/RotationIssues.pdf>, p. 8.
    %   [2] Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors, J. Diebel, Stanford University, 2006,
    %       <https://www.astro.rug.nl/software/kapteyn/_downloads/attitude.pdf>, p. 18, eq. (198) & (199).
    %   [3] Technical Concepts: Orientation, Rotation, Velocity and Acceleration and the SRM, P. Berner, Version 2.0, 2008,
    %       <http://sedris.org/wg8home/Documents/WG80485.pdf>, p. 36.
    axang(4,1) = 2*acos(quat(1,1)); % rotation angle theta
    if ( (axang(4,1) == 0) || (axang(4,1) == 2*pi) )
        % Null rotation --> singularity: The unit vector u is indeterminate.
        % By convention, set u = (0, 0, 0).
        axang(1,1) = 0;
        axang(2,1) = 0;
        axang(3,1) = 0;
        return
    end
    % else, if theta ~= 0:
    div = 1/sqrt(1 - quat(1,1)*quat(1,1));
    % unit vector n:
    axang(1,1) = quat(2,1)*div;
    axang(2,1) = quat(3,1)*div;
    axang(3,1) = quat(4,1)*div;
    %axang(4,1) = 2*atan2(norm(quat(2:4,1)), quat(1,1)); % variant - numerically more stable.
end
