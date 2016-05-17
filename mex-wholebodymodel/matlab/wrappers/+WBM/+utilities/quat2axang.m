function axang = quat2axang(quat)
    if (size(quat,1) ~= 4)
        error('quat2axang: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
    end
    axang = zeros(4,1);

    %% Translate the given quaternion to the axis-angle representation (u, theta):
    % For further details, see:
    %   [1] Geometric Tools Engine, Documentation: <http://www.geometrictools.com/Documentation/RotationIssues.pdf>, p. 8.
    %   [2] Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors, J. Diebel, Stanford University, 2006,
    %       <https://www.astro.rug.nl/software/kapteyn/_downloads/attitude.pdf>, p. 18, eq. (198) & (199).
    %   [3] Quaternion Computation, Neil Dantam, Georgia Institute of Technology, 2014, <http://www.neil.dantam.name/note/dantam-quaternion.pdf>,
    %       p. 5, eq. (49) & (50).
    %   [4] A Mathematical Introduction to Robotic Manipulation, Murray & Li & Sastry, CRC Press, 1994, p. 34.
    axang(4,1) = 2*acos(quat(1,1)); % rotation angle theta
    if ( (axang(4,1) == 0) || (axang(4,1) == 2*pi) )
        % Null or full rotation --> singularity: The unit vector u is indeterminate. By convention, set u to the
        % default value (0, 0, 1) according to the ISO/IEC IS 19775-1:2013 standard of the Web3D Consortium.
        % See: <http://www.web3d.org/documents/specifications/19775-1/V3.3/Part01/fieldsDef.html#SFRotationAndMFRotation>
        axang(3,1) = 1;
        return
    end
    % else the general case, if theta ~= 0 and theta ~= 2*pi:
    div = 1/sqrt(1 - quat(1,1)*quat(1,1));
    % unit vector u:
    axang(1,1) = quat(2,1)*div;
    axang(2,1) = quat(3,1)*div;
    axang(3,1) = quat(4,1)*div;
end
