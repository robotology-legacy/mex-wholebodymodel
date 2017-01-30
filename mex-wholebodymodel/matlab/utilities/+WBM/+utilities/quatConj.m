function q_conj = quatConj(q)
    if (size(q,1) ~= 4)
        error('quatConj: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
    end
    q_conj = zeros(4,1);

    %% Calculate the conjugate q* of a quaternion q:
    % For further details, please check:
    %   [1] GPGPU Programming for Games and Science, David H. Eberly, CRC Press, 2014, p. 296.
    %   [2] Theory of Applied Robotics: Kinematics, Dynamics, and Control, Reza N. Jazar, 2nd Edition, Springer, 2010, p. 113, eq. (3.170).
    %   [3] Geometric Tools Engine, Documentation: <http://www.geometrictools.com/Documentation/Quaternions.pdf>, p. 2, eq. (3).
    q_conj(1,1) =  q(1,1);
    q_conj(2,1) = -q(2,1);
    q_conj(3,1) = -q(3,1);
    q_conj(4,1) = -q(4,1);
end
