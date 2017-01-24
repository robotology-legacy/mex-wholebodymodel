function q_inv = quatInv(q)
    q_conj  = WBM.utilities.quatConj(q);
    epsilon = 1e-12; % min. value to treat a number as zero ...

    %% Compute the inverse q^(-1) of a quaternion q:
    % Further details can be found in:
   	%   [1] GPGPU Programming for Games and Science, David H. Eberly, CRC Press, 2014, p. 296.
	%   [2] Geometric Tools Engine, Documentation: <http://www.geometrictools.com/Documentation/Quaternions.pdf>, p. 2, eq. (5).
	%   [3] Theory of Applied Robotics: Kinematics, Dynamics, and Control, Reza N. Jazar, 2nd Edition, Springer, 2010, p. 113, eq. (3.173).
    n2 = q.'*q;
    if ((n2 - 1) <= epsilon)
    	q_inv = q_conj;
    	return
    end
    % else ...
    n2_inv = 1/n2;
    q_inv  = q_conj*n2_inv;
end
