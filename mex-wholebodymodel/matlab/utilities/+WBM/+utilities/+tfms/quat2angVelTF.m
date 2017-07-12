function B = quat2angVelTF(quat)
    WBM.utilities.chkfun.checkCVecDim(quat, 4, 'quat2angVelTF');

    %% Body angle velocity transformation:
    %
    %         |-q_1    q_0   q_3   -q_2|
    %  B(q) = |-q_2   -q_3   q_0    q_1|
    %         |-q_3    q_2   q_1    q_0|
    %
    %  The transformation matrix relates the time derivative of the quaternion q
    %  to the body angular velocity vector w = (w_x, w_y, w_z)^T, s.t.
    %
    %       w = 2*B(q)*dq/dt.
    %
    % Sources:
    %   [1] Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors, J. Diebel,
    %       Stanford University, 2006, <http://ai.stanford.edu/~diebel/attitude/attitude.pdf>, p. 16, eq. (151).
    %   [2] MATLAB TOOLBOX FOR RIGID BODY KINEMATICS, Hanspeter Schaub & John L. Junkins,
    %       9th AAS/AIAA Astrodynamics Specialist Conference, AAS 99-139, 1999, <http://hanspeterschaub.info/Papers/mtb1.pdf>, p. 11, eq. (49).
    %   [3] GitHub: ShoolCode/ASEN 5010-Spacecraft Attitude Dynamics and Control/AIAA Software (2nd)/Matlab Toolbox,
    %       <https://github.com/ZachDischner/SchoolCode/tree/master/ASEN 5010-Spacecraft Attitude Dynamics and Control/AIAA Software (2nd)/Matlab Toolbox/>
    %   [4] Rigid Body Kinematics and C++ Code, Sergio Pissanetzky, Scientific Controls, 2005, p. 66, eq. (5.29).
    B = zeros(3,4);
    B(1,1) = -quat(2,1);
    B(1,2) =  quat(1,1);
    B(1,3) =  quat(4,1);
    B(1,4) = -quat(3,1);

    B(2,1) = -quat(3,1);
    B(2,2) = -quat(4,1);
    B(2,3) =  quat(1,1);
    B(2,4) =  quat(2,1);

    B(3,1) = -quat(4,1);
    B(3,2) =  quat(3,1);
    B(3,3) = -quat(2,1);
    B(3,4) =  quat(1,1);
end
