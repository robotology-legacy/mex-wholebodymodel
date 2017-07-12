function B_inv = quat2angRateTF(quat)
    WBM.utilities.chkfun.checkCVecDim(quat, 4, 'quat2angRateTF');

    %% Quaternion rate transformation matrix:
    %
    %                       |-q_1   -q_2   -q_3|
    %  B(q)^(-1) = B(q)^T = | q_0   -q_3    q_2|
    %                       | q_3    q_0    q_1|
    %                       |-q_2    q_1    q_0|
    %
    %  The transformation matrix relates the body angular velocity w = (w_x, w_y, w_z)^T
    %  to the time derivative of the quaternion q, s.t.
    %
    %       dq/dt = 1/2*B(q)^(-1)*w.
    %
    % Sources:
    %   [1] Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors, J. Diebel,
    %       Stanford University, 2006, <http://ai.stanford.edu/~diebel/attitude/attitude.pdf>, p. 16, eq. (151) & (159).
    %   [2] MATLAB TOOLBOX FOR RIGID BODY KINEMATICS, Hanspeter Schaub & John L. Junkins,
    %       9th AAS/AIAA Astrodynamics Specialist Conference, AAS 99-139, 1999, <http://hanspeterschaub.info/Papers/mtb1.pdf>, p. 10, eq. (47).
    %   [3] GitHub: ShoolCode/ASEN 5010-Spacecraft Attitude Dynamics and Control/AIAA Software (2nd)/Matlab Toolbox,
    %       <https://github.com/ZachDischner/SchoolCode/tree/master/ASEN 5010-Spacecraft Attitude Dynamics and Control/AIAA Software (2nd)/Matlab Toolbox/>
    %   [4] Optimal Spacecraft Rotational Maneuvers, John L. Junkins & James D. Turner, Elsevier, 1986, pp. 288-289, eq. (8.80).
    B_inv = zeros(4,3);
    B_inv(1,1) = -quat(2,1);
    B_inv(1,2) = -quat(3,1);
    B_inv(1,3) = -quat(4,1);

    B_inv(2,1) =  quat(1,1);
    B_inv(2,2) = -quat(4,1);
    B_inv(2,3) =  quat(3,1);

    B_inv(3,1) =  quat(4,1);
    B_inv(3,2) =  quat(1,1);
    B_inv(3,3) = -quat(2,1);

    B_inv(4,1) = -quat(3,1);
    B_inv(4,2) =  quat(2,1);
    B_inv(4,3) =  quat(1,1);
end
