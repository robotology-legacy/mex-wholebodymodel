function B_inv = cyl2linRateTF(pos_z)
    WBM.utilities.chkfun.checkCVecDim(pos_z, 3, 'cyl2linRateTF');

    % p_z = (r, θ, z)^T
    r_inv = 1/pos_z(1,1);
    theta = pos_z(2,1);

    c_theta = cos(theta);
    s_theta = sin(theta);

    %% Linear rate transformation for cylindrical coordinates:
    %
    %                | c_theta     s_theta     0|
    %  B(p_z)^(-1) = |-s_theta/r   c_theta/r   0|
    %                | 0           0           1|
    %
    %  The transformation matrix relates the linear velocity v = (x_dot, y_dot, z_dot)^T of the body
    %  to the time derivative of the position p_z with cylindrical coordinates, s.t.
    %
    %       dp_z/dt = B(p_z)^(-1)*v.
    %
    % Sources:
    %   [1] Advanced Robotic Manipulation: Lecture Notes (CS327A), Oussama Khatib, Stanford University, Spring 2005,
    %       <http://www.in.tum.de/fileadmin/user_upload/Lehrstuehle/Lehrstuhl_XXIII/AdvancedRoboticManipulation.pdf>, p. 26, eq. (2.18).
    %   [2] Robot Dynamics: Lecture Notes (151-0851-00L), M. Hutter & R. Siegwart & T. Stastny, ETH-Zürich, Jan. 2017,
    %       <https://www.ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf>,
    %       p. 8, eq. (2.14).
    B_inv = eye(3,3);
    B_inv(1,1) =  c_theta;
    B_inv(1,2) =  s_theta;

    B_inv(2,1) = -s_theta*r_inv;
    B_inv(2,2) =  c_theta*r_inv;
end
