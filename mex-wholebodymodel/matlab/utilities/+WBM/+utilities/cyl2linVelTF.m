function B = cyl2linVelTF(pos_z)
    WBM.utilities.checkCVecDim(pos_z, 3, 'cyl2linVelTF');

    % p_z = (r, θ, z)^T
    r     = pos_z(1,1);
    theta = pos_z(2,1);

    c_theta = cos(theta);
    s_theta = sin(theta);

    %% Body linear velocity transformation for cylindrical coordinates:
    %
    %           |c_theta   -r*s_theta   0|
    %  B(p_z) = |s_theta    r*c_theta   0|
    %           |0          0           1|
    %
    %  The transformation matrix relates the time derivative of the position p_z with
    %  cylindrical coordinates to the body linear velocity vector v = (x_dot, y_dot, z_dot)^T, s.t.
    %
    %       v = B(p_z)*dp_z/dt.
    %
    % Sources:
    %   [1] Advanced Robotic Manipulation: Lecture Notes (CS327A), Oussama Khatib, Stanford University, Spring 2005,
    %       <http://www.in.tum.de/fileadmin/user_upload/Lehrstuehle/Lehrstuhl_XXIII/AdvancedRoboticManipulation.pdf>, p. 32, eq. (2.40).
    %   [2] Robot Dynamics: Lecture Notes (151-0851-00L), M. Hutter & R. Siegwart & T. Stastny, ETH-Zürich, Jan. 2017,
    %       <https://www.ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf>,
    %       p. 8, eq. (2.15).
    B = eye(3,3);
    B(1,1) =  c_theta;
    B(1,2) = -r*s_theta;

    B(2,1) =  s_theta;
    B(2,2) =  r*c_theta;
end
