function B_inv = sph2linRateTF(pos_s)
    WBM.utilities.checkCVecDim(pos_s, 3, 'sph2linRateTF');

    % p_s = (ρ, θ, ϕ)^T
    rho   = pos_s(1,1);
    theta = pos_s(2,1);
    phi   = pos_s(3,1);

    c_theta = cos(theta);
    s_theta = sin(theta);
    c_phi   = cos(phi);
    s_phi   = sin(phi);

    r_inv  = 1/rho;
    rs_inv = 1/(rho*s_phi);

    %% Linear rate transformation for spherical coordinates:
    %
    %           | c_theta*s_phi         s_theta*s_phi               c_phi|
    %  B(p_s) = |-s_theta/(rho*s_phi)   c_theta*/(rho*s_phi)            0|
    %           | (c_theta*c_phi)/rho   (s_theta*c_phi)/rho    -s_phi/rho|
    %
    %  The transformation matrix relates the linear velocity v = (x_dot, y_dot, z_dot)^T of the body
    %  to the time derivative of the position p_s with spherical coordinates, s.t.
    %
    %       dp_s/dt = B(p_s)^(-1)*v.
    %
    % Sources:
    %   [1] Advanced Robotic Manipulation: Lecture Notes (CS327A), Oussama Khatib, Stanford University, Spring 2005,
    %       <http://www.in.tum.de/fileadmin/user_upload/Lehrstuehle/Lehrstuhl_XXIII/AdvancedRoboticManipulation.pdf>, p. 26, eq. (2.19).
    %   [2] Robot Dynamics: Lecture Notes (151-0851-00L), M. Hutter & R. Siegwart & T. Stastny, ETH-Zürich, Jan. 2017,
    %       <https://www.ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf>,
    %       p. 8, eq. (2.17).
    B_inv = zeros(3,3);
    B_inv(1,1) =  c_theta*s_phi;
    B_inv(1,2) =  s_theta*s_phi;
    B_inv(1,3) =  c_phi;

    B_inv(2,1) = -s_theta*rs_inv;
    B_inv(2,2) =  c_theta*rs_inv;

    B_inv(3,1) =  (c_theta*c_phi)*r_inv;
    B_inv(3,2) =  (s_theta*c_phi)*r_inv;
    B_inv(3,3) = -s_phi*r_inv;
end
