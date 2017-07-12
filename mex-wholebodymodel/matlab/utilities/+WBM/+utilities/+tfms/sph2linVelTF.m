function B = sph2linVelTF(pos_s)
    WBM.utilities.chkfun.checkCVecDim(pos_s, 3, 'sph2linVelTF');

    % p_s = (ρ, θ, ϕ)^T
    rho   = pos_s(1,1);
    theta = pos_s(2,1);
    phi   = pos_s(3,1);

    c_theta = cos(theta);
    s_theta = sin(theta);
    c_phi   = cos(phi);
    s_phi   = sin(phi);

    %% Body linear velocity transformation for spherical coordinates:
    %
    %           |c_theta*s_phi   -rho*s_theta*s_phi    rho*c_theta*c_phi|
    %  B(p_s) = |s_theta*s_phi    rho*c_theta*s_phi    rho*s_theta*c_phi|
    %           |c_phi            0                   -rho*s_phi        |
    %
    %  The transformation matrix relates the time derivative of the position p_s with
    %  spherical coordinates to the body linear velocity vector v = (x_dot, y_dot, z_dot)^T, s.t.
    %
    %       v = B(p_s)*dp_s/dt.
    %
    % Sources:
    %   [1] Advanced Robotic Manipulation: Lecture Notes (CS327A), Oussama Khatib, Stanford University, Spring 2005,
    %       <http://www.in.tum.de/fileadmin/user_upload/Lehrstuehle/Lehrstuhl_XXIII/AdvancedRoboticManipulation.pdf>, p. 32, eq. (2.41).
    %   [2] Robot Dynamics: Lecture Notes (151-0851-00L), M. Hutter & R. Siegwart & T. Stastny, ETH-Zürich, Jan. 2017,
    %       <https://www.ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf>,
    %       p. 8, eq. (2.16).
    B = zeros(3,3);
    B(1,1) =  c_theta*s_phi;
    B(1,2) = -rho*s_theta*s_phi;
    B(1,3) =  rho*c_theta*c_phi;

    B(2,1) =  s_theta*s_phi;
    B(2,2) =  rho*c_theta*s_phi;
    B(2,3) =  rho*s_theta*c_phi;

    B(3,1) =  c_phi;
    B(3,3) = -rho*s_phi;
end
