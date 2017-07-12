function pos_c = sph2cart(pos_s)
    WBM.utilities.chkfun.checkCVecDim(pos_s, 3, 'sph2cart');

    %% Transform spherical coordinates to Cartesian:
    %  Sources:
    %   [1] Advanced Robotic Manipulation: Lecture Notes (CS327A), Oussama Khatib, Stanford University, Spring 2005,
    %       <http://www.in.tum.de/fileadmin/user_upload/Lehrstuehle/Lehrstuhl_XXIII/AdvancedRoboticManipulation.pdf>, p. 26.
    %   [2] Robot Dynamics: Lecture Notes (151-0851-00L), M. Hutter & R. Siegwart & T. Stastny, ETH-Zürich, Jan. 2017,
    %       <https://www.ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf>,
    %       p. 7, eq. (2.7) & (2.8).
    rho   = pos_s(1,1);
    theta = pos_s(2,1);
    phi   = pos_s(3,1);

    c_theta = cos(theta);
    s_theta = sin(theta);
    c_phi   = cos(phi);
    s_phi   = sin(phi);

    pos_c = zeros(3,1);
    pos_c(1,1) = rho*c_theta*s_phi; % x
    pos_c(2,1) = rho*s_theta*s_phi; % y
    pos_c(3,1) = rho*c_phi;         % z
end
