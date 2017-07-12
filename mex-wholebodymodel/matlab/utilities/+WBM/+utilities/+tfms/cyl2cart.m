function pos_c = cyl2cart(pos_z)
    WBM.utilities.chkfun.checkCVecDim(pos_z, 3, 'cyl2cart');

    %% Transform cylindrical coordinates to Cartesian:
    %  Sources:
    %   [1] Advanced Robotic Manipulation: Lecture Notes (CS327A), Oussama Khatib, Stanford University, Spring 2005,
    %       <http://www.in.tum.de/fileadmin/user_upload/Lehrstuehle/Lehrstuhl_XXIII/AdvancedRoboticManipulation.pdf>, p. 26.
    %   [2] Robot Dynamics: Lecture Notes (151-0851-00L), M. Hutter & R. Siegwart & T. Stastny, ETH-Zürich, Jan. 2017,
    %       <https://www.ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf>,
    %       p. 7, eq. (2.5) & (2.6).
    r     = pos_z(1,1);
    theta = pos_z(2,1);

    pos_c = zeros(3,1);
    pos_c(1,1) = r*cos(theta); % x
    pos_c(2,1) = r*sin(theta); % y
    pos_c(3,1) = pos_z(3,1);   % z
end
