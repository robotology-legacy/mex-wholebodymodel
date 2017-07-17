function vX = mixveltfm(varargin)
    %% Mixed velocity transformation matrix:
    %   supported coordinate transformations of size (3x3) for E_p:
    %       - cartesian,
    %       - cylindrical,
    %       - spherical.
    %
    %   supported orientation transformations for E_r:
    %       - Euler angles (3x3),
    %       - quaternions/Euler parameters (3x4),
    %       - axis-angle (3x4).
    %
    %  Sources:
    %   [1] Multibody Dynamics Notation, S. Traversaro & A. Saccon, Eindhoven University of Technology,
    %       Department of Mechanical Engineering, 2016, <http://repository.tue.nl/849895>, p. 8, eq. (35).
    %   [2] Advanced Robotic Manipulation: Lecture Notes (CS327A), Oussama Khatib, Stanford University, Spring 2005,
    %       <http://www.in.tum.de/fileadmin/user_upload/Lehrstuehle/Lehrstuhl_XXIII/AdvancedRoboticManipulation.pdf>, p. 26 & p. 31, eq. (2.17) & (2.39).
    %   [3] Robot Dynamics: Lecture Notes (151-0851-00L), M. Hutter & R. Siegwart & T. Stastny, ETH-Zürich, Jan. 2017,
    %       <https://www.ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf>,
    %       p. 41, eq. (2.191).
    switch nargin
        case 2
            % general case:
            E_p = varargin{1,1};
            E_r = varargin{1,2};

            % check the dimensions ...
            [m1,n1] = size(E_p);
            [m2,n2] = size(E_r);
            n = n1 + n2;
            if ( ((m1 + m2) ~= 6) || (n < 6) || (n > 7) )
                error('mixveltfm: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end

            vX = zeros(6,n);
            vX(1:3,1:3) = E_p;
            vX(4:6,4:n) = E_r;
        case 1
            % if E_p is cartesian:
            E_r = varargin{1,1};

            [m1,n1] = size(E_r);
            if ( (m1 ~= 3) || (n1 < 3) || (n1 > 4) )
                error('mixveltfm: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end
            n = m1 + n1;

            vX = eye(6,n);
            vX(4:6,4:n) = E_r;
        case 0
            vX = eye(6,6); % default
        otherwise
            error('mixveltfm: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
end
