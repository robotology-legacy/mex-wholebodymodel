function rX = mixratetfm(varargin)
    %% Mixed rate transformation matrix:
    %   supported coordinate transformations of size (3x3) for Ep_inv:
    %       - cartesian,
    %       - cylindrical,
    %       - spherical.
    %
    %   supported orientation transformations for Er_inv:
    %       - Euler angles (3x3),
    %       - quaternions/Euler parameters (4x3),
    %       - axis-angle (4x3).
    %
    % Useful for example to calculate the analytical Jacobian.
    %
    % Sources:
    %   [1] Robotics, Vision & Control: Fundamental Algorithms in Matlab, Peter I. Corke, Springer, 2011, pp. 176-177.
    %   [2] Robot Modeling and Control, M. W. Spong & S. Hutchinson & M. Vidyasagar, Wiley, 2005, pp. 131-132, eq. (4.108).
    switch nargin
        case 2
            % general case:
            Ep_inv = varargin{1,1};
            Er_inv = varargin{1,2};

            % check the dimensions ...
            [m1,n1] = size(Ep_inv);
            [m2,n2] = size(Er_inv);
            m = m1 + m2;
            if ( (m < 6) || (m > 7) || ((n1 + n2) ~= 6) )
                error('mixratetfm: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end

            rX = zeros(m,6);
            rX(1:3,1:3) = Ep_inv;
            rX(4:m,4:6) = Er_inv;
        case 1
            % if Ep_inv is cartesian:
            Er_inv = varargin{1,1};

            [m1,n1] = size(Er_inv);
            if ( (m1 < 3) || (m1 > 4) || (n1 ~= 3) )
                error('mixratetfm: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end
            m = m1 + n1;

            rX = eye(m,6);
            rX(4:m,4:6) = Er_inv;
        case 0
            rX = eye(6,6); % default
        otherwise
            error('mixratetfm: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
end
