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
    %       - axis angles (3x3).
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
            error('mixveltfm: %s', WBM.wbmErrorMsg.WRONG_ARG);
    end
end
