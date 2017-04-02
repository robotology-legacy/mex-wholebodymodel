function b_X_a = iadjoint(varargin)
    %% Inverse adjoint transformation:
    %
    %  This matrix describes a coordinate transformation of screws (twists) from frame a to frame b.
    %  Further details about the transformation matrix can be taken from:
    %   [1] Multibody Dynamics Notation, S. Traversaro & A. Saccon, Eindhoven University of Technology,
    %       Department of Mechanical Engineering, 2016, <http://repository.tue.nl/849895>, p. 7.
    %   [2] A Mathematical Introduction to Robotic Manipulation, Murray & Li & Sastry, CRC Press, 1994, p. 56.
    %   [3] Passivity-Based Control and Estimation in Networked Robotics, T. Hatanaka & N. Chopra & M. Fujita & M. W. Spong,
    %       Springer, 2015, p. 284, eq. (B.35).
    %   [4] Robotics: Modelling, Planning and Control, B. Siciliano & L. Sciavicco & L. Villani & G. Oriolo,
    %       Springer, 2010, p. 150, eq. (3.113).
    %   [5] Rigid Body Dynamics Algorithms, Roy Featherstone, Springer, 2008, p. 22, eq. (2.24)-(2.27).
    %   [6] Modeling, Identification and Control of Robots, W. Khalil & E. Dombre, Kogan Page Science, 2004,
    %       p. 29, eq. (2.47).
    switch nargin
        case 2
            % general case:
            a_R_b = varargin{1,1};
            a_p_b = varargin{1,2};

            WBM.utilities.checkMatCVecDs(a_R_b, a_p_b, 3, 3, 'iadjoint');

            S_p = WBM.utilities.skewm(a_p_b); % skew-symmetric matrix S(a_p_b)
            R_t = a_R_b.';

            b_X_a = zeros(6,6);
            b_X_a(1:3,1:3) = R_t;
            b_X_a(1:3,4:6) = R_t*S_p;
            b_X_a(4:6,4:6) = R_t;
        case 1
            if iscolumn(varargin{1,1})
                % a_R_b is an identity matrix and
                % a_p_b is not zero:
                a_p_b = varargin{1,1};

                WBM.utilities.checkCVecDim(a_p_b, 3, 'iadjoint');
                S_p = WBM.utilities.skewm(a_p_b);

                b_X_a = eye(6,6);
                b_X_a(1:3,4:6) = S_p;
            elseif ismatrix(varargin{1,1})
                % a_p_b is a 0-vector:
                a_R_b = varargin{1,1};

                WBM.utilities.checkMatDim(a_R_b, 3, 3, 'iadjoint');
                R_t = a_R_b.';

                b_X_a = zeros(6,6);
                b_X_a(1:3,1:3) = R_t;
                b_X_a(4:6,4:6) = R_t;
            else
                error('iadjoint: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
        otherwise
            error('iadjoint: %s', WBM.wbmErrorMsg.WRONG_ARG);
    end
end
