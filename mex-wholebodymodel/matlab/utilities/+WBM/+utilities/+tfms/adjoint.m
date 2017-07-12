function a_X_b = adjoint(varargin)
    %% Adjoint transformation:
    %  This matrix describes a coordinate transformation of screws (twists) from frame b to frame a.
    %
    %  Further details about the transformation matrix can be taken from:
    %   [1] Multibody Dynamics Notation, S. Traversaro & A. Saccon, Eindhoven University of Technology,
    %       Department of Mechanical Engineering, 2016, <http://repository.tue.nl/849895>, p. 6, eq. (27).
    %   [2] A Mathematical Introduction to Robotic Manipulation, Murray & Li & Sastry, CRC Press, 1994, p. 55 & p. 421, eq. (2.57) & (2.58).
    %   [3] Passivity-Based Control and Estimation in Networked Robotics, T. Hatanaka & N. Chopra & M. Fujita & M. W. Spong,
    %       Springer, 2015, p. 91 & p. 283, eq. (5.14) & (B.30).
    %   [4] Robotics: Modelling, Planning and Control, B. Siciliano & L. Sciavicco & L. Villani & G. Oriolo,
    %       Springer, 2010, p. 150, eq. (3.112).
    %   [5] Rigid Body Dynamics Algorithms, Roy Featherstone, Springer, 2008, p. 22, eq. (2.24)-(2.27).
    %   [6] Modeling, Identification and Control of Robots, W. Khalil & E. Dombre, Kogan Page Science, 2004,
    %       pp. 28-29, eq. (2.44) & (2.45).
    switch nargin
        case 2
            % general case:
            a_R_b = varargin{1,1};
            a_p_b = varargin{1,2};

            WBM.utilities.chkfun.checkMatCVecDs(a_R_b, a_p_b, 3, 3, 'adjoint');

            % compute the skew-symmetric matrix S(a_p_b) ...
            S_p = WBM.utilities.tfms.skewm(a_p_b);

            a_X_b = zeros(6,6);
            a_X_b(1:3,1:3) =  a_R_b;
            a_X_b(1:3,4:6) = -S_p*a_R_b;
            a_X_b(4:6,4:6) =  a_R_b;
        case 1
            if iscolumn(varargin{1,1})
                % a_R_b is an identity matrix and
                % a_p_b is not zero:
                a_p_b = varargin{1,1};

                WBM.utilities.chkfun.checkCVecDim(a_p_b, 3, 'adjoint');
                S_p = WBM.utilities.tfms.skewm(a_p_b);

                a_X_b = eye(6,6);
                a_X_b(1:3,4:6) = -S_p;
            elseif ismatrix(varargin{1,1})
                % a_p_b is a 0-vector:
                a_R_b = varargin{1,1};

                WBM.utilities.chkfun.checkMatDim(a_R_b, 3, 3, 'adjoint');

                a_X_b = zeros(6,6);
                a_X_b(1:3,1:3) = a_R_b;
                a_X_b(4:6,4:6) = a_R_b;
            else
                error('adjoint: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
        case 0
            % default: a_R_b is an identity matrix and
            %          a_p_b is a 0-vector.
            a_X_b = eye(6,6);
        otherwise
            error('adjoint: %s', WBM.wbmErrorMsg.WRONG_ARG);
    end
end
