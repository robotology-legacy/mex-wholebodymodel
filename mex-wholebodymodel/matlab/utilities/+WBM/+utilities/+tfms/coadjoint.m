function a_cX_b = coadjoint(varargin)
    %% Coadjoint transformation:
    %  This matrix describes a coordinate transformation of wrenches from frame b to frame a.
    %
    %  Further details about the transformation matrix can be taken from:
    %   [1] Multibody Dynamics Notation, S. Traversaro & A. Saccon, Eindhoven University of Technology,
    %       Department of Mechanical Engineering, 2016, <http://repository.tue.nl/849895>, p. 11, eq. (50).
    %   [2] Rigid Body Dynamics Algorithms, Roy Featherstone, Springer, 2008, p. 22, eq. (2.24)-(2.27).
    %   [3] Robotics: Modelling, Planning and Control, B. Siciliano & L. Sciavicco & L. Villani & G. Oriolo,
    %       Springer, 2010, p. 151, eq. (3.116).
    %   [4] A Mathematical Introduction to Robotic Manipulation, Murray & Li & Sastry, CRC Press, 1994, p. 62, eq. (2.66).
    %   [5] Introduction to Mechanics and Symmetry: A Basic Exposition of Classical Mechanical Systems, J. E. Marsden & T. Ratiu,
    %       2nd Edition, Springer, 2009, p. 311, def. (d) Coadjoint Action.
    switch nargin
        case 2
            % general case:
            a_R_b = varargin{1,1};
            a_p_b = varargin{1,2};

            WBM.utilities.chkfun.checkMatCVecDs(a_R_b, a_p_b, 3, 3, 'coadjoint');

            S_p = WBM.utilities.tfms.skewm(a_p_b); % skew-symmetric matrix S(a_p_b)
            R_t = a_R_b.';

            a_cX_b = zeros(6,6);
            a_cX_b(1:3,1:3) = R_t;
            a_cX_b(4:6,1:3) = R_t*S_p;
            a_cX_b(4:6,4:6) = R_t;
        case 1
            % special case:
            if iscolumn(varargin{1,1})
                % a_R_b is an identity matrix and
                % a_p_b is not zero:
                a_p_b = varargin{1,1};

                WBM.utilities.chkfun.checkCVecDim(a_p_b, 3, 'coadjoint');
                S_p = WBM.utilities.tfms.skewm(a_p_b);

                a_cX_b = eye(6,6);
                a_cX_b(4:6,1:3) = S_p;
            else
                error('coadjoint: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
        otherwise
            error('coadjoint: %s', WBM.wbmErrorMsg.WRONG_ARG);
    end
end
