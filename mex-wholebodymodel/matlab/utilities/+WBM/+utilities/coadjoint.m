function a_cX_b = coadjoint(varargin)
    switch nargin
        case 2
            % general case:
            a_R_b = varargin{1,1};
            a_p_b = varargin{1,2};

            WBM.utilities.checkMatCVecDs(a_R_b, a_p_b, 3, 3, 'coadjoint');

            S_p = WBM.utilities.skewm(a_p_b); % skew-symmetric matrix S(a_p_b)
            R_t = a_R_b.';

            a_cX_b = zeros(6,6);
            a_cX_b(1:3,1:3) = R_t;
            a_cX_b(4:6,1:3) = R_t*S_p;
            a_cX_b(4:6,4:6) = R_t;
        case 1
            if iscolumn(varargin{1,1})
                % a_R_b is an identity matrix and
                % a_p_b is not zero:
                a_p_b = varargin{1,1};

                WBM.utilities.checkCVecDim(a_p_b, 3, 'coadjoint');
                S_p = WBM.utilities.skewm(a_p_b);

                a_cX_b = eye(6,6);
                a_cX_b(4:6,1:3) = S_p;
            else
                error('coadjoint: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
        otherwise
            error('coadjoint: %s', WBM.wbmErrorMsg.WRONG_ARG);
    end
end
