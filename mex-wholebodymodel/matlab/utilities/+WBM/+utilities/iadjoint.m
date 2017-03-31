function b_X_a = iadjoint(varargin)
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
