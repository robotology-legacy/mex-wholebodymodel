function a_X_b = adjoint(varargin)
    switch nargin
        case 2
            % general case:
            a_R_b = varargin{1,1};
            a_p_b = varargin{1,2};

            WBM.utilities.checkMatCVecDs(a_R_b, a_p_b, 3, 3, 'adjoint');

            % compute the skew-symmetric matrix S(a_p_b) ...
            S_p = WBM.utilities.skewm(a_p_b);

            a_X_b = zeros(6,6);
            a_X_b(1:3,1:3) =  a_R_b;
            a_X_b(1:3,4:6) = -S_p*a_R_b;
            a_X_b(4:6,4:6) =  a_R_b;
        case 1
            if iscolumn(varargin{1,1})
                % a_R_b is an identity matrix and
                % a_p_b is not zero:
                a_p_b = varargin{1,1};

                WBM.utilities.checkCVecDim(a_p_b, 3, 'adjoint');
                S_p = WBM.utilities.skewm(a_p_b);

                a_X_b = eye(6,6);
                a_X_b(1:3,4:6) = -S_p;
            elseif ismatrix(varargin{1,1})
                % a_p_b is a 0-vector:
                a_R_b = varargin{1,1};

                WBM.utilities.checkMatDim(a_R_b, 3, 3, 'adjoint');

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
