function WBMAssertEqual(val1, val2, errorMsg, tol)
    % iDynTreeAssertEqual custom assert function
    %
    %   Don't do anything if val1 == val2, print the error message and exit otherwise!
    %
    if (nargin < 3)
        errorMsg = '';
    end

    if (nargin < 4)
        tol = 1e-5;
    end

    if (norm(val1 - val2) > tol)
        disp(['WBM Tests: ' errorMsg])
        disp(['mismatch between ' mat2str(val1) ' and ' mat2str(val2)])
        assert(norm(val1-val2) <= tol)
    end
end
