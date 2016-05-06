function S = skew(w)
    if (length(w) ~= 3)
        error('skew: %s', WBM.wbmErrorMsg.WRONG_VEC_SIZE);
    end

    % this is twice faster than an array-concatenation ...
    S = zeros(3,3);

    S(1,2) = -w(3);
    S(1,3) =  w(2);

    S(2,1) =  w(3);
    S(2,3) = -w(1);

    S(3,1) = -w(2);
    S(3,2) =  w(1);
end
