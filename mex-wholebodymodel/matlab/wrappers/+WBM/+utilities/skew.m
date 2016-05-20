function S = skew(w)
    if (size(w,1) ~= 3)
        error('skew: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
    end
    S = zeros(3,3);

    S(1,2) = -w(3,1);
    S(1,3) =  w(2,1);

    S(2,1) =  w(3,1);
    S(2,3) = -w(1,1);

    S(3,1) = -w(2,1);
    S(3,2) =  w(1,1);
end
