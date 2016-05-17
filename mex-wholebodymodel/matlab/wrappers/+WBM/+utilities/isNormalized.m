function result = isNormalized(q)
    if ~iscolumn(q)
        error('isNormalized: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
    end
    result  = false;
    epsilon = 1e-12; % min. value to treat a number as zero ...

    if ((q'*q - 1) <= epsilon)
        % the vector is already normalized ...
        result = true;
    end
end
