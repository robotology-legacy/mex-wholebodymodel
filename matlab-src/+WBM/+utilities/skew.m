function X = skew(x)
    if (length(x) ~= 3)
        error('skew: %s', wbmErrMsg.WRONG_VEC_SIZE);
    end
    X = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];            
end
