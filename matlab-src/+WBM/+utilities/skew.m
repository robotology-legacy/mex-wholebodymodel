function S = skew(w)
    if (length(w) ~= 3)
        error('skew: %s', wbmErrMsg.WRONG_VEC_SIZE);
    end
    %X = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0]; % easy to read but slow ...
    
    % faster ...
    S = zeros(3,3);
    
    S(1,2) = -w(3);
    S(1,3) =  w(2);
    
    S(2,1) =  w(3);
    S(2,3) = -w(1);
    
    S(3,1) = -w(2);
    S(3,2) =  w(1);    
end
