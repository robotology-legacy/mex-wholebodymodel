function S = skewm(v)
    WBM.utilities.chkfun.checkCVecDim(v, 3, 'skewm');

    S = zeros(3,3);
    S(1,2) = -v(3,1);
    S(1,3) =  v(2,1);

    S(2,1) =  v(3,1);
    S(2,3) = -v(1,1);

    S(3,1) = -v(2,1);
    S(3,2) =  v(1,1);
end
