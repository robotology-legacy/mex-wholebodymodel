function v = skewm2vec(S)
    WBM.utilities.checkMatDim(S, 3, 3, 'skewm2vec');

    v = zeros(3,1);
    v(1,1) = -S(2,3);
    v(2,1) =  S(1,3);
    v(3,1) = -S(1,2);
end
