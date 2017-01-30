function  v  = fromSkew2Vector(S)
%fromSkew2Vector Function to generate a 3X1 vector from a 3x3 Skew Symmetric matrix.

    v    = zeros(3,1);
    v(1) = -S(2,3);
    v(2) =  S(1,3);
    v(3) = -S(1,2);
end

