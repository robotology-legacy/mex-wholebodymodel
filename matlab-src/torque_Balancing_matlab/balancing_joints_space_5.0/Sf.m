function S = Sf(w)
% Sf 
% defines a skew-symmetric matrix from a vector [3x1]
S = [   0,   -w(3),    w(2);
       w(3),   0,     -w(1);
      -w(2),  w(1),     0 ];
end

