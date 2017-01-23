function S = skew(v)
    % SKEW function to generate a (3 x 3) skew-symmetric matrix S out of a (3 x 1) vector v.
    S = [ 0    -v(3)  v(2);
          v(3)  0    -v(1);
         -v(2)  v(1)  0  ];
end
