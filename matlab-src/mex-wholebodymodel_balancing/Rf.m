function R = Rf(u)
% Rf calculates the rotation matrix from a quaternion (real followed by imaginary definition)
% R = Rf(u) returns a 3x3 valid rotation matrix from a given quaternion u (4x1), where u(1) is 
% a scalar parameter and u(2:4) is a vector parameter

qt_b_mod_s = u(1);
qt_b_mod_r = u(2:4);
R          = eye(3) + 2*qt_b_mod_s*skew(qt_b_mod_r) + 2*skew(qt_b_mod_r)^2;
    
end

