function R = Rf(u)

%calculate the rotation matrix from a quaternion (angle-axis definition)
qt_b_mod_s = u(1);
qt_b_mod_r = u(2:4);
R          = eye(3) + 2*qt_b_mod_s*skew(qt_b_mod_r) + 2*skew(qt_b_mod_r)^2;
    
end

