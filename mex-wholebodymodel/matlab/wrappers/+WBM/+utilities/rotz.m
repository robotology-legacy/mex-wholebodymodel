function R_z = rotz(ang)
    R_z = zeros(3,3);

    R_z(3,3) =  1;
    R_z(1,1) =  cos(ang);
    R_z(1,2) = -sin(ang);
    R_z(2,1) =  sin(ang);
    R_z(2,2) =  cos(ang);
end
