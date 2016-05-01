function R_y = roty(ang)
    R_y = zeros(3,3);

    R_y(2,2) =  1;
    R_y(1,1) =  cos(ang);
    R_y(1,3) =  sin(ang);
    R_y(3,1) = -sin(ang);
    R_y(3,3) =  cos(ang);
end
