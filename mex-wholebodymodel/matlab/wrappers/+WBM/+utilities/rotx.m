function R_x = rotx(ang)
    R_x = zeros(3,3);

    R_x(1,1) =  1;
    R_x(2,2) =  cos(ang);
    R_x(2,3) = -sin(ang);
    R_x(3,2) =  sin(ang);
    R_x(3,3) =  cos(ang);
end
