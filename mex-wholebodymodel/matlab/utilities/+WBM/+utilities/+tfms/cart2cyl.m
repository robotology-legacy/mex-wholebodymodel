function pos_z = cart2cyl(pos_c)
    WBM.utilities.chkfun.checkCVecDim(pos_c, 3, 'cart2cyl');

    x = pos_c(1,1);
    y = pos_c(2,1);

    pos_z = zeros(3,1);
    pos_z(1,1) = sqrt(x*x + y*y); % r
    pos_z(2,1) = atan2(y, x);     % theta
    pos_z(3,1) = pos_c(3,1);      % z
end
