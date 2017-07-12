function pos_s = cart2sph(pos_c)
    WBM.utilities.chkfun.checkCVecDim(pos_c, 3, 'cart2sph');

    x = pos_c(1,1);
    y = pos_c(2,1);
    z = pos_c(3,1);

    r2 = x*x + y*y;

    pos_s = zeros(3,1);
    pos_s(1,1) = sqrt(r2 + z*z);     % rho = sqrt(x^2 + y^2 + z^2)
    pos_s(2,1) = atan2(y, x);        % theta
    pos_s(3,1) = atan2(sqrt(r2), z); % phi = arccos(z/rho) = arctan(sqrt(x^2 + y^2)/z)
end
