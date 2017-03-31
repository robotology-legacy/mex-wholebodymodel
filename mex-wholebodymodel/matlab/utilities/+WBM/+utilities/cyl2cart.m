function pos_c = cyl2cart(pos_z)
    WBM.utilities.checkCVecDim(pos_z, 3, 'cyl2cart');

    r     = pos_z(1,1);
    theta = pos_z(2,1);

    pos_c = zeros(3,1);
    pos_c(1,1) = r*cos(theta); % x
    pos_c(2,1) = r*sin(theta); % y
    pos_c(3,1) = pos_z(3,1);   % z
end
