function pos_c = sph2cart(pos_s)
    WBM.utilities.checkCVecDim(pos_s, 3, 'sph2cart');

    rho   = pos_s(1,1);
    theta = pos_s(2,1);
    phi   = pos_s(3,1);

    c_theta = cos(theta);
    s_theta = sin(theta);
    c_phi   = cos(phi);
    s_phi   = sin(phi);

    pos_c = zeros(3,1);
    pos_c(1,1) = rho*c_theta*s_phi; % x
    pos_c(2,1) = rho*s_theta*s_phi; % y
    pos_c(3,1) = rho*c_phi;         % z
end
