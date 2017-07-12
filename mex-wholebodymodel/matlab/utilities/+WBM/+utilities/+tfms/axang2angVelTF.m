function B = axang2angVelTF(axang)
    WBM.utilities.chkfun.checkCVecDim(axang, 4, 'axang2angVelTF');

    u     = axang(1:3,1);
    theta = axang(4,1);

    n = u.'*u;
    if (n > 1)
        u = u./sqrt(n); % normalize u
    end

    %% Body angular velocity transformation:
    %
    %  B(a) = |sin(theta)*I + (1 - cos(theta))*S(u)   u|,
    %
    %  where I is a (3x3)-identity matrix and S(u) denotes the skew-symmetric matrix of u.
    %
    %  The transformation matrix relates the time derivative of the axis-angle vector a = (u, theta)^T
    %  to the body angular velocity vector w = (w_x, w_y, w_z)^T, s.t.
    %
    %       w = B(a)*da/dt.
    %
    % Sources:
    %   [1] Robot Dynamics: Lecture Notes (151-0851-00L), M. Hutter & R. Siegwart & T. Stastny, ETH-Zürich, Jan. 2017,
    %       <https://www.ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf>,
    %       p. 25, eq. (2.102).
    S_u = WBM.utilities.tfms.skewm(u);

    B = zeros(3,4);
    B(1:3,1:3) = sin(theta)*eye(3,3) + (1 - cos(theta))*S_u;
    B(1:3,4)   = u;
end
