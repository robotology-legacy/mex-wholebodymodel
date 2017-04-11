function B_inv = axang2angRateTF(axang)
    WBM.utilities.checkCVecDim(axang, 4, 'axang2angRateTF');

    u     = axang(1:3,1);
    theta = axang(4,1);

    u_t = u.';
    n = u_t*u;
    if (n > 1)
        u = u./sqrt(n); % normalize u
    end

    %% Axis-angle rate transformation:
    %
    %              |-1/2*(sin(theta)/(1 - cos(theta)))*S(u)^2 - 1/2*S(u)|
    % B^(-1)(a) =  |                                                    |,
    %              |                        u^T                         |
    %
    %  where S(u) denotes the skew-symmetric matrix of u.
    %
    %  The transformation matrix relates the angular velocity vector  w = (w_x, w_y, w_z)^T of the body
    %  to the time derivative of the axis-angle vector a = (u, theta)^T, s.t.
    %
    %       da/dt = B^(-1)(a)*w.
    %
    % Sources:
    %   [1] Robot Dynamics: Lecture Notes (151-0851-00L), M. Hutter & R. Siegwart & T. Stastny, ETH-Zürich, Jan. 2017,
    %       <https://www.ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf>,
    %       p. 25, eq. (2.103).
    S_u  = WBM.utilities.skewm(u);
    S2_u = S_u*S_u; % = [S(u)]^2

    B_inv = zeros(4,3);
    B_inv(1:3,1:3) = -0.5*((sin(theta)/(1 - cos(theta))*S2_u) + S_u);
    B_inv(4,1:3)   = u_t;
end
