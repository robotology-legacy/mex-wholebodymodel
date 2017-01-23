function [dchi, h, g, fc, kinEnergy] = forwardDynamics_kinEnergyTest(t, chi, param)
    %FORWARDDYNAMICS Forward dynamics of the wholeBodyModel
    %
    %   This is the forward dynamics of the model loaded in the
    %   wholeBodyInterface from the URDF description. The dynamic model is
    %   described as an explicit ordinary differential equation of the form:
    %
    %                dchi = forwardDynamics( t,chi)
    %
    %   where chi is to variable to be integrated. For a floating base
    %   articulated chain, the variable chi conatins the following
    %   subvariables:
    %
    %   x_b:      the cartesian position of the base (R^3)
    %   qt_b:     the quaternion describing the orientation of the base (global parametrization of SO(3))
    %   qj:       the joint positions (R^ndof)
    %   dx_b:     the cartesian velocity of the base (R^3)
    %   omega_b:  the velocity describing the orientation of the base (so(3))
    %   dqj:      the joint velocities (R^ndof)
    %

    %% extraction of state
    ndof = param.ndof;

    param.demux.baseOrientationType = 1;  % sets the base orientation in stateDemux.m as positions + quaternions (1) or transformation matrix (0)
    [basePose,qj,baseVelocity,dqj]  = stateDemux(chi,param);

    % position and orientation
    x_b     = basePose(1:3,:);

    % normalize quaternions to avoid numerical errors
    % qt_b = qt_b/norm(qt_b);

    qt_b    = basePose(4:7,:);

    % linear and angular velocity
    dx_b    = baseVelocity(1:3,:);
    omega_W = baseVelocity(4:6,:);

    v = [dx_b;omega_W;dqj];

    %% MexWholeBodyModel calls
    %reconstructing rotation of root to world from the quaternion
    %[~,T_b,~,~] = wholeBodyModel('get-state');

    w_R_b = quaternion2dcm(qt_b);

    wbm_setWorldFrame(w_R_b,x_b,[0 0 0]');
    wbm_updateState(qj,dqj,[dx_b;omega_W]);

    M = wbm_massMatrix();
    h = wbm_generalizedBiasForces();

    %M = wbm_massMatrix(qj);
    %h = wbm_generalizedBiasForces(qj,dqj,[dx_b;omega_W]);

    g = zeros(size(h));

    %h = zeros(size(h));
    %H = wbm_centroidalMomentum();

    %% Building up contraints jacobian and djdq
    numConstraints = length(param.constraintLinkNames);
    Jc = zeros(6*numConstraints,6+ndof);
    dJcDq = zeros(6*numConstraints,1);
    for i=1:numConstraints
        Jc(6*(i-1)+1:6*i,:) = wbm_jacobian(param.constraintLinkNames{i});
        dJcDq(6*(i-1)+1:6*i,:) = wbm_dJdq(param.constraintLinkNames{i});
    end

    %% control torque
    tau = param.tau(t);

    %% Contact forces computation
    JcMinv = Jc/M;
    JcMinvJct = JcMinv * Jc';

    tauDamp = -param.dampingCoeff*dqj;

    fc = (JcMinvJct)\(JcMinv*(h-[tau+tauDamp;zeros(6,1)])-dJcDq);

    % need to apply the inverse of the {}^w R_b (R_b) to the angular velocity omega_W
    % expressed in the world frame to the obtain angular velocity in body frame omega_b.
    % This is then used in the quaternion derivative computation.

    b_R_w   = w_R_b';
    omega_b = b_R_w*omega_W;
    dqt_b   = quaternionDerivative(omega_b, qt_b);%,param.QuaternionDerivativeParam);

    dx        = [dx_b;dqt_b;dqj];
    dv        = M\(Jc'*fc + [tau+tauDamp; zeros(6,1)]-h);
    dchi      = [dx;dv];
    kinEnergy = 0.5*v'*M*v;
    %dchi     = zeros(size(dchi));
end
