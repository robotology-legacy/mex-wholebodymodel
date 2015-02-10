function [ dchi ] = forwardDynamics( t,chi,param )
%FORWARDDYNAMICS Forward dynamics of the wholeBodyModel
%
%   This is the forward dynamics of the model loaded in the
%   wholeBodyInterface from the URDF description

%% extraction of state
ndof = param.ndof;

%x_b = chi(1:3,:);
qt_b = chi(4:7,:);
qj = chi(8:ndof+7,:);
%x = [x_b;qt_b;qj];

dx_b = chi(ndof+8:ndof+10,:);
omega_b = chi(ndof+11:ndof+13,:);
dqj = chi(ndof+14:2*ndof+13,:);

%dx = [dqj;dx_b;omega_b];
%v = dx;
%chi = [x;dx];

%% MexWholeBodyModel calls
wbm_updateState(qj,dqj,[dx_b;omega_b]);
M = wbm_massMatrix();
h = wbm_generalisedBiasForces();
%g = wbm_modelGeneralisedForces(qj,zeros(25,1),zeros(6,1)); 
%H = wbm_centroidalMomentum();

%% Building up contraints jacobian and djdq
numConstraints = length(param.constraintLinkNames);
Jc = zeros(6*numConstraints,6+ndof);
dJcDq = zeros(6*numConstraints,1);
for i=1:numConstraints
    Jc(6*(i-1)+1:6*i,:) = wbm_jacobian(param.constraintLinkNames{i});
    dJcDq(6*(i-1)+1:6*i,:) = wbm_djdq(param.constraintLinkNames{i});
end


%% control torque
tau = param.tau(t);


%% Contact forces computation
JcMinv = Jc/M;
JcMinvJct = JcMinv * Jc';   

tauDamp = -param.dampingCoeff*qj;
  
fc = (JcMinvJct)\(JcMinv*(h-[tau+tauDamp;zeros(6,1)])-dJcDq);
dqt_b = quaternionDerivative(omega_b, qt_b);%,param.QuaternionDerivativeParam);

%dx = [dqj;dx_b;dqt_b];
dx = [dx_b;dqt_b;dqj];
dv = M\(Jc'*fc + [tau+tauDamp; zeros(6,1)]-h);

dchi = [dx;dv];  

end

