function [tau_js]   =  controllerFCN (LEFT_RIGHT_FOOT_IN_CONTACT, DOF,...
                                       q, v, M, g, CNu, Jc, JcDv,impedances, dampings, dq_inv, ddq_inv, q_inv)
% controllerFCN 
% computes the desired control torques
% at joints. 
 ROBOT_DOF       = DOF;
 constraints     = LEFT_RIGHT_FOOT_IN_CONTACT;
%PINV_TOL        = 1e-8;
 reg             = 0.0000001;

%% parameters
% Mb           = M(1:6,1:6);
Mj             = M(7:end,7:end);

S              = [ zeros(6,ROBOT_DOF);
                    eye(ROBOT_DOF,ROBOT_DOF)];

dq             = v(7:end);

%% terms from the constraint equation
qTilde          = q-q_inv;
dqTilde         = dq-dq_inv;

Jct             = Jc.';
JcMinv          = Jc/M;
JcMinvJct       = JcMinv*transpose(Jc);

%% joints space controller 
Lambda  =  (JcMinvJct\JcMinv);

g_new   =  (-Jct*Lambda)*g;
CNu_new =  (-Jct*Lambda)*CNu;
v_new   =    Jct*(JcMinvJct\JcDv);

B       = (eye(31) - Jct*Lambda)*S;

B_bar   = B(7:end,:);

CNu_bar = CNu_new(7:end);
g_bar   = g_new(7:end);
v_bar   = v_new(7:end);

if     constraints == 1

tau_js     = B_bar\(CNu_bar + g_bar + v_bar + Mj*ddq_inv -diag(impedances)*qTilde -diag(dampings)*dqTilde);

elseif constraints == 2 

 pinvB_bar  = B_bar'/(B_bar*B_bar' + reg*eye(size(B_bar,1)));  
%pinvB_bar  = pinv(B_bar,PINV_TOL);

tau_js      = pinvB_bar*(CNu_bar + g_bar + v_bar + Mj*ddq_inv -diag(impedances)*qTilde -diag(dampings)*dqTilde);

end

end
