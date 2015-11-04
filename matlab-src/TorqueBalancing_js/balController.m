function  [tau_js, cVisualParam] = balController(t,param,controlParam)
% balController 
% is the main function for the balancing controller. it contains the definition of every
% parameters necessary for the controller and calls the other functions
% whose are organized as in simulink code
%% initial variables
q             = controlParam.qj;
v             = controlParam.v;

M             = controlParam.M;
g             = controlParam.g;
CNu           = controlParam.CNu;

Jc            = controlParam.Jc;
JcDv          = controlParam.dJcdv;

xcom          = controlParam.com(1:3);

%% gains definition
[impedances, dampings] = gains (param.ndof, param.numConstraints);

%% interpolation of the trajectory generated with inverse kinematics
[dq_inv, ddq_inv, q_inv, ecom] = interpolation (t, param, xcom);

%% balancing joints space controller
tau_js               =  controllerFCN (param.numConstraints, param.ndof, ...
                                       q, v, M, g, CNu, Jc, JcDv,impedances, dampings, dq_inv, ddq_inv, q_inv);

%%  parameters for the visualization
cVisualParam.e_com   = ecom;
cVisualParam.q_inv   = q_inv;
cVisualParam.dq_inv  = dq_inv;
cVisualParam.ddq_inv = ddq_inv;

end
