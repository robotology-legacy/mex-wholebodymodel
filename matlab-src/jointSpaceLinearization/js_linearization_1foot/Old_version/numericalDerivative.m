function num_der = numericalDerivative(jj, linParam)
%% numericalDerivative
%  computes the numerical derivative of a part of joint accelerations equation
%  with respect of the jjth joint, using the quotient's formula. The output is:
%
%  num_der [ndof x 1] which is the vector containing the joint acceleration
%                     derivative with respect of the jjth DoF
%
%% Parameters definition
qjDes  = linParam.qjDes;
delta  = linParam.delta;
ndof   = linParam.ndof;

%% Add a small delta to joint position
qj_add     = qjDes;
qj_add(jj) = qj_add(jj) + delta;

% update the robot state
wbm_updateState(qj_add, zeros(ndof,1), zeros(6,1));

[Rb_add,position_add] = wbm_getWorldFrameFromFixedLink('l_sole',qj_add);
 
accParam_add          = jointAccCalculation(Rb_add, position_add, qj_add, linParam);

ddqj_add              = accParam_add.acc_steady;

%% Subtract a small delta to joint position
qj_sub     = qjDes;
qj_sub(jj) = qj_sub(jj) + delta;

% update the robot state
wbm_updateState(qj_sub, zeros(ndof,1), zeros(6,1));

[Rb_sub,position_sub] = wbm_getWorldFrameFromFixedLink('l_sole',qj_sub);
 
accParam_sub          = jointAccCalculation(Rb_sub, position_sub, qj_sub, linParam);

ddqj_sub              = accParam_sub.acc_steady;

%% Numerical derivative
num_der = (ddqj_add - ddqj_sub)/(2*delta);

end

