function Jw_der = num_der(j,param)

qjInit    = param.qjInit;
delta     = 0.00001;

%% Add a small delta in joints variables
qj_add    = qjInit;
qj_add(j) = qj_add(j) + delta;

[rot_add,pos_add]   = wbm_getWorldFrameFromFixedLink('l_sole',qj_add);

Jw_delta_add        = add_delta(rot_add, pos_add, qj_add, param);

%% Subtract a small delta in joints variables
qj_sub    = qjInit;
qj_sub(j) = qj_sub(j) - delta;

[rot_sub,pos_sub]   = wbm_getWorldFrameFromFixedLink('l_sole',qj_sub);

Jw_delta_sub        = add_delta(rot_sub, pos_sub, qj_sub, param);
 
%% Numerical derivative
Jw_der_matrix = (Jw_delta_add - Jw_delta_sub)/(2*delta);

Jw_der        = sum(Jw_der_matrix,2);

end