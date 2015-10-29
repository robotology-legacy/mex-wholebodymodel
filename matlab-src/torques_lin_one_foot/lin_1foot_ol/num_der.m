function tau_der = num_der(j,lparam)

qjDes    = lparam.qjDes;
delta    = lparam.delta;
n_joints = lparam.n_joints;
n_base   = lparam.n_base;

%% Add a small delta in joints variables
qj2    = qjDes;
qj2(j) = qj2(j) + delta;

wbm_updateState(qj2, zeros(n_joints,1), zeros(n_base,1));

[~, T_b2, ~, ~] = wbm_getState();
[pos2, rot2]    = frame2posrot(T_b2);

tau_delta2 = add_delta(rot2.', pos2, qj2, lparam);

wbm_updateState(qjDes, zeros(n_joints,1), zeros(n_base,1));

%% Subtract a small delta in joints variables
qj3    = qjDes;
qj3(j) = qj3(j) - delta;

wbm_updateState(qj3, zeros(n_joints,1), zeros(n_base,1));

[~, T_b3, ~, ~] = wbm_getState();
[pos3, rot3]    = frame2posrot(T_b3);

tau_delta3 = add_delta(rot3.', pos3, qj3, lparam);

wbm_updateState(qjDes, zeros(n_joints,1), zeros(n_base,1));

%% Numerical derivative
tau_der = (tau_delta2 - tau_delta3)/(2*delta);

end