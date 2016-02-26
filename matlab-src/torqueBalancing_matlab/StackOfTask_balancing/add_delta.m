function Jw_delta = add_delta(rotation, position, qjoints, param)

ndof = param.ndof;

Jc          = wbm_jacobian(rotation, position, qjoints,'l_sole');

Jw_b        = zeros(6,6);
Jw_j        = zeros(6,ndof);

for ii = 1:6
    
Nu_base         = zeros(6,1);
Nu_base(ii)     = 1;

Jw_b(:,ii)      = wbm_centroidalMomentum(rotation, position, qjoints, zeros(ndof,1), Nu_base);
 
end

for ii = 1:ndof

dqj_Jw          = zeros(ndof,1);
dqj_Jw(ii)      = 1;

Jw_j(:,ii)      = wbm_centroidalMomentum(rotation, position, qjoints, dqj_Jw, zeros(6,1));

end

Jw_delta        = (Jw_j -Jw_b*(eye(6)/Jc(1:6,1:6))*Jc(1:6,7:end));

end















