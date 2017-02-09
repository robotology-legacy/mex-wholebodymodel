function [d_dxi_t] = compute_ddxi_ref(ddqj,STATE,CONFIG,xi,ELASTICITY,dxi)

ndof        = CONFIG.ndof;
dqj         = STATE.dqj;

%% partial derivative w.r.t. position
mode        = 'position';
diff_dxi_qj = zeros(ndof,ndof);

for i = 1:ndof  
    diff_dxi_i       = numericalDerivative(STATE,CONFIG,xi,ELASTICITY,mode,i);
    diff_dxi_qj(:,i) = diff_dxi_i;
end

%% partial derivative w.r.t. velocity
mode         = 'velocity';
diff_dxi_dqj = zeros(ndof,ndof);

for i = 1:ndof  
    diff_dxi_i        = numericalDerivative(STATE,CONFIG,xi,ELASTICITY,mode,i);
    diff_dxi_dqj(:,i) = diff_dxi_i;
end

%% partial derivative w.r.t. motor position
mode        = 'motorPos';
diff_dxi_xi = zeros(ndof,ndof);

for i = 1:ndof  
    diff_dxi_i       = numericalDerivative(STATE,CONFIG,xi,ELASTICITY,mode,i);
    diff_dxi_xi(:,i) = diff_dxi_i;
end

%% total derivative
d_dxi_t = diff_dxi_xi*dxi + diff_dxi_qj*dqj + diff_dxi_xi*ddqj;

end
