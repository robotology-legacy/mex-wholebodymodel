function  [dq_inv, ddq_inv, q_inv, ecom] = interpolation(t,params, xcom)
% interpolation interpolates the inverse kinematic trajectory previously generated
tt = params.t_kin;

% find in tt the step which is before t
logic_t = tt<t;
index   = sum(logic_t);
index_n = index+1;

if tt(index_n) == t
    
    q_inv   = params.joints_traj.qj(:,index_n);
    dq_inv  = params.joints_traj.dqj(:,index_n);
    ddq_inv = params.joints_traj.ddqj(:,index_n);
    des_com = params.joints_traj.traj(1:3,index_n);

else
  
  % if t is between tt(index) and tt(index_n), it will be interpolated
  % using a line
  x_delta    = tt(index_n)-tt(index);
        
  y1_delta   = params.joints_traj.qj(:,index_n)   - params.joints_traj.qj(:,index);
  y2_delta   = params.joints_traj.dqj(:,index_n)  - params.joints_traj.dqj(:,index);
  y3_delta   = params.joints_traj.ddqj(:,index_n) - params.joints_traj.ddqj(:,index);
  
  com_delta  = params.joints_traj.traj(1:3,index_n) - params.joints_traj.traj(1:3,index);
  
  % tangent calculation
  m1         = y1_delta./x_delta;
  m2         = y2_delta./x_delta;
  m3         = y3_delta./x_delta;
  m_com      = com_delta./x_delta;
  
  % desired value interpolation
  x0_delta   = t - tt(index);
  
  q_inv   = params.joints_traj.qj(:,index)   + m1*x0_delta;
  dq_inv  = params.joints_traj.dqj(:,index)  + m2*x0_delta;
  ddq_inv = params.joints_traj.ddqj(:,index) + m3*x0_delta;
  des_com = params.joints_traj.traj(1:3,index) + m_com*x0_delta;

end
  
% position error at CoM
ecom = abs(des_com-xcom);

end