function CONFIG_updated = state_machine(t,CONFIG)

global force_feet state com_error diffState;
import WBM.utilities.rotm2quat;
previousState = state;

%% Two feet balancing
if state == 1
    
   CONFIG_updated = CONFIG;
   
   if CONFIG.left_right_yoga(1) == 1 
  
       CONFIG_updated.foot_selector = 3;
       
   elseif CONFIG.left_right_yoga(1) == 0

       CONFIG_updated.foot_selector = 9;       
   end
   
   t_treshold = 1;
   
   if t >= t_treshold
       state = 2;
   end
end

%% CoM transition to stance leg
if state == 2
   
   CONFIG_updated = CONFIG;
   
   if CONFIG.left_right_yoga(1) == 1 
      
       CONFIG_updated.xCoMRef(2) = -0.0381;
       CONFIG_updated.foot_selector = 3;
       
   elseif CONFIG.left_right_yoga(1) == 0
       
       CONFIG_updated.xCoMRef(2) = -0.0981;
       CONFIG_updated.foot_selector = 9;
   end

   treshold  = 151;
   eps       = 0.001;

   if  t > 2%force_feet > treshold && norm(com_error) < eps
       state = 3;
   end
end

%% One foot balancing
if state == 3
   
   CONFIG_updated = CONFIG;
   CONFIG_updated.foot_selector = 3;
   leftArmInit    = [ -20  30  0  45  0]';
   rightArmInit   = [ -20  30  0  45  0]';
   torsoInit      = [ -10   0  0]';

   if CONFIG.left_right_yoga(1) == 1 
      
       CONFIG_updated.feet_on_ground = [1,0];  
       CONFIG_updated.constraintLinkNames = {'l_sole'};
       % initial conditions for the robot standing on the left foot
       leftLegInit  = [  25.5   15   0  -18.5  -5.5  0]';
       rightLegInit = [  25.5    5   0  -40    -5.5  0]';
       
   elseif CONFIG.left_right_yoga(1) == 0
       
       CONFIG_updated.feet_on_ground = [0,1];  
       CONFIG_updated.constraintLinkNames = {'r_sole'};
       % initial conditions for the robot standing on the right foot
       leftLegInit  = [  25.5    5   0  -40    -5.5  0]';
       rightLegInit = [  25.5   15   0  -18.5  -5.5  0]';
   end

   CONFIG_updated.numConstraints = length(CONFIG_updated.constraintLinkNames);
   ndof                          = CONFIG.ndof;
   % joints configuration [rad]
   CONFIG_updated.qjInit = [torsoInit;leftArmInit;rightArmInit;leftLegInit;rightLegInit]*(pi/180);
   
   dqjInit               = zeros(ndof,1);
   dx_bInit              = zeros(3,1);
   w_omega_bInit         = zeros(3,1);

   % fixing the world reference frame w.r.t. the foot on ground position
   [x_bInit,w_R_bInit] = wbm_getWorldFrameFromFixLnk(CONFIG_updated.constraintLinkNames{1},CONFIG_updated.qjInit);
   qt_bInit            = rotm2quat(w_R_bInit);
   % initial state (floating base + joints)
   chi_robotInit       = [x_bInit; qt_bInit; CONFIG_updated.qjInit; dx_bInit; w_omega_bInit; dqjInit];
  
   CONFIG_updated.gainsInit         = gains(CONFIG_updated);
   CONFIG_updated.initState         = robotState(chi_robotInit,CONFIG_updated);
   CONFIG_updated.initDynamics      = robotDynamics(CONFIG_updated.initState,CONFIG_updated);
   CONFIG_updated.initForKinematics = robotForKinematics(CONFIG_updated.initState,CONFIG_updated.initDynamics);
   
   if force_feet > 1000000
       state = 4;
   end
end

%% Yoga ++ demo

%% Update diffState
diffState = state - previousState -1;

end