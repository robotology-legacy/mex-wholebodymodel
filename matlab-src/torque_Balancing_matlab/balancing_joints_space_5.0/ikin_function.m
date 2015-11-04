function [dikin, visual_kin_param ] = ikin_function( t, ikin, param )
% ikin_function
% calculates the inverse kinematics from a desired CoM
% trajectory to the corresponding joints pos, vel, acc, with respect of
% constraints at feet
waitbar(t/param.tEnd,param.wait)
  
%% state definition
% ikin(1:7) is the quaternion associated with the desired
% trajectory and ikin(8:end) is the joints desired position
param.kin_total = ikin;

%% desired oscillation at CoM
directionOfOscillation            = [0; 0; 0];
referenceParams                   = [0  0]; 
 
if (param.demo_movements == 1)
        
        directionOfOscillation = param.direction;
        referenceParams        = param.reference;   
    
end

%% com trajectory generator
desired_x_dx_ddx_CoM = generTraj (param.com_ini(1:3),t,referenceParams,directionOfOscillation,param.noOscillationTime);

%% desired acceleration and velocity at joints
[ddq_inv, dikin, traj_obt, delta] = ikin_generator (desired_x_dx_ddx_CoM, param);

%% errors visualization and stored trajectory
traj_des = [desired_x_dx_ddx_CoM(:,1); desired_x_dx_ddx_CoM(:,2); desired_x_dx_ddx_CoM(:,3)];
traj     = [traj_des; traj_obt];

visual_kin_param.traj           = traj;
visual_kin_param.delta          = delta;
visual_kin_param.ddq_inv        = ddq_inv;

end
