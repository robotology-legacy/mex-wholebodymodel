function [dikin, visual_kin_param] = ikin_function( t, ikin, param )
%% ikin_function
%  Calculates the inverse kinematics from a desired CoM
%  trajectory to the corresponding joints pos, vel, acc, with respect of
%  constraints at feet.
%  The outputs are:
%
%  dikin [ndof+6 x 1]   which is the vector that will be integrated
%
%  visual_kin_param     which contains parameters for visualization

waitbar(t/param.tEnd,param.wait)
  
%% State definition
%  ikin(1:7) is the quaternion associated with the desired trajectory and ikin(8:end) is the joints desired position
param.kin_total = ikin;

%% Desired oscillation at CoM
directionOfOscillation            = [0; 0; 0];
referenceParams                   = [0  0]; 
 
if (param.demo_movements == 1)
        
        directionOfOscillation = param.direction;
        referenceParams        = param.reference;   
    
end

%% CoM trajectory generator
desired_x_dx_ddx_CoM = generTraj_js(param.CoM_ini(1:3),t,referenceParams,directionOfOscillation,param.noOscillationTime);

%% Desired acceleration and velocity at joints
[ddqjDes, dikin, traj_obt, Delta] = ikin_generator(desired_x_dx_ddx_CoM, param);

%% errors visualization and stored trajectory
traj_des = [desired_x_dx_ddx_CoM(:,1); desired_x_dx_ddx_CoM(:,2); desired_x_dx_ddx_CoM(:,3)];
traj     = [traj_des; traj_obt];

visual_kin_param.traj           = traj;
visual_kin_param.Delta          = Delta;
visual_kin_param.ddqjDes        = ddqjDes;

end
