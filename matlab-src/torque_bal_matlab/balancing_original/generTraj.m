function  desired_x_dx_ddx_CoM = generTraj (xCom0,t,trajParam)
%% generTraj 
% Generates a sine trajectory for robot's global CoM. It could be applied
% in every direction
if t > trajParam.noOscillationTime
    
    A = trajParam.referenceParams(1);
    
else
    
    A = 0;

end

 f = trajParam.referenceParams(2);

 xcomDes    =  xCom0 + A*sin(2*pi*f*t)*trajParam.directionOfOscillation;
 dxcomDes   =  A*2*pi*f*cos(2*pi*f*t)*trajParam.directionOfOscillation;
 ddxcomDes  = -A*(2*pi*f)^2*sin(2*pi*f*t)*trajParam.directionOfOscillation;

 desired_x_dx_ddx_CoM = [xcomDes dxcomDes ddxcomDes];
 
end