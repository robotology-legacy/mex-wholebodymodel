function  desired_x_dx_ddx_CoM   = generTraj (xCom0, t, referenceParams, directionOfOscillation, noOscillationTime)
% generTraj generates a desired trajectory for CoM position

%% time before the oscillation starts
if t >= noOscillationTime
    
    A = referenceParams(1);
    
else
    
    A = 0;

end

%% trajectory generator
 f = referenceParams(2);

 xcomDes    =  xCom0 + A*sin(2*pi*f*t)*directionOfOscillation;
 
 xDcomDes   =  A*2*pi*f*cos(2*pi*f*t)*directionOfOscillation;
 
 xDDcomDes  = -A*(2*pi*f)^2*sin(2*pi*f*t)*directionOfOscillation;


 desired_x_dx_ddx_CoM = [xcomDes xDcomDes xDDcomDes];
 
end
