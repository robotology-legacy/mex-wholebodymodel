function  desired_x_dx_ddx_CoM   = generTraj (xCom0, t, referenceParams, directionOfOscillation, noOscillationTime)

%this is the function that generates the desired trajectory of the center
%of mass and the desired velocities and accelerations.

if t > noOscillationTime
    
    A = referenceParams(1);
    
else
    
    A = 0;

end

 f = referenceParams(2);

 xcomDes    =  xCom0 + A*sin(2*pi*f*t)*directionOfOscillation;
%xcomDes(1) =  xcomDes(1) + 0.01;
 
 xDcomDes   =  A*2*pi*f*cos(2*pi*f*t)*directionOfOscillation;
 
 xDDcomDes  = -A*(2*pi*f)^2*sin(2*pi*f*t)*directionOfOscillation;

 desired_x_dx_ddx_CoM = [xcomDes xDcomDes xDDcomDes];
 
end
