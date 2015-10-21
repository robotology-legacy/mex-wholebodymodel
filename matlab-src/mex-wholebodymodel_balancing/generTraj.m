function  desired_x_dx_ddx_CoM   = generTraj (xCom0, t, referenceParams, directionOfOscillation, noOscillationTime)

%this is the function that generates the desired trajectory of the center
%of mass and the desired velocities and accelerations.

if t > noOscillationTime
    
    A = referenceParams(1)/2;
    
else
    
    A = 0;

end

 f = referenceParams(2);

 xcomDes    =  xCom0 + 0*A*[0;sin(2*pi*f*t);-1+cos(2*pi*f*t)];
%xcomDes(1) =  xcomDes(1) + 0.01;
 
 xDcomDes   =  0*2*pi*f*A*[0;cos(2*pi*f*t);-sin(2*pi*f*t)];
 
 xDDcomDes  = 0*(2*pi*f)^2*A*[0;-sin(2*pi*f*t);-cos(2*pi*f*t)];

 desired_x_dx_ddx_CoM = [xcomDes xDcomDes xDDcomDes];
 
end
