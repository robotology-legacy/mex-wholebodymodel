function  desired_x_dx_ddx_CoM   = generTraj_js(xCoM_ini, t, referenceParams, directionOfOscillation, noOscillationTime)
%%  generTraj_js 
% Generates a desired trajectory for robot's global CoM. It can be applied
% in every direction (X,Y,Z)
% Output
% 
% desired_x_dx_ddx_CoM [3x3]    it is a matrix wich contains the desired
%                               CoM position, velocity and acceleration

%% Time before the oscillation starts
if t >= noOscillationTime
    
    Amp = referenceParams(1);
    
else
    
    Amp = 0;

end

%% Trajectory generator
 freq = referenceParams(2);

 xCoMDes    =  xCoM_ini + Amp*sin(2*pi*freq*t)*directionOfOscillation;
 
 dxCoMDes   =  Amp*2*pi*freq*cos(2*pi*freq*t)*directionOfOscillation;
 
 ddxCoMDes  = -Amp*(2*pi*freq)^2*sin(2*pi*freq*t)*directionOfOscillation;


 desired_x_dx_ddx_CoM = [xCoMDes dxCoMDes ddxCoMDes];
 
end
