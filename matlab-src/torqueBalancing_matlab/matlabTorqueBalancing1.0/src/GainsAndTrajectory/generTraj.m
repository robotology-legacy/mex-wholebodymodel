function  desired_x_dx_ddx_CoM = generTraj(xCoMInit,t,trajectory)
%GENERTRAJ generates a desired CoM trajectory. The default trajectory is a
%          sine.
%   desired_x_dx_ddx_CoM = GENERTRAJ(xCoMInit,t,trajectory) takes as an
%   input the initial CoM position, xCoMInit, the current time t and the
%   structure trajectory which containts the amplitude, frequancy and
%   direction of oscillation. The output desired_x_dx_ddx_CoM is a matrix
%   [3x3] which contains the reference acceleration, velocity and position.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% initial parameters
if t >= trajectory.noOscillationTime
    
Amplitude = trajectory.referenceParams(1);    
else
Amplitude = 0;
end

frequency = trajectory.referenceParams(2);
 
%% TRAJECTORY GENERATION
xCoMDes    =  xCoMInit + Amplitude*sin(2*pi*frequency*t)*trajectory.directionOfOscillation;
dxCoMDes   =             Amplitude*2*pi*frequency*cos(2*pi*frequency*t)*trajectory.directionOfOscillation;
ddxCoMDes  =           - Amplitude*(2*pi*frequency)^2*sin(2*pi*frequency*t)*trajectory.directionOfOscillation;

desired_x_dx_ddx_CoM = [xCoMDes dxCoMDes ddxCoMDes]; 
end