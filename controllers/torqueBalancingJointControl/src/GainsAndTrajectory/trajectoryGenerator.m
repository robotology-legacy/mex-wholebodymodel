function  desired_x_dx_ddx_CoM = trajectoryGenerator(xCoMInit,t,CONFIG)
%TRAJECTORYGENERATOR generates a desired CoM trajectory. The default trajectory 
%                    is a sinusoid in the Y direction.
%
%   desired_x_dx_ddx_CoM = GENERTRAJ(xCoMInit,t,config) takes as an input 
%   the initial CoM position, XCOMINIT, the current time T and the structure
%   CONFIG which contains all the user-defined parameters.
%
%   The output DESIRED_X_DX_DDX_COM is a matrix [3x3] which contains the
%   reference acceleration, velocity and position.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% Config parameters
feet_on_ground             = CONFIG.feet_on_ground;
demo_movements             = CONFIG.demo_movements;

% Initial parameters
directionOfOscillation     = [0;0;0];
referenceParams            = [0.0 0.0];   %referenceParams(1) = amplitude of ascillations in meters 
                                          %referenceParams(2) = frequency of ascillations in Hertz

noOscillationTime          = 0;           % If params.demo_movements = 1, the variable noOscillationTime is the time, in seconds, 
                                          % that the robot waits before starting the left-and-right
                                          
%% Trajectory definition
if  demo_movements == 1
    
if  sum(feet_on_ground) == 2
   
    directionOfOscillation = [0;1;0];
    referenceParams        = [0.035 0.35];     
else
        
    directionOfOscillation = [0;1;0];
    referenceParams        = [0.015 0.15];    
end
end

%% Trajectory generation
frequency  = referenceParams(2);

if t >= noOscillationTime
    
Amplitude  = referenceParams(1);    
else
Amplitude  = 0;
end
 
xCoMDes    =  xCoMInit + Amplitude*sin(2*pi*frequency*t)*directionOfOscillation;
dxCoMDes   =             Amplitude*2*pi*frequency*cos(2*pi*frequency*t)*directionOfOscillation;
ddxCoMDes  =           - Amplitude*(2*pi*frequency)^2*sin(2*pi*frequency*t)*directionOfOscillation;

desired_x_dx_ddx_CoM = [xCoMDes dxCoMDes ddxCoMDes]; 

end
