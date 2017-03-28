function  desired_x_dx_ddx_CoM = trajectoryGenerator(xCoMInit,t,CONFIG)
%TRAJECTORYGENERATOR generates a desired CoM trajectory. The default trajectory
%                    is a sinusoid in the Y direction.
%
% desired_x_dx_ddx_CoM = TRAJECTORYGENERATOR(xCoMInit,t,CONFIG) takes as an 
% input the initial CoM position, xCoMInit, the current time t and the structure
% CONFIG which contains the user-defined parameters.
% The output desired_x_dx_ddx_CoM is a matrix [3x3] which contains the CoM
% reference acceleration, velocity and position.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% Config parameters
feet_on_ground             = CONFIG.feet_on_ground;

% Initial parameters
directionOfOscillation     = [0;0;0];
referenceParams            = [0.0 0.0];   %referenceParams(1) = amplitude of ascillations in meters
                                          %referenceParams(2) = frequency of ascillations in Hertz
                                          
noOscillationTime          = 0;           % If params.demo_movements = 1, the variable noOscillationTime is the time, in seconds,
                                          % that the robot waits before starting the left-and-right
smoothingTime              = 1;
alpha                      = (t-noOscillationTime)/(smoothingTime);

%% Trajectory definition
if  strcmp(CONFIG.demo_type,'movements')   
    if  sum(feet_on_ground) == 2       
        directionOfOscillation = [0;1;0];
        referenceParams        = [0.025 0.25];
    else    
        directionOfOscillation = [0;1;0];
        referenceParams        = [0.01 0.2];
    end
end

%% Trajectory generation
frequency  = referenceParams(2);

if t >= noOscillationTime
    
    amplitude  = referenceParams(1);
else
    amplitude  = 0;
end
    
xCoMDes   =  xCoMInit + (1-exp(-alpha))*amplitude*sin(2*pi*frequency*t)*directionOfOscillation;
dxCoMDes  =             (1-exp(-alpha))*amplitude*2*pi*frequency*cos(2*pi*frequency*t)*directionOfOscillation + ...
                        (1/smoothingTime)*exp(-alpha)*amplitude*sin(2*pi*frequency*t)*directionOfOscillation;
                             
ddxCoMDes =           - (1-exp(-alpha))*amplitude*(2*pi*frequency)^2*sin(2*pi*frequency*t)*directionOfOscillation + ...
                        (1/smoothingTime)*exp(-alpha)*amplitude*2*pi*frequency*cos(2*pi*frequency*t)*directionOfOscillation + ...
                      - (1/smoothingTime)^2*exp(-alpha)*amplitude*sin(2*pi*frequency*t)*directionOfOscillation + ...
                        (1/smoothingTime)*exp(-alpha)*amplitude*2*pi*frequency*cos(2*pi*frequency*t)*directionOfOscillation;

% CoM references
desired_x_dx_ddx_CoM = [xCoMDes dxCoMDes ddxCoMDes];
   
end


