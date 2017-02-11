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
demo_movements             = CONFIG.demo_movements;

% Initial parameters
directionOfOscillation     = [0;0;0];
referenceParams            = [0.0 0.0];   %referenceParams(1) = amplitude of ascillations in meters
                                          %referenceParams(2) = frequency of ascillations in Hertz

% NB: IF THE INTEGRATION IS PERFORMED USING ODE15S THIS PARAMETER SHOULD REMAIN ZERO                                           
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

if size(t,1)==1 && size(t,2)==1
    
    if t >= noOscillationTime
    
        amplitude  = referenceParams(1);
    else
        amplitude  = 0;
    end

    xCoMDes    =  xCoMInit + amplitude*sin(2*pi*frequency*t)*directionOfOscillation;
    dxCoMDes   =             amplitude*2*pi*frequency*cos(2*pi*frequency*t)*directionOfOscillation;
    ddxCoMDes  =           - amplitude*(2*pi*frequency)^2*sin(2*pi*frequency*t)*directionOfOscillation;

    desired_x_dx_ddx_CoM = [xCoMDes dxCoMDes ddxCoMDes];
else
    
    % in case t is a vector
    desired_x_dx_ddx_CoM = zeros(3,3,length(t));

    for k = 1:length(t)
        if t(k) >= noOscillationTime
    
            amplitude  = referenceParams(1);
         else
            amplitude  = 0;
        end

        xCoMDes    =  xCoMInit + amplitude*sin(2*pi*frequency*t(k))*directionOfOscillation;
        dxCoMDes   =             amplitude*2*pi*frequency*cos(2*pi*frequency*t(k))*directionOfOscillation;
        ddxCoMDes  =           - amplitude*(2*pi*frequency)^2*sin(2*pi*frequency*t(k))*directionOfOscillation;

        desired_x_dx_ddx_CoM(:,:,k) = [xCoMDes dxCoMDes ddxCoMDes]; 
    end
end


