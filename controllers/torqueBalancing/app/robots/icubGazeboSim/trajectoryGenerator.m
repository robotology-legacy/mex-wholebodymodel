function  desired_x_dx_ddx_CoM = trajectoryGenerator(xCoMInit,t,CONFIG)
%TRAJECTORYGENERATOR generates a desired CoM trajectory. The default trajectory
%                    is a sinusoid in the robot lateral direction.
%
% Format: desired_x_dx_ddx_CoM = TRAJECTORYGENERATOR(xCoMInit,t,CONFIG)
%
% Inputs:  - xCoMInit [3 x 1] initial CoM position; 
%          - t  current integration time;
%          - CONFIG it is the structure containing all user-defined parameters. 
%
% Output:  - desired_x_dx_ddx_CoM [3 x 3] matrix whose columns are the CoM
%            reference positions, velocities and accelerations.
%         
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
% config parameters
directionOfOscillation     = CONFIG.directionOfOscillation;
amplitudeOfOscillation     = CONFIG.amplitudeOfOscillation;
frequencyOfOscillation     = CONFIG.frequencyOfOscillation;
noOscillationTime          = CONFIG.noOscillationTime;           
% initial parameters
referenceParams            = [0.0 0.0];   %referenceParams(1) = amplitude of ascillations in meters
                                          %referenceParams(2) = frequency of ascillations in Hertz
          
% it is necessary to initially smooth the reference trajectory to avoid
% discontinuities with robot initial conditions
smoothingTime              = 1;
alpha                      = (t-noOscillationTime)/(smoothingTime);

%% Trajectory definition
if  strcmp(CONFIG.demo_type,'movements')   
    % update reference trajectory    
    referenceParams        = [amplitudeOfOscillation frequencyOfOscillation];
end

%% Trajectory generation
% frequency of oscillation
frequency      = referenceParams(2);
% amplitude of oscillation
if t >= noOscillationTime 
    amplitude  = referenceParams(1);
else
    amplitude  = 0;
end
 
%% CoM REFERENCES
% position
xCoMDes   =  xCoMInit + (1-exp(-alpha))*amplitude*sin(2*pi*frequency*t)*directionOfOscillation;
% velocity
dxCoMDes  =             (1-exp(-alpha))*amplitude*2*pi*frequency*cos(2*pi*frequency*t)*directionOfOscillation + ...
                        (1/smoothingTime)*exp(-alpha)*amplitude*sin(2*pi*frequency*t)*directionOfOscillation;
% acceleration                             
ddxCoMDes =           - (1-exp(-alpha))*amplitude*(2*pi*frequency)^2*sin(2*pi*frequency*t)*directionOfOscillation + ...
                        (1/smoothingTime)*exp(-alpha)*amplitude*2*pi*frequency*cos(2*pi*frequency*t)*directionOfOscillation + ...
                      - (1/smoothingTime)^2*exp(-alpha)*amplitude*sin(2*pi*frequency*t)*directionOfOscillation + ...
                        (1/smoothingTime)*exp(-alpha)*amplitude*2*pi*frequency*cos(2*pi*frequency*t)*directionOfOscillation;

% CoM reference matrix
desired_x_dx_ddx_CoM = [xCoMDes dxCoMDes ddxCoMDes];
   
end


