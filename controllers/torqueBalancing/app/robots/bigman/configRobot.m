function [ndof,qjInit,footSize] = configRobot(CONFIG)
%CONFIGROBOT setup the initial configuration of the robot.
%
% [ndof,qjInit,footSize] = CONFIGROBOT(CONFIG) takes as an input the structure 
% CONFIG, which contains all the robot configuration parameters. The output 
% are the number of degrees of freedom, the initial joints positions and
% the robot's feet size. 
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, February 2017
%

% ------------Initialization----------------
%% Config parameters
% number of dofs
ndof = 25;

% feet size
footSize  = [-0.16 0.16;     % xMin, xMax
             -0.075 0.075];  % yMin, yMax
         
%% Initial joints position [deg]
leftArmInit  = [ 0  8  0  0  0]';
rightArmInit = [ 0 -8  0  0  0]';
torsoInit    = [ 0   0  0]';

if sum(CONFIG.feet_on_ground) == 2
    
    % initial conditions for balancing on two feet
    leftLegInit  = [ 0   0   0   0   0  0 ]';
    rightLegInit = [ 0   0   0   0   0  0 ]';
    
elseif CONFIG.feet_on_ground(1) == 1 && CONFIG.feet_on_ground(2) == 0
    
    % initial conditions for the robot standing on the left foot
    leftLegInit  = [  25.5   15   0  -18.5  -5.5  0]';
    rightLegInit = [  25.5    5   0  -40    -5.5  0]';
    
elseif CONFIG.feet_on_ground(1) == 0 && CONFIG.feet_on_ground(2) == 1
    
    % initial conditions for the robot standing on the right foot
    leftLegInit  = [  25.5    5   0  -40    -5.5  0]';
    rightLegInit = [  25.5   15   0  -18.5  -5.5  0]';
end

% joints configuration [rad]
qjInit = [torsoInit;leftArmInit;rightArmInit;leftLegInit;rightLegInit]*(pi/180);

end

