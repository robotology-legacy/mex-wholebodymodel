clear all
close all

addpath('./../build');
addpath('./icub_stls');
addpath('./worker_functions');
addpath('./experiment_results');


R = [ cos(-0.5*pi)      0    sin(-0.5*pi);
                 0      1               0; 
     -sin(-0.5*pi)     0     cos(-0.5*pi)];
  
% R = [ cos(0.5*pi)   -sin(0.5*pi)        0;
%       sin(0.5*pi)    cos(0.5*pi)        0; 
%                0               0        1];
  

wholeBodyModel('model-initialise','icubGazeboSim');

wholeBodyModel('set-world-frame','l_sole', reshape(R,[],1), [0;0;0]);

params.torsoInit    = [-10.0  0.0   0.0]';
params.leftArmInit  = [ 19.7  29.7  0.0  44.9  0.0]';
params.rightArmInit = [ 19.7  29.7  0.0  44.9  0.0]';
params.leftLegInit  = [ 40.5   0.1  0.0 -18.5 -5.5 -0.1]';
params.rightLegInit = [ 40.5   0.1  0.0 -18.5 -5.5 -0.1]';

params.qjDotInit = zeros(25,1);
params.v_baseInit = zeros(6,1);

params.qjInit = pi*([params.torsoInit;params.leftArmInit;params.rightArmInit;params.leftLegInit;params.rightLegInit])/180;

wholeBodyModel('update-state',params.qjInit,zeros(25,1),zeros(6,1));

[qj,T_baseInit,qjDot,vb] = wholeBodyModel('get-state');

pos_CoM =  wholeBodyModel('forward-kinematics',qj,'com');
pos_CoM = pos_CoM(1:3)

%% resetting the frame
disp('Resetting base frame link');
wholeBodyModel('set-world-frame','r_sole', reshape(R,[],1), [0;0;0]);

params.torsoInit    = [-10.0  0.0   0.0]';
params.leftArmInit  = [ 19.7  29.7  0.0  44.9  0.0]';
params.rightArmInit = [ 19.7  29.7  0.0  44.9  0.0]';
params.leftLegInit  = [ 40.5   0.1  0.0 -18.5 -5.5 -0.1]';
params.rightLegInit = [ 40.5   0.1  0.0 -18.5 -5.5 -0.1]';

params.qjDotInit = zeros(25,1);
params.v_baseInit = zeros(6,1);

params.qjInit = pi*([params.torsoInit;params.leftArmInit;params.rightArmInit;params.leftLegInit;params.rightLegInit])/180;

wholeBodyModel('update-state',params.qjInit,zeros(25,1),zeros(6,1));

[qj,T_baseInit,qjDot,vb] = wholeBodyModel('get-state');

pos_CoM =  wholeBodyModel('forward-kinematics',qj,'com');
pos_CoM = pos_CoM(1:3)

%% resetting the frame
disp('Resetting world frame rototranslation from frame link');

R = [ cos(0.5*pi)   -sin(0.5*pi)        0;
      sin(0.5*pi)    cos(0.5*pi)        0; 
               0               0        1];
  
wholeBodyModel('set-world-frame','r_sole', reshape(R,[],1), [0;0;0]);

params.torsoInit    = [-10.0  0.0   0.0]';
params.leftArmInit  = [ 19.7  29.7  0.0  44.9  0.0]';
params.rightArmInit = [ 19.7  29.7  0.0  44.9  0.0]';
params.leftLegInit  = [ 40.5   0.1  0.0 -18.5 -5.5 -0.1]';
params.rightLegInit = [ 40.5   0.1  0.0 -18.5 -5.5 -0.1]';

params.qjDotInit = zeros(25,1);
params.v_baseInit = zeros(6,1);

params.qjInit = pi*([params.torsoInit;params.leftArmInit;params.rightArmInit;params.leftLegInit;params.rightLegInit])/180;

wholeBodyModel('update-state',params.qjInit,zeros(25,1),zeros(6,1));

[qj,T_baseInit,qjDot,vb] = wholeBodyModel('get-state');

pos_CoM =  wholeBodyModel('forward-kinematics',qj,'com');
pos_CoM = pos_CoM(1:3)