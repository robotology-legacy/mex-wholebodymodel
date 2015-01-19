%clear all
%close all

addpath('./../build');
%addpath('./icub_stls');
addpath('./worker_functions');
addpath('./experiment_results');

R = eye(3);
% 
% R = [ cos(-0.5*pi)      0    sin(-0.5*pi);
%                  0      1               0; 
%      -sin(-0.5*pi)     0     cos(-0.5*pi)];
%   
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

params.qjInit = zeros(size(params.qjInit));

wholeBodyModel('update-state',params.qjInit,zeros(25,1),zeros(6,1));

%disp('************');
[qj,T_baseInit,qjDot,vb] = wholeBodyModel('get-state');
%disp('************');
T_baseInit'

%disp(' ');
%disp('++++++++++++');
pos_Root =  wholeBodyModel('forward-kinematics',qj,'root_link');
%disp(' ');
%disp('++++++++++++');

disp('=======================================l_sole==========');
pos_RootD = pos_Root(1:3)'
rot_RootD = pos_Root(4:end)'
disp('=======================================l_sole==========');

%% resetting the frame
disp('Resetting base frame link');
disp('------------------------------------------------');
wholeBodyModel('set-world-frame','r_sole', reshape(R,[],1), [0;0;0]);

params.gJInit = zeros(size(params.qjInit));
wholeBodyModel('update-state',params.qjInit,zeros(25,1),zeros(6,1));

%disp('************');
[qj,T_baseInit,qjDot,vb] = wholeBodyModel('get-state');
%disp('************');

T_baseInit'

%disp(' ');
%disp('++++++++++++');
pos_Root =  wholeBodyModel('forward-kinematics',qj,'root_link');
%disp(' ');
%disp('++++++++++++');
disp('=======================================r_sole==========');
pos_RootD = pos_Root(1:3)'
rot_RootD = pos_Root(4:end)'
disp('=======================================r_sole==========');

%% resetting the frame
disp('Resetting world frame rototranslation from frame link');

R = [ cos(0.5*pi)   -sin(0.5*pi)        0;
     sin(0.5*pi)    cos(0.5*pi)        0; 
              0               0        1];
 
           
theta = pi/2;
R = [ cos(theta)   -sin(theta)        0;
      sin(theta)    cos(theta)        0; 
               0               0        1];
  
wholeBodyModel('set-world-frame','l_sole', reshape(R',[],1), [0;0;0]);

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

T_baseInit'
%disp('++++++++++++');
pos_Root =  wholeBodyModel('forward-kinematics',qj,'root_link');
%disp('++++++++++++');

disp('=======================================R_soleRot==========');
pos_RootD = pos_Root(1:3)'
rot_RootD = pos_Root(4:end)'
disp('=======================================R_soleRot==========');
