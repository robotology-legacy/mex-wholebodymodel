clear all
close all
clc

% This is the main program for integrating the forward dynamics of the
% robot in matlab

%% setup path
addpath('./../whole_body_model_functions/');
addpath('./../../../../build/');
addpath('./../worker_functions');
addpath('./../');

%% initialise mexWholeBodyModel
 wbm_modelInitialise('icubGazeboSim');
 wbm_setWorldLink('l_sole',eye(3),[0 0 0]',[0,0,-9.81]');

%% params for controller
% the user can set this parameters depending on what he wants to do with
% the robot
 params.demo_left_and_right      =  1;                                      %either 0 or 1; only for two feet on the ground
 params.QP_solver                =  0;                                      %either 0 or 1
% this is  for the new angular momentum controller
 params.HDes_solver              =  0;                                      %either 0 or 1 

%% setup params
% this is assuming a 25DoF iCub
 params.ndof         = 25;

% initial conditions
 params.torsoInit    = [-10.0  0.0   0.0]';                   
 params.leftArmInit  = [ -19.7  29.7  0.0  44.9 0.0]';          
 params.rightArmInit = [ -19.7  29.7  0.0  44.9 0.0]';         
 params.leftLegInit  = [ 25.5   0.1  0.0 -18.5 -5.5 -0.1]';     
 params.rightLegInit = [ 25.5   0.1  0.0 -18.5 -5.5 -0.1]';

% right leg initial conditions for the robot standing on one foot
%params.rightLegInit = [ 25.5   0.1  0.0 -70 -5.5 -0.1]'; 

 params.qjInit  = [params.torsoInit;params.leftArmInit;params.rightArmInit;params.leftLegInit;params.rightLegInit] * (pi/180);
 params.dqjInit = zeros(params.ndof,1);
   
 params.dx_bInit    = zeros(3,1);
 params.omega_bInit = zeros(3,1);
  
% fixing the world reference frame to the ground, not to the left foot 
 wbm_updateState(params.qjInit,zeros(params.ndof,1),zeros(6,1));
 [qj,T_b,dqj,vb] = wbm_getState();
 [pos,rot] = frame2posrot(T_b);
 wbm_setWorldFrame(rot,pos,[0,0,-9.81]')

% the vector of variables is redefined to add the position of the feet to
% correct numerical errors
%params.chiInit = [T_bInit_mod;params.qjInit;...
%                  params.dx_bInit;params.omega_bInit;params.dqjInit];

 params.chiInit = [T_b;params.qjInit;...
                   params.dx_bInit;params.omega_bInit;params.dqjInit; zeros(12,1)];

%% contact constraints                
 params.constraintLinkNames      = {'l_sole','r_sole'}; 
 
% constraint for the robot on one foot
%params.constraintLinkNames      = {'l_sole'};   

 params.numConstraints           = length(params.constraintLinkNames);
   
%% others parameters for controller
 load('jointLimits.mat')
 limits        = [jl1 jl2];
 params.limits = limits;
 params.com    = wbm_forwardKinematics('com');
 params.qjDes  = params.qjInit;
 
%% setup integration
 forwardDynFunc  = @(t,chi)forwardDynamics(t,chi,params);
    
 params.tStart   = 0;
 params.tEnd     = 20;   
 params.sim_step = 0.01;

%% integrate forward dynamics
 disp('starting numerical integration');
 options = odeset('RelTol',1e-2,'AbsTol',1e-4);
   
 [t,chi] = ode15s(forwardDynFunc,params.tStart:params.sim_step:params.tEnd,params.chiInit,options);

 save('storedTestTrajectory.mat','t','chi','params');
 disp('numerical integration complete..rendering.');

%% visualize forward dynamics
% this is the visualization part of the code
temp = 1;

while(temp<10)

    figure; 
    clf;
    
  % initialize GUI
    figure_main = figure('Name', 'iCub Simulator', 'NumberTitle', 'off',...
                         'Position', [50,400,600,650]);
    
    params.figure_main = figure_main;
    set(figure_main, 'MenuBar', 'none', 'BackingStore', 'off');
    set(figure_main, 'BackingStore', 'off');
 
    params.plot_main =zeros(1,4);
    plot_pos = [0.51,0.20,0.45,0.40;
                0.01,0.20,0.45,0.40;
                0.51,0.62,0.45,0.40;
                0.01,0.62,0.45,0.40];

    for ii=1:4
        params.plot_main(ii) = subplot('Position', plot_pos(ii,:));
        params.plot_objs{ii} = plot3(0,0,0,'.');
        axis([-0.5 0.5 -0.42 0.58 0 1]); hold on;
        patch([-0.45 -0.45 0.45 0.45],[-0.37 0.53 0.53 -0.37],[0 0 0 0],[0.6 0.6 0.8]);
        set(gca,'Color',[0.8 0.8 0.8]);
        set(gca,'XColor',[0.8 0.8 0.8]);
        set(gca,'YColor',[0.8 0.8 0.8]);
        set(gca,'ZColor',[0.8 0.8 0.8]);
        set(gca,'xdir','reverse')
        set(gca, 'drawmode', 'fast');
        params.draw_init = 1;
        rotate3d(gca,'on');

        figure(figure_main);
    end
    
    axes(params.plot_main(1))

  % CoM trajectory
    ndof = params.ndof;
    x_b  = chi(:,1:3);
    qt_b = chi(:,4:7);
    qj   = chi(:,8:ndof+7);

    visualizeForwardDynamics([x_b,qt_b,qj],t,params);
    temp = 10;    
    pause;
    
end

%% plot results
% position of the base frame. for other results run the script
% "visualizer" 

    figure(2)
    plot3(x_b(:,1),x_b(:,2),x_b(:,3))
    hold on
    plot3(x_b(1,1),x_b(1,2),x_b(1,3),'ro')
    grid on
    title('Base frame movements in space') 
 
    axis square
   %axis equal
    
    xlabel('X(m)')
    ylabel('Y(m)')
    zlabel('Z(m)')
       
    