function [] = visualizer_js(t,chi,params)
%% visualizer_js
%  Generates a demo of the robot state obtained using forward dynamics
%  integration, and plot some graphics such as contact forces, CoM error,
%  control torques, ZMP at feet. The user can decide what has been
%  represented by selecting the corresponding variable in
%  integrateForwardDynamics.m

%% Robot demo
 ndof  = params.ndof;
 
 if params.visualizerDemo == 1
     
 figure_main = figure('Name', 'iCub Simulator', 'NumberTitle', 'off',...
                      'Position', [50,400,600,650]);
    
 params.figure_main = figure_main;
 set(figure_main, 'MenuBar', 'none', 'BackingStore', 'off');
 set(figure_main, 'BackingStore', 'off');
 
 params.plot_main = zeros(1,4);
    
 plot_pos = [0.51,0.20,0.45,0.40;
             0.01,0.20,0.45,0.40;
             0.51,0.62,0.45,0.40;
             0.01,0.62,0.45,0.40];

  for ii=1:4
    
        params.plot_main(ii) = subplot('Position', plot_pos(ii,:));
        params.plot_objs{ii} = plot3(0,0,0,'.');
        axis([-0.5 0.5 -0.42 0.58 0 1]);
        hold on;
        
        if params.feet_on_ground(2) == 0 || sum(params.feet_on_ground) == 2
            
          patch([-0.45 -0.45 0.45 0.45],[-0.53 0.37 0.37 -0.53],[0 0 0 0],[0.6 0.6 0.8]);
            
        else
            
          patch([-0.45 -0.45 0.45 0.45],[-0.37 0.53 0.53 -0.37],[0 0 0 0],[0.6 0.6 0.8]);
          
        end
        
        set(gca,'Color',[0.8 0.8 0.8]);
        set(gca,'XColor',[0.8 0.8 0.8]);
        set(gca,'YColor',[0.8 0.8 0.8]);
        set(gca,'ZColor',[0.8 0.8 0.8]);
        set(gca,'xdir','reverse')
        set(gca,'ydir','reverse')
        set(gca, 'drawmode', 'fast');
        params.draw_init = 1;
        rotate3d(gca,'on');

        figure(figure_main);
        
  end
    
 axes(params.plot_main(1))

% base link trajectory
 x_b   = chi(:,1:3);
 qt_b  = chi(:,4:7);
 qj    = chi(:,8:ndof+7);

 visualizeForwardDynamics([x_b,qt_b,qj],params);

% plot root link position
 figure(8)
 plot3(x_b(1:end,1),x_b(1:end,2),x_b(1:end,3));
 hold on
 plot3(x_b(1,1),x_b(1,2),x_b(1,3),'ro');
 grid on;
 title('Position of the root link')
 
 axis square;
%axis equal

 xlabel('X(m)');
 ylabel('Y(m)');
 zlabel('Z(m)');

 end

%% Graphics visualization
if params.visualizerGraphics == 1

dim_t      = length(t);
 
qj         = zeros(ndof,dim_t);
tau        = zeros(ndof,dim_t);
pos_feet   = zeros(12,dim_t);
   
fc         = zeros(6*params.numConstraints,dim_t);
CoP        = zeros(2*params.numConstraints,dim_t);

error_CoM  = zeros(3,dim_t);
qjDes      = qj;

t_kin      = params.t_kin;
qjIkin     = params.joints_traj.qj;
norm_tau   = zeros(dim_t,1);

for time = 1:length(t)
    
[~,visual]           = forwardDynamics_js(t(time), chi(time,:)', params);

qj(:,time)           = visual.qj;
pos_feet(:,time)     = visual.pos_feet;
fc(:,time)           = visual.fc;

error_CoM(:,time)    = visual.errorCoM;
tau(:,time)          = visual.tau;

qjDes(:,time)        = visual.qjDes;

norm_tau(time)       = norm(visual.tau);

% CoP at feet
CoP(1) = -visual.fc(5)/visual.fc(3);
CoP(2) =  visual.fc(4)/visual.fc(3);

if  sum(params.feet_on_ground) == 2
    
CoP(3) = -visual.fc(11)/visual.fc(9);
CoP(4) =  visual.fc(10)/visual.fc(9);

end

end

%% Feet position and orientation
for k=1:3

if params.feet_on_ground(1) == 1

figure(9)
subplot(1,2,1)
hold all
grid on
plot(t,pos_feet(k,:))
title('Left foot position')
xlabel('s')
ylabel('m')

figure(9)
subplot(1,2,2)
hold all
grid on
plot(t,pos_feet(k+3,:))
title('Left foot orientation')
xlabel('s')
ylabel('rad')

else

figure(9)
subplot(1,2,1)
hold all
grid on
plot(t,pos_feet(k+6,:))
title('Right foot position')
xlabel('s')
ylabel('m')

figure(9)
subplot(1,2,2)
hold all
grid on
plot(t,pos_feet(k+9,:))
title('Right foot orientation')
xlabel('s')
ylabel('rad')

end

end    
 
%% Contact forces
for k=1:6*params.numConstraints

figure(10)
hold all
grid on
plot(t,fc(k,:))
title('Contact forces and moments')
xlabel('s')
ylabel('N,Nm')

end

%% CoM error
for k=1:3

figure(11)
hold all
grid on
plot(t,error_CoM(k,:))
title('CoM error')
xlabel('s')
ylabel('m')

end

%% Norm of joints torques
figure(12)
hold on
grid on
plot(t,norm_tau)
title('Square norm of joints torques')
xlabel('s')
ylabel('Nm')

%% Joints variables visualization
if params.visualizerJoints == 1
        
for k=1:5

% Arms position
figure(13)
subplot(3,2,k)
plot(t,qj(k+3,:),t,qjDes(k+3,:),'r',t_kin,qjIkin(k+3,:),'k')
hold on
grid on
xlabel('s')
ylabel('rad')

name = whatname('l_arm',k);
title(name)
legend('Real pos','Ikin pos','Desired pos')

figure(14)
subplot(3,2,k)
plot(t,qj(k+3+5,:),t,qjDes(k+3+5,:),'r',t_kin,qjIkin(k+3+5,:),'k')
hold on
grid on
xlabel('s')
ylabel('rad')

name = whatname('r_arm',k);
title(name)
legend('Real pos','Ikin pos','Desired pos')

end

for k=1:6
    
% Legs position
figure(15)
subplot(3,2,k)
plot(t,qj(k+13,:),t,qjDes(k+13,:),'r',t_kin,qjIkin(k+13,:),'k')
hold on
grid on
xlabel('s')
ylabel('rad')

name = whatname('l_leg',k);
title(name)
legend('Real pos','Ikin pos','Desired pos')

figure(16)
subplot(3,2,k)
plot(t,qj(k+13+6,:),t,qjDes(k+13+6,:),'r',t_kin,qjIkin(k+13+6,:),'k')
hold on
grid on
xlabel('s')
ylabel('rad')

name = whatname('r_leg',k);
title(name)
legend('Real pos','Ikin pos','Desired pos')

end

for k=1:3
  
% Torso position
figure(17)
subplot(3,1,k)
plot(t,qj(k,:),t,qjDes(k,:),'r',t_kin,qjIkin(k,:),'k')
hold on
grid on
xlabel('s')
ylabel('rad')

name = whatname('torso',k);
title(name)
legend('Real pos','Ikin pos','Desired pos')

end

end

%% CoP at feet
for k=1:2

if sum(params.feet_on_ground) == 2

figure(18)
subplot(1,2,1)
plot(CoP(1,:),CoP(2,:))
hold on
grid on 
subplot(1,2,1)
plot(CoP(1,1),CoP(2,1),'or')
title('Left foot CoP')
xlabel('Y direction (m)')
ylabel('X direction (m)')
axis([-0.1 0.1 -0.1 0.1])

figure(18)
subplot(1,2,2)
plot(CoP(3,:),CoP(4,:))
hold on
grid on
subplot(1,2,2)
plot(CoP(3,1),CoP(4,1),'or')
title('Right foot CoP')
xlabel('Y direction (m)')
ylabel('X direction (m)')
axis([-0.1 0.1 -0.1 0.1])

else
    
figure(18)
plot(CoP(1,:),CoP(2,:))
hold on
grid on
plot(CoP(1,1),CoP(2,1),'or')
title('Foot CoP')
xlabel('Y direction (m)')
ylabel('X direction (m)')
axis([-0.1 0.1 -0.1 0.1])

end

end

end
 
end

  