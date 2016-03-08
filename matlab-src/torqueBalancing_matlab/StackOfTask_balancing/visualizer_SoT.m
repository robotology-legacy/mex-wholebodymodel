function [] = visualizer_SoT(t,chi,param)
%% visualizer_SoT
%  Visualizes some user-defined parameters such as contact forces, torques,
%  CoM error. It can also generate a demo of the robot's movements.

%% Demo generation
ndof  = param.ndof;

if param.visualizer_demo == 1
    
figure_main = figure('Name', 'iCub Simulator', 'NumberTitle', 'off', 'Position', [50,400,600,650]);
    
param.figure_main = figure_main;
set(figure_main, 'MenuBar', 'none', 'BackingStore', 'off');
set(figure_main, 'BackingStore', 'off');
 
param.plot_main = zeros(1,4);
    
plot_pos = [0.51,0.20,0.45,0.40;
            0.01,0.20,0.45,0.40;
            0.51,0.62,0.45,0.40;
            0.01,0.62,0.45,0.40];

for ii=1:4
    
        param.plot_main(ii) = subplot('Position', plot_pos(ii,:));
        param.plot_objs{ii} = plot3(0,0,0,'.');
        axis([-0.5 0.5 -0.42 0.58 0 1]);
        hold on
        patch([-0.45 -0.45 0.45 0.45],[-0.37 0.53 0.53 -0.37],[0 0 0 0],[0.6 0.6 0.8]);
        set(gca,'Color',[0.8 0.8 0.8]);
        set(gca,'XColor',[0.8 0.8 0.8]);
        set(gca,'YColor',[0.8 0.8 0.8]);
        set(gca,'ZColor',[0.8 0.8 0.8]);
        set(gca,'xdir','reverse')
        set(gca, 'drawmode', 'fast');
        param.draw_init = 1;
        rotate3d(gca,'on');

        figure(figure_main);
        
end
    
axes(param.plot_main(1))

% CoM trajectory
x_b   = chi(:,1:3);
qt_b  = chi(:,4:7);
qj    = chi(:,8:ndof+7);

visualizeForwardDynamics([x_b,qt_b,qj],t,param);

%% Plot base link position
figure(2)
plot3(x_b(:,1),x_b(:,2),x_b(:,3));
hold on
plot3(x_b(1,1),x_b(1,2),x_b(1,3),'ro');
grid on;
title('Root link position')
 
 axis square;
%axis equal
    
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');

end

%% Graphics generation 
if param.visualizer_graphics == 1
    
%  generates the parameters defined in forwardDynamics_SoT.m
qj            = zeros(param.ndof,length(t));
fc            = zeros(6*param.numConstraints,length(t));
f0            = zeros(6*param.numConstraints,length(t));
tau           = zeros(param.ndof,length(t));
error_CoM     = zeros(3,length(t));
pos_feet      = zeros(14,length(t));
norm_tau      = zeros(1,length(t));
CoP           = zeros(4,length(t));
phi_lfoot     = zeros(3,length(t));
phi_rfoot     = zeros(3,length(t));

%Added for IROS
xCoM          = zeros(3,length(t));
dxCoM         = zeros(3,length(t));
xCoMdes       = zeros(3,length(t));
dxCoMdes      = zeros(3,length(t));
H             = zeros(6,length(t));
Href          = zeros(6,length(t));
norm_qjErr    = zeros(1,length(t));
norm_HErr     = zeros(1,length(t));
m             = param.M0(1,1);

for time=1:length(t)
    
[~,visual]          = forwardDynamics_SoT(t(time), chi(time,:)', param);

qj(:,time)          = visual.qj;
pos_feet(:,time)    = visual.pos_feet;
fc(:,time)          = visual.fc;
f0(:,time)          = visual.f0;
tau(:,time)         = visual.tau;
error_CoM(:,time)   = visual.error_com;

% norm of joint torques and CoP
norm_tau(time)   = norm(visual.tau);

%Added for IROS
norm_qjErr(time) = norm(visual.qj-param.qjInit);
norm_HErr(time)  = norm(visual.H-visual.Href);
H(:,time)        = visual.H;
xCoM(:,time)     = visual.xCoM;
dxCoM(:,time)    = visual.dxCoM;
xCoMdes(:,time)  = visual.xCoMdes;
dxCoMdes(:,time) = visual.dxCoMdes;
Href(:,time)     = visual.Href;
 
CoP(1,time)    = -visual.fc(5)/visual.fc(3);
CoP(2,time)    =  visual.fc(4)/visual.fc(3);

if  param.numConstraints == 2 
    
CoP(3,time)    = -visual.fc(11)/visual.fc(9);
CoP(4,time)    =  visual.fc(10)/visual.fc(9);

end

% left foot orientation
quat_lFoot             = visual.pos_feet(1:7);
[~,Rot_lFoot]          = frame2posrot(quat_lFoot);
[~,phi_lfoot(:,time)]  = parametrization(Rot_lFoot);

% right foot orientation
quat_rFoot             = visual.pos_feet(8:end);
[~,Rot_rFoot]          = frame2posrot(quat_rFoot);
[~,phi_rfoot(:,time)]  = parametrization(Rot_rFoot);

end

%% Save datas for IROS
% save('visualizer', 'xCoM','dxCoM','m','H','Href','norm_qjErr','norm_HErr','fc','tau','xCoMdes','dxCoMdes')

%% Feet position and orientation
for k=1:3

if param.feet_on_ground(1) == 1  
    
figure(3)
subplot(1,2,1)
hold all
grid on
plot(t,pos_feet(k,:))
title('Left foot position')
xlabel('s')
ylabel('m')

figure(3)
subplot(1,2,2)
hold all
grid on
plot(t,phi_lfoot(k,:))
title('Left foot orientation')
xlabel('s')
ylabel('rad')

else
    
figure(3)
subplot(1,2,1)
hold all
grid on
plot(t,pos_feet(k+7,:))
title('Right foot position')
xlabel('s')
ylabel('m')

figure(3)
subplot(1,2,2)
hold all
grid on
plot(t,phi_rfoot(k,:))
title('Right foot orientation')
xlabel('s')
ylabel('rad')

end

end
    
%% Contact forces
if sum(param.feet_on_ground) == 1

for k=1:6

figure(4)
hold all
grid on
plot(t,fc(k,:))
title('Contact forces and moments')
xlabel('s')
ylabel('N,Nm')

end

elseif sum(param.feet_on_ground) == 2

for k=1:12

figure(4)
hold all
grid on
plot(t,fc(k,:))
title('Contact forces and moments')
xlabel('s')
ylabel('N,Nm')

figure(5)
hold all
grid on
plot(t,f0(k,:))
title('Contact forces and moments null space')
xlabel('s')
ylabel('N,Nm')

end

end

%% Control torques and CoM error
for k=1:ndof

figure(6)
hold all
grid on
plot(t,tau(k,:))
title('Torques at joints')
xlabel('s')
ylabel('Nm')

end

for k=1:3

figure(7)
hold all
grid on
plot(t,error_CoM(k,:))
title('CoM error')
xlabel('s')
ylabel('m')

end

figure(8)
hold on
grid on
plot(t,norm_tau)
title('Square norm of joints torques')
xlabel('s')
ylabel('Nm')

%% Joints positions
if param.visualizer_jointsPos == 1
    
for k=1:5
  
% Robot arms    
figure(9)
subplot(3,2,k)
plot(t,qj(k+3,:))
hold on
grid on
xlabel('s')
ylabel('rad')
name = whatname('l_arm',k);
title(name)

figure(10)
subplot(3,2,k)
plot(t,qj(k+3+5,:))
hold on
grid on
xlabel('s')
ylabel('rad')
name = whatname('r_arm',k);
title(name)

end

for k=1:6

% Robot legs
figure(11)
subplot(3,2,k)
plot(t,qj(k+13,:))
hold on
grid on
xlabel('s')
ylabel('rad')
name = whatname('l_leg',k);
title(name)

figure(12)
subplot(3,2,k)
plot(t,qj(k+13+6,:))
hold on
grid on
xlabel('s')
ylabel('rad')
name = whatname('r_leg',k);
title(name)

end

for k=1:3
    
% Robot torso
figure(13)
subplot(3,1,k)
plot(t,qj(k,:))
hold on
grid on
xlabel('s')
ylabel('rad')
name = whatname('torso',k);
title(name)  

end

end

%% CoP at feet 
if param.numConstraints == 1
    
figure(14)
plot(CoP(1,:),CoP(2,:))
hold on
grid on
plot(CoP(1,1),CoP(2,1),'or')
title('Foot CoP')
xlabel('X direction (m)')
ylabel('Y direction (m)')
axis([-0.1 0.1 -0.1 0.1])

end

if param.numConstraints == 2 

figure(14)
subplot(1,2,1)
plot(CoP(1,:),CoP(2,:))
hold on
grid on
plot(CoP(1,1),CoP(2,1),'or')

title('Left foot CoP')
xlabel('X direction (m)')
ylabel('Y direction (m)')
axis([-0.1 0.1 -0.1 0.1])

figure(14)
subplot(1,2,2)
plot(CoP(3,:),CoP(4,:))
hold on
grid on
plot(CoP(3,1),CoP(4,1),'or')

title('Right foot CoP')
xlabel('X direction (m)')
ylabel('Y direction (m)')
axis([-0.1 0.1 -0.1 0.1])

end

%% Square norm of joints positions
figure(15)
hold on
grid on
plot(t,norm_qjErr)

xlabel('s')
ylabel('rad')
title('Square norm of joints pos error')

%% Square norm of H error
figure(16)
hold on
grid on
plot(t,norm_HErr)

xlabel('s')
ylabel('|H-H_{ref}|')
title('Square norm of H error')

end
