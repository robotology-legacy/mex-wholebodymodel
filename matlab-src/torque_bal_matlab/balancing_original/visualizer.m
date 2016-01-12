function [] = visualizer(t,chi,param)
%% visualizer
% visualize some user-defined parameters such as contact forces, torques,
% CoM error. It also visualizes a demo of the robot's movements
if param.visualizer_demo == 1
figure_main = figure('Name', 'iCub Simulator', 'NumberTitle', 'off',...
                     'Position', [50,400,600,650]);
    
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
ndof  = param.ndof;
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
title('Positions of the root link')
 
 axis square;
%axis equal
    
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');

end

if param.visualizer_graphics == 1
% plot all the desired quantities
% the parameters are calculated in forwardDynamics.m
qj       = zeros(param.ndof,length(t));
fc       = zeros(6*param.numConstraints,length(t));
f0       = zeros(6*param.numConstraints,length(t));
tau      = zeros(param.ndof,length(t));
ecom     = zeros(3,length(t));
pos      = zeros(14,length(t));
norm_tau = zeros(1,length(t));
CoP      = zeros(4,length(t));

for tt=1:length(t)
    
[~,visual]   = forwardDynamics(t(tt), chi(tt,:)', param);

qj(:,tt)     = visual.qj;
pos(:,tt)    = visual.pos_feet;
fc(:,tt)     = visual.fc;
f0(:,tt)     = visual.f0;
tau(:,tt)    = visual.tau;
ecom(:,tt)   = visual.error_com;
norm_tau(tt) = norm(visual.tau);
CoP(:,tt)    = visual.CoP;

end

% feet position and orientation
for k=1:3

if param.feet_on_ground(1) == 1   
figure(3)
hold all
grid on
plot(t,pos(k,:))
title('left foot position')

figure(4)
hold all
grid on
plot(t,pos(k+4,:))
title('left foot orientation')

else
    
figure(3)
hold all
grid on
plot(t,pos(k+7,:))
title('right foot position')

figure(4)
hold all
grid on
plot(t,pos(k+10,:))
title('right foot orientation')

end

end
    
% contact forces
if sum(param.feet_on_ground) == 1

for k=1:6

figure(5)
hold all
grid on
plot(t,fc(k,:))
title('contact forces')

end

elseif sum(param.feet_on_ground) == 2

for k=1:12

figure(5)
hold all
grid on
plot(t,fc(k,:))
title('contact forces')

figure(6)
hold all
grid on
plot(t,f0(k,:))
title('f0')

end

end

% torques and CoM
for k=1:25

figure(7)
hold all
grid on
plot(t,tau(k,:))
title('torques at joints')

end

for k=1:3

figure(8)
hold all
grid on
plot(t,ecom(k,:))
title('CoM error')

end

figure(9)
hold on
grid on
plot(t,norm_tau)
title('norm of joints torques')

%% Joints positions
if param.visualizer_joints == 1
    
for k=1:5
   
figure(10)
subplot(3,2,k)
plot(t,qj(k+3,:))
hold on
grid on
xlabel('s')
ylabel('rad')

name = whatname('l_arm',k);
title(name)

figure(11)
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

figure(12)
subplot(3,2,k)
plot(t,qj(k+13,:))
hold on
grid on
xlabel('s')
ylabel('rad')

name = whatname('l_leg',k);
title(name)

figure(13)
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
    
figure(14)
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

if param.feet_on_ground(1) == 1
    
figure(15)
plot(CoP(2,:),CoP(1,:))
hold on
grid on
title('Left foot CoP')

end

if param.feet_on_ground(2) == 1 

figure(16)
plot(CoP(4,:),CoP(3,:))
hold on
grid on
title('Right foot CoP')

end

end
