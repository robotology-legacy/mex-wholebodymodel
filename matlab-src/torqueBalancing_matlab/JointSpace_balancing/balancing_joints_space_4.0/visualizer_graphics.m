function [] = visualizer_graphics(t,chi,params)
% visualizer_graphics plots all the desired quantities
% for now, these are the parameters calculated in ForwardDynamics.m
dim_t  = length(t);

qj     = zeros(25,dim_t);
dqj    = qj;
ddqj   = qj;

norm_tau    = zeros(dim_t,1);
pos_feet    = zeros(12,dim_t);

if params.feet_on_ground == 1
    
fc          = zeros(6,dim_t);
CoP         = zeros(2,dim_t);

elseif params.feet_on_ground == 2

fc          = zeros(12,dim_t);
CoP         = zeros(4,dim_t);

end

ecom    = zeros(3,dim_t);

q       = qj;
dq      = qj;
ddq     = qj;

t_kin    = params.t_kin;
qj_d     = params.joints_traj.qj;
dqj_d    = params.joints_traj.dqj;
ddqj_d   = params.joints_traj.ddqj;

for tt = 1:length(t)
    
[~,c]   = forwardDynamics(t(tt), chi(tt,:).', params);

qj(:,tt)      = c.qj;
dqj(:,tt)     = c.dqj;
ddqj(:,tt)    = c.ddqj;

pos_feet(:,tt)       = c.pos_feet;
fc(:,tt)             = c.fc;

ecom(:,tt)           = c.e_com;
CoP(:,tt)            = c.CoP;

q(:,tt)              = c.q;
dq(:,tt)             = c.dq;
ddq(:,tt)            = c.ddq;

norm_tau(tt) = c.norm_t;

end

%% Feet position and orientation
for k=1:3

figure(9)
hold all
grid on
plot(t,pos_feet(k,:))
title('left foot position')

figure(10)
hold all
grid on
plot(t,pos_feet(k+3,:))
title('left foot orientation')

if params.feet_on_ground == 2

figure(11)
hold all
grid on
plot(t,pos_feet(k+6,:))
title('right foot position')

figure(12)
hold all
grid on
plot(t,pos_feet(k+9,:))
title('right foot orientation')

end

end    
 
%% contact forces at feet
if params.feet_on_ground == 1
    
for k=1:6

figure(13)
hold all
grid on
plot(t,fc(k,:))
title('contact forces')

end

elseif params.feet_on_ground == 2

for k=1:12

figure(13)
hold all
grid on
plot(t,fc(k,:))
title('contact forces')

end

end

%% CoM error
for k=1:3

figure(14)
hold all
grid on
plot(t,ecom(k,:))
title('CoM error')

end

%% norm of joints torques
figure(15)
hold on
grid on
plot(t,norm_tau)
title('norm of joints torques')

%% joints variables visualization
if params.vis_joints_variables == 1
    
%% arms joints variables
for k=1:5
    
% position
figure(16)
subplot(3,2,k)
plot(t,qj(k+3,:),t,q(k+3,:),'r',t_kin,qj_d(k+3,:),'k')
hold on
grid on
xlabel('s')
ylabel('rad')

name = whatname('l_arm',k);
title(name)
legend('Real pos','Desired pos','Obtained desired pos')

figure(17)
subplot(3,2,k)
plot(t,qj(k+3+5,:),t,q(k+3+5,:),'r',t_kin,qj_d(k+3+5,:),'k')
hold on
grid on
xlabel('s')
ylabel('rad')

name = whatname('r_arm',k);
title(name)
legend('Real pos','Desired pos','Obtained desired pos')

%% velocity 
figure(21)
subplot(3,2,k)
plot(t,dqj(k+3,:),t,dq(k+3,:),'r',t_kin,dqj_d(k+3,:),'k')
hold on
grid on
xlabel('s')
ylabel('rad/s')

name = whatname('l_arm',k);
title(name)
legend('Real vel','Desired vel','Obtained desired vel')

figure(22)
subplot(3,2,k)
plot(t,dqj(k+3+5,:),t,dq(k+3+5,:),'r',t_kin,dqj_d(k+3+5,:),'k')
hold on
grid on
xlabel('s')
ylabel('rad/s')

name = whatname('r_arm',k);
title(name)
legend('Real vel','Desired vel','Obtained desired vel')

%% acceleration 
figure(26)
subplot(3,2,k)
plot(t,ddqj(k+3,:),t,ddq(k+3,:),'r',t_kin,ddqj_d(k+3,:),'k')
hold on
grid on
xlabel('s')
ylabel('rad/s^2')

name = whatname('l_arm',k);
title(name)
legend('Real acc','Desired acc','Obtained desired acc')

figure(27)
subplot(3,2,k)
plot(t,ddqj(k+3+5,:),t,ddq(k+3+5,:),'r',t_kin,ddqj_d(k+3+5,:),'k')
hold on
grid on
xlabel('s')
ylabel('rad/s^2')

name = whatname('r_arm',k);
title(name)
legend('Real acc','Desired acc','Obtained desired acc')

end

%% legs joints variables

for k=1:6
    
% position
figure(18)
subplot(3,2,k)
plot(t,qj(k+13,:),t,q(k+13,:),'r',t_kin,qj_d(k+13,:),'k')
hold on
grid on
xlabel('s')
ylabel('rad')

name = whatname('l_leg',k);
title(name)
legend('Real pos','Desired pos','Obtained desired pos')

figure(19)
subplot(3,2,k)
plot(t,qj(k+13+6,:),t,q(k+13+6,:),'r',t_kin,qj_d(k+13+6,:),'k')
hold on
grid on
xlabel('s')
ylabel('rad')

name = whatname('r_leg',k);
title(name)
legend('Real pos','Desired pos','Obtained desired pos')

%% velocity
figure(23)
subplot(3,2,k)
plot(t,dqj(k+13,:),t,dq(k+13,:),'r',t_kin,dqj_d(k+13,:),'k')
hold on
grid on
xlabel('s')
ylabel('rad/s')

name = whatname('l_leg',k);
title(name)
legend('Real vel','Desired vel','Obtained desired vel')

figure(24)
subplot(3,2,k)
plot(t,dqj(k+13+6,:),t,dq(k+13+6,:),'r',t_kin,dqj_d(k+13+6,:),'k')
hold on
grid on
xlabel('s')
ylabel('rad/s')

name = whatname('r_leg',k);
title(name)
legend('Real vel','Desired vel','Obtained desired vel')

%% acceleration
figure(28)
subplot(3,2,k)
plot(t,ddqj(k+13,:),t,ddq(k+13,:),'r',t_kin,ddqj_d(k+13,:),'k')
hold on
grid on
xlabel('s')
ylabel('rad/s^2')

name = whatname('l_leg',k);
title(name)
legend('Real acc','Desired acc','Obtained desired acc')

figure(29)
subplot(3,2,k)
plot(t,ddqj(k+13+6,:),t,ddq(k+13+6,:),'r',t_kin,ddqj_d(k+13+6,:),'k')
hold on
grid on
xlabel('s')
ylabel('rad/s^2')

name = whatname('r_leg',k);
title(name)
legend('Real acc','Desired acc','Obtained desired acc')

end

%% torso joints variables
for k=1:3
  
%position
figure(20)
subplot(3,1,k)
plot(t,qj(k,:),t,q(k,:),'r',t_kin,qj_d(k,:),'k')
hold on
grid on
xlabel('s')
ylabel('rad')

name = whatname('torso',k);
title(name)
legend('Real pos','Desired pos','Obtained desired pos')

%% velocity
figure(25)
subplot(3,1,k)
plot(t,dqj(k,:),t,dq(k,:),'r',t_kin,dqj_d(k,:),'k')
hold on
grid on
xlabel('s')
ylabel('rad/s')

name = whatname('torso',k);
title(name)
legend('Real vel','Desired vel','Obtained desired vel')

%% acceleration
figure(30)
subplot(3,1,k)
plot(t,ddqj(k,:),t,ddq(k,:),'r',t_kin,ddqj_d(k,:),'k')
hold on
grid on
xlabel('s')
ylabel('rad/s^2')

name = whatname('torso',k);
title(name)  
legend('Real acc','Desired acc','Obtained desired acc')

end

end

%% CoP graphics
for k=1:2

figure(31)
plot(t,CoP(k,:))
hold all
grid on
title('Left foot CoP')

if params.feet_on_ground == 2

figure(32)
plot(t,CoP(k+2,:))
hold all
grid on
title('Right foot CoP') 

end

end

end
