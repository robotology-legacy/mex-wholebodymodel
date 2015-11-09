function visualizeForwardDynamics(xout,tSpan,params)
%% visualize_forwardDyn 
%   Visualize the simulation results obtained from integration 
%   of the forward dynamics of the iCub. 
%   
%   visualize_forwardDyn(XOUT,PARAMETERS) visualizes the motion 
%   of the robot. XOUT is the output vector of the integration carried 
%   out in the forward dynamics part, containing the position and the
%   orientation of the base and the joint positions in the first 32 
%   elements of its row vectors along a time span. PARAMETERS is the 
%   struct variable which contains constant parameters related to the 
%   simulation environment, robot, controller etc.
%  

n  = size(xout,1); % number of instances of the simulation results
q  = xout(:,1:7);    % first 3 elements provide the position and next 4 elements provide the orientation of the base 
qj = xout(:,8:32);  % joint positions

% time span vector of the simulation
%tSpan   = linspace(  params.sim_start_time,  params.sim_start_time+params.sim_duration,  params.sim_duration/params.sim_step);

vis_speed = 1;    % this variable is set to change the visualization speed, 
                  % to make its speed close to the real time in case 
                  % the simulation time step is changed.
                  
%wbm_setWorldLink('l_sole',eye(3),[0 0 0]',[ 0,0,-9.81]');

[rot,pos] = wbm_getWorldFrameFromFixedLink('l_sole',params.qjInit);
wbm_setWorldFrame(rot,pos,[ 0,0,-9.81]');
wbm_updateState(params.qjInit,zeros(params.ndof,1),zeros(6,1));


    
% the list of link/joint names that are used to construct the robot in the visualizer                  
L = cell(15,1);
L{1}  = 'root_link'   ;
L{2}  = 'r_hip_1'     ;
L{3}  = 'r_lower_leg' ;
L{4}  = 'r_sole'      ;
L{5}  = 'l_hip_1'     ;
L{6}  = 'l_lower_leg' ;   
L{7}  = 'l_sole'      ;
L{8}  = 'neck_1'      ;
L{9}  = 'r_shoulder_1';
L{10} = 'r_elbow_1'   ;
L{11} = 'r_gripper'   ;
L{12} = 'l_shoulder_1';
L{13} = 'l_elbow_1'   ;
L{14} = 'l_gripper'   ;
L{15} = 'com'         ;


% RELATED TO WORLD REFERENCE FRAME ISSUE
% since for now the world reference frame is that of the codyco_balancing_world, 
% the z-axis is the vertical axis for the robot.
xaxis = 'xdata';
yaxis = 'ydata';
zaxis = 'zdata';

n_plot = 15; % number of points to be plotted (virtual joints)
n_lin = 13;  % number of lines to be plotted (virtual links)

kin = zeros(size(xout,1),7,n_plot);
pB = zeros(3,1);
for jj=1:n_plot
    for ii=1:n % at each instance
       % fprintf('Link Name %s \n',L{jj})
       % convert base state to rotation
       [pB,R] = frame2posrot(squeeze(q(ii,:)'));
       % set world to base
       wbm_setWorldFrame(R,pB,[0,0,-9.81]');
       kin(ii,:,jj) = (wbm_forwardKinematics(R,pB,qj(ii,:)',L{jj}))'; % forward kinematics for the list of joints/links
        
    end
    %q(jj,1:3)
end

kin(:,:,1)= q; %use base data instead of fwdkin rootlink

% clear and reset the plots
for ii=4:-1:1  
    axes(params.plot_main(ii));
    cla;
    axis off;
    patch([-0.45 -0.45 0.45 0.45],[-0.37 0.53 0.53 -0.37],[0 0 0 0],[201 220 222]./255);

    drawnow
end

axes(params.plot_main(1));

% get the desired trajectory for the given task
traj_dat = zeros(size(tSpan,1),3); 
%for kk = 1:n
    %temp = params.controller.Desired_x_dx_ddx_CoM(tSpan(kk));
    %traj_dat(kk,:) = temp(:,1)';
%end

%% INITIAL PLOTS

% allocate memory
x = zeros(1,n_plot);
y = zeros(1,n_plot);
z = zeros(1,n_plot);
R = zeros(3,3,n_plot);
pos = zeros(1,n_plot);
xyzpairs = zeros(n_lin,6);

% plot the desired trajectory
%plot_traj = plot3(traj_dat(:,1),traj_dat(:,2),traj_dat(:,3),'r');

% plot the base position
x(1)=kin(1,1,1);
y(1)=kin(1,2,1);
z(1)=kin(1,3,1);
pos(1)=plot3(x(1),y(1),z(1),'w.');

% plot the joints
for jj=2:n_plot-1
    [Ptemp,Rtemp] = frame2posrot(kin(1,:,jj)');
    x(jj) = Ptemp(1);
    y(jj) = Ptemp(2);
    z(jj) = Ptemp(3);
   
%    if(jj == 7)
%     col = 'ro';
%    end
%    if(jj == 4)
%     col = 'bo';
%    end
  %  col = 'bo';
  %  pos(jj)=plot3(x(jj),y(jj),z(jj),col);
  %  pause;
    col = 'w.';
    pos(jj)=plot3(x(jj),y(jj),z(jj),col);
end

% plot the position of the center of mass
jj=n_plot;
%[x(jj),y(jj),z(jj)] = quat2rot(kin(1,:,jj));
    [Ptemp,Rtemp] = frame2posrot(kin(1,:,jj)');
    x(jj) = Ptemp(1);
    y(jj) = Ptemp(2);
    z(jj) = Ptemp(3);
 
pos(jj)=plot3(x(jj),y(jj),z(jj),'g*');

%% RELATED TO WORLD REFERENCE FRAME ISSUE (see line 44)
%for jj=1:n_plot
%    [x(jj),y(jj),z(jj)] = quat2rot(kin(1,:,jj));
%    set(pos(jj),xaxis,x(jj),yaxis,y(jj),zaxis,z(jj));
%end


% define the pairs between the joints that will form the links
xyzpairs( 1,:) = [ x(1)  x(8)  y(1)  y(8)  z(1)  z(8)];
xyzpairs( 2,:) = [ x(1)  x(2)  y(1)  y(2)  z(1)  z(2)];
xyzpairs( 3,:) = [ x(3)  x(2)  y(3)  y(2)  z(3)  z(2)];
xyzpairs( 4,:) = [ x(3)  x(4)  y(3)  y(4)  z(3)  z(4)];
xyzpairs( 5,:) = [ x(1)  x(5)  y(1)  y(5)  z(1)  z(5)];
xyzpairs( 6,:) = [ x(6)  x(5)  y(6)  y(5)  z(6)  z(5)];
xyzpairs( 7,:) = [ x(6)  x(7)  y(6)  y(7)  z(6)  z(7)];
xyzpairs( 8,:) = [ x(8)  x(9)  y(8)  y(9)  z(8)  z(9)];
xyzpairs( 9,:) = [x(10)  x(9) y(10)  y(9) z(10)  z(9)];
xyzpairs(10,:) = [x(10) x(11) y(10) y(11) z(10) z(11)];
xyzpairs(11,:) = [ x(8) x(12)  y(8) y(12)  z(8) z(12)];
xyzpairs(12,:) = [x(13) x(12) y(13) y(12) z(13) z(12)];
xyzpairs(13,:) = [x(13) x(14) y(13) y(14) z(13) z(14)];

% allocate memory
lin         = zeros(1,n_lin);
lnkpatch    = zeros(1,n_lin);
xyzpatch.vertices = zeros(8,3);
xyzpatch.faces    = zeros(6,4);

% constant multipliers related to the sizes of the patches around the links to form the robot figure 
mult_patch = [0.07 , 0.03;
              0.04 , 0.02;
              0.03 , 0.02;
              0.025, 0.02;
              0.04 , 0.02;
              0.03 , 0.02;
              0.025, 0.02;
              0.03 , 0.02;
              0.025, 0.02;
              0.02 , 0.02;
              0.03 , 0.02;
              0.025, 0.02;
              0.02 , 0.02];
          
% plot the lines depicting the links
for jj=1:n_lin
    
    lin(jj) = line(xaxis,xyzpairs(jj,1:2),yaxis,xyzpairs(jj,3:4),zaxis,xyzpairs(jj,5:6),'erasemode','normal','linewidth',3,'color','black');
    
    % for the patches (to determine the orientation of the patch to be applied to the links)
    vectlnk  = [xyzpairs(jj,2)-xyzpairs(jj,1),xyzpairs(jj,4)-xyzpairs(jj,3),xyzpairs(jj,6)-xyzpairs(jj,5)];
    orthlnk  = null(vectlnk);
    orthlnk1 = mult_patch(jj,1)*orthlnk(:,1); 
    orthlnk2 = mult_patch(jj,2)*orthlnk(:,2);
    
    % offsets in the direction orthogonal to the link
    qq1      = orthlnk1+orthlnk2;
    qq2      = -orthlnk1+orthlnk2;
    qq3      = -orthlnk1-orthlnk2;
    qq4      = orthlnk1-orthlnk2;
    
    % vertices for the patch
    xyzpatch.vertices = [xyzpairs(jj,2)+qq1(1) , xyzpairs(jj,4)+qq1(2) , xyzpairs(jj,6)+qq1(3);
                         xyzpairs(jj,2)+qq2(1) , xyzpairs(jj,4)+qq2(2) , xyzpairs(jj,6)+qq2(3);
                         xyzpairs(jj,2)+qq3(1) , xyzpairs(jj,4)+qq3(2) , xyzpairs(jj,6)+qq3(3);
                         xyzpairs(jj,2)+qq4(1) , xyzpairs(jj,4)+qq4(2) , xyzpairs(jj,6)+qq4(3);              
                         xyzpairs(jj,1)+qq1(1) , xyzpairs(jj,3)+qq1(2) , xyzpairs(jj,5)+qq1(3);
                         xyzpairs(jj,1)+qq2(1) , xyzpairs(jj,3)+qq2(2) , xyzpairs(jj,5)+qq2(3);
                         xyzpairs(jj,1)+qq3(1) , xyzpairs(jj,3)+qq3(2) , xyzpairs(jj,5)+qq3(3);
                         xyzpairs(jj,1)+qq4(1) , xyzpairs(jj,3)+qq4(2) , xyzpairs(jj,5)+qq4(3)];
            
    % RELATED TO WORLD REFERENCE FRAME ISSUE (see line 44)        
    %temp = xyzpatch.vertices(:,1);        
    %xyzpatch.vertices(:,1) = xyzpatch.vertices(:,3);
    %xyzpatch.vertices(:,3) = temp;
   % 
    xyzpatch.faces = [ 1 2 3 4;
                       1 4 8 5;
                       5 8 7 6;
                       7 3 2 6;
                       2 6 5 1;
                       3 7 8 4];

    lnkpatch(jj) = patch('vertices',xyzpatch.vertices,'faces',xyzpatch.faces,'FaceAlpha',0.2);
end

% right foot patch
jj=n_lin+1; 

orthlnk1 = [0 0.03 0]';
orthlnk2 = [0 0 0.03]';

qq1 =  orthlnk1+2*orthlnk2;
qq2 = -orthlnk1+2*orthlnk2;
qq3 = -orthlnk1-orthlnk2;
qq4 =  orthlnk1-orthlnk2;

xyzpatch.vertices = [xyzpairs(4,2)+qq1(1)      , xyzpairs(4,4)+qq1(2) , xyzpairs(4,6)+qq1(3);
                     xyzpairs(4,2)+qq2(1)      , xyzpairs(4,4)+qq2(2) , xyzpairs(4,6)+qq2(3);
                     xyzpairs(4,2)+qq3(1)      , xyzpairs(4,4)+qq3(2) , xyzpairs(4,6)+qq3(3);
                     xyzpairs(4,2)+qq4(1)      , xyzpairs(4,4)+qq4(2) , xyzpairs(4,6)+qq4(3);
                     xyzpairs(4,2)+qq1(1)+0.03 , xyzpairs(4,4)+qq1(2) , xyzpairs(4,6)+qq1(3);
                     xyzpairs(4,2)+qq2(1)+0.03 , xyzpairs(4,4)+qq2(2) , xyzpairs(4,6)+qq2(3);
                     xyzpairs(4,2)+qq3(1)+0.03 , xyzpairs(4,4)+qq3(2) , xyzpairs(4,6)+qq3(3);
                     xyzpairs(4,2)+qq4(1)+0.03 , xyzpairs(4,4)+qq4(2) , xyzpairs(4,6)+qq4(3)];
 
% RELATED TO WORLD REFERENCE FRAME ISSUE (see line 44)
%temp = xyzpatch.vertices(:,1);        
%xyzpatch.vertices(:,1) = xyzpatch.vertices(:,3);
%xyzpatch.vertices(:,3) = temp;

lnkpatch(jj) = patch('vertices',xyzpatch.vertices,'faces',xyzpatch.faces,'FaceAlpha',0.2);

% left foot patch
jj=n_lin+2; 


orthlnk1 = [0 0.03 0]';
orthlnk2 = [0 0 0.03]';

qq1 =  orthlnk1+2*orthlnk2;
qq2 = -orthlnk1+2*orthlnk2;
qq3 = -orthlnk1-orthlnk2;
qq4 =  orthlnk1-orthlnk2;

xyzpatch.vertices = [xyzpairs(7,2)+qq1(1)      , xyzpairs(7,4)+qq1(2) , xyzpairs(7,6)+qq1(3);
                     xyzpairs(7,2)+qq2(1)      , xyzpairs(7,4)+qq2(2) , xyzpairs(7,6)+qq2(3);
                     xyzpairs(7,2)+qq3(1)      , xyzpairs(7,4)+qq3(2) , xyzpairs(7,6)+qq3(3);
                     xyzpairs(7,2)+qq4(1)      , xyzpairs(7,4)+qq4(2) , xyzpairs(7,6)+qq4(3);
                     xyzpairs(7,2)+qq1(1)+0.03 , xyzpairs(7,4)+qq1(2) , xyzpairs(7,6)+qq1(3);
                     xyzpairs(7,2)+qq2(1)+0.03 , xyzpairs(7,4)+qq2(2) , xyzpairs(7,6)+qq2(3);
                     xyzpairs(7,2)+qq3(1)+0.03 , xyzpairs(7,4)+qq3(2) , xyzpairs(7,6)+qq3(3);
                     xyzpairs(7,2)+qq4(1)+0.03 , xyzpairs(7,4)+qq4(2) , xyzpairs(7,6)+qq4(3)];

% RELATED TO WORLD REFERENCE FRAME ISSUE (see line 44)
%temp = xyzpatch.vertices(:,1);        
%xyzpatch.vertices(:,1) = xyzpatch.vertices(:,3);
%xyzpatch.vertices(:,3) = temp;
    
lnkpatch(jj) = patch('vertices',xyzpatch.vertices,'faces',xyzpatch.faces,'FaceAlpha',0.2);
% 
%  ii=1;
%  for jj=1:n_plot
%      [x(jj),y(jj),z(jj),R(:,:,jj)] = quat2rot(kin(ii,:,jj));
%      set(pos(jj),xaxis,x(jj),yaxis,y(jj),zaxis,z(jj));
%  end

if exist('params.plot_objs')    
    delete(params.plot_objs{2});
    delete(params.plot_objs{3});
    delete(params.plot_objs{4});    
end
    
% store axes objects' handles to a vector
params.plot_objs{1}=[lnkpatch';lin';pos'];%;plot_traj];

% copy all objects to other axes with different views
axes(params.plot_main(2));
params.plot_objs{2} = copyobj(params.plot_objs{1},params.plot_main(2));
view(-90,90);
axes(params.plot_main(3));
params.plot_objs{3} = copyobj(params.plot_objs{1},params.plot_main(3));
view(0,1);
axes(params.plot_main(4));
params.plot_objs{4} = copyobj(params.plot_objs{1},params.plot_main(4));
view(-90,1);
axes(params.plot_main(1));



%% UPDATING THE PLOTS

%set(plot_traj,xaxis,traj_dat(:,1),yaxis,traj_dat(:,2),zaxis,traj_dat(:,3));

ii=2; 

while ii<n+1 % the visualization instance
    
    tic;  % visualizer step timer start (to setting the visualizer speed)
    
    % get the positions for the current instance
    for jj=1:n_plot
        %[x(jj),y(jj),z(jj),R(:,:,jj)] = quat2rot(kin(ii,:,jj));
        [Ptemp,Rtemp] = frame2posrot(kin(ii,:,jj)');
         x(jj) = Ptemp(1);
          y(jj) = Ptemp(2);
           z(jj) = Ptemp(3);
        set(pos(jj),xaxis,[x(jj)],yaxis,[y(jj)],zaxis,[z(jj)]);  
       % fprintf('%d, ',ii);
    end
    
%     if x(8)<=0
%         break; % stop visualization when the head hits the ground 
%     end
    
    xyzpairs(1,:)  = [ x(1)  x(8)  y(1)  y(8)  z(1)  z(8)];
    xyzpairs(2,:)  = [ x(1)  x(2)  y(1)  y(2)  z(1)  z(2)];
    xyzpairs(3,:)  = [ x(3)  x(2)  y(3)  y(2)  z(3)  z(2)];
    xyzpairs(4,:)  = [ x(3)  x(4)  y(3)  y(4)  z(3)  z(4)];
    xyzpairs(5,:)  = [ x(1)  x(5)  y(1)  y(5)  z(1)  z(5)];
    xyzpairs(6,:)  = [ x(5)  x(6)  y(5)  y(6)  z(5)  z(6)];
    xyzpairs(7,:)  = [ x(6)  x(7)  y(6)  y(7)  z(6)  z(7)];
    xyzpairs(8,:)  = [ x(9)  x(8)  y(9)  y(8)  z(9)  z(8)];
    xyzpairs(9,:)  = [ x(9) x(10)  y(9) y(10)  z(9) z(10)];
    xyzpairs(10,:) = [x(10) x(11) y(10) y(11) z(10) z(11)];
    xyzpairs(11,:) = [ x(8) x(12)  y(8) y(12)  z(8) z(12)];
    xyzpairs(12,:) = [x(13) x(12) y(13) y(12) z(13) z(12)];
    xyzpairs(13,:) = [x(13) x(14) y(13) y(14) z(13) z(14)];
    
    % update the lines for the links wrt the new joint positions
    for jj=1:n_lin
        
        set(lin(jj),xaxis,xyzpairs(jj,1:2),yaxis,xyzpairs(jj,3:4),zaxis,xyzpairs(jj,5:6));
        
        vectlnk   =  [xyzpairs(jj,2)-xyzpairs(jj,1),xyzpairs(jj,4)-xyzpairs(jj,3),xyzpairs(jj,6)-xyzpairs(jj,5)];
        orthlnk   =  null(vectlnk);
        orthlnk1  =  mult_patch(jj,1)*orthlnk(:,1);
        orthlnk2  =  mult_patch(jj,2)*orthlnk(:,2);
        qq1       =  orthlnk1+orthlnk2;
        qq2       = -orthlnk1+orthlnk2;
        qq3       = -orthlnk1-orthlnk2;
        qq4       =  orthlnk1-orthlnk2;
    
    
    
        xyzpatch.vertices = [xyzpairs(jj,2)+qq1(1) , xyzpairs(jj,4)+qq1(2) , xyzpairs(jj,6)+qq1(3);
                             xyzpairs(jj,2)+qq2(1) , xyzpairs(jj,4)+qq2(2) , xyzpairs(jj,6)+qq2(3);
                             xyzpairs(jj,2)+qq3(1) , xyzpairs(jj,4)+qq3(2) , xyzpairs(jj,6)+qq3(3);
                             xyzpairs(jj,2)+qq4(1) , xyzpairs(jj,4)+qq4(2) , xyzpairs(jj,6)+qq4(3);              
                             xyzpairs(jj,1)+qq1(1) , xyzpairs(jj,3)+qq1(2) , xyzpairs(jj,5)+qq1(3);
                             xyzpairs(jj,1)+qq2(1) , xyzpairs(jj,3)+qq2(2) , xyzpairs(jj,5)+qq2(3);
                             xyzpairs(jj,1)+qq3(1) , xyzpairs(jj,3)+qq3(2) , xyzpairs(jj,5)+qq3(3);
                             xyzpairs(jj,1)+qq4(1) , xyzpairs(jj,3)+qq4(2) , xyzpairs(jj,5)+qq4(3)];
            
        % RELATED TO WORLD REFERENCE FRAME ISSUE (see line 44)   
    %    temp = xyzpatch.vertices(:,1);        
    %    xyzpatch.vertices(:,1) = xyzpatch.vertices(:,3);
    %   xyzpatch.vertices(:,3) = temp;
    
        set(lnkpatch(jj),'vertices',xyzpatch.vertices);
    end
    
    % feet patches

    % right foot
    jj=n_lin+1; 

    orthlnk1 = [0 0.03 0]';
    orthlnk2 = [0 0 0.03]';

    qq1 =  orthlnk1+2*orthlnk2;
    qq2 = -orthlnk1+2*orthlnk2;
    qq3 = -orthlnk1-orthlnk2;
    qq4 =  orthlnk1-orthlnk2;

    xyzpatch.vertices = [xyzpairs(4,2)+qq1(1)      , xyzpairs(4,4)+qq1(2) , xyzpairs(4,6)+qq1(3);
                         xyzpairs(4,2)+qq2(1)      , xyzpairs(4,4)+qq2(2) , xyzpairs(4,6)+qq2(3);
                         xyzpairs(4,2)+qq3(1)      , xyzpairs(4,4)+qq3(2) , xyzpairs(4,6)+qq3(3);
                         xyzpairs(4,2)+qq4(1)      , xyzpairs(4,4)+qq4(2) , xyzpairs(4,6)+qq4(3);
                         xyzpairs(4,2)+qq1(1)+0.03 , xyzpairs(4,4)+qq1(2) , xyzpairs(4,6)+qq1(3);
                         xyzpairs(4,2)+qq2(1)+0.03 , xyzpairs(4,4)+qq2(2) , xyzpairs(4,6)+qq2(3);
                         xyzpairs(4,2)+qq3(1)+0.03 , xyzpairs(4,4)+qq3(2) , xyzpairs(4,6)+qq3(3);
                         xyzpairs(4,2)+qq4(1)+0.03 , xyzpairs(4,4)+qq4(2) , xyzpairs(4,6)+qq4(3)];

    % RELATED TO WORLD REFERENCE FRAME ISSUE (see line 44)                 
    %temp = xyzpatch.vertices(:,1);        
    %xyzpatch.vertices(:,1) = xyzpatch.vertices(:,3);
    %xyzpatch.vertices(:,3) = temp;%

    set(lnkpatch(jj),'vertices',xyzpatch.vertices);

    % left foot
    jj=n_lin+2; 

    orthlnk1 = [0 0.03 0]';
    orthlnk2 = [0 0 0.03]';

    qq1 =  orthlnk1+2*orthlnk2;
    qq2 = -orthlnk1+2*orthlnk2;
    qq3 = -orthlnk1-orthlnk2;
    qq4 =  orthlnk1-orthlnk2;

    xyzpatch.vertices = [xyzpairs(7,2)+qq1(1)      , xyzpairs(7,4)+qq1(2) , xyzpairs(7,6)+qq1(3);
                         xyzpairs(7,2)+qq2(1)      , xyzpairs(7,4)+qq2(2) , xyzpairs(7,6)+qq2(3);
                         xyzpairs(7,2)+qq3(1)      , xyzpairs(7,4)+qq3(2) , xyzpairs(7,6)+qq3(3);
                         xyzpairs(7,2)+qq4(1)      , xyzpairs(7,4)+qq4(2) , xyzpairs(7,6)+qq4(3);
                         xyzpairs(7,2)+qq1(1)+0.03 , xyzpairs(7,4)+qq1(2) , xyzpairs(7,6)+qq1(3);
                         xyzpairs(7,2)+qq2(1)+0.03 , xyzpairs(7,4)+qq2(2) , xyzpairs(7,6)+qq2(3);
                         xyzpairs(7,2)+qq3(1)+0.03 , xyzpairs(7,4)+qq3(2) , xyzpairs(7,6)+qq3(3);
                         xyzpairs(7,2)+qq4(1)+0.03 , xyzpairs(7,4)+qq4(2) , xyzpairs(7,6)+qq4(3)];

    % RELATED TO WORLD REFERENCE FRAME ISSUE (see line 44)                 
    %temp = xyzpatch.vertices(:,1);        
    %xyzpatch.vertices(:,1) = xyzpatch.vertices(:,3);
    %xyzpatch.vertices(:,3) = temp;

    set(lnkpatch(jj),'vertices',xyzpatch.vertices);

    % end feet patches    
    
    % store axes objects to a vector
    params.plot_objs{1}=[lnkpatch';lin';pos'];%;plot_traj]; 

    % delete previous and copy the current plot objects to other axes
    delete(params.plot_objs{2});    
    params.plot_objs{2} = copyobj(params.plot_objs{1},params.plot_main(2));
    delete(params.plot_objs{3});
    params.plot_objs{3} = copyobj(params.plot_objs{1},params.plot_main(3));
    delete(params.plot_objs{4});    
    params.plot_objs{4} = copyobj(params.plot_objs{1},params.plot_main(4));
      
    %if ishghandle(params.gui.gui_edit_currentTime)
    %    set(params.gui.gui_edit_currentTime,'String',num2str(ii*params.sim_step));
    %end
    
    drawnow;
    
    % to update the visualizer speed to keep it close to real simulation time
    time_dif = vis_speed*params.sim_step-toc();
    if time_dif>0
        %         1
        pause(time_dif);
    else
        %         2
        vis_speed=vis_speed+1;
    end
        
    ii=ii+vis_speed;
end
%disp('------------');

end

