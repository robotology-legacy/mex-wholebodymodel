function visualizeForwardDynamics(xout,tSpan,config,references,jetsIntensities)
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

text.color    = [1 1 1];
text.fontSize = 22;

[xout,tSpan,comDes,jetsIntensities] = resizeData(xout,tSpan,squeeze(references(:,1,:))',jetsIntensities,config);


config.plotThusts = false;
if nargin == 5
    config.plotThusts = true;
    % the list of link names that are used for identifying the jets positions
    config.jets.frames     = cell(4,1);
    config.jets.frames{1}  = 'l_hand_dh_frame';
    config.jets.frames{2}  = 'r_hand_dh_frame';
    config.jets.frames{3}  = 'l_foot_dh_frame';
    config.jets.frames{4}  = 'r_foot_dh_frame';
end

robotColor  = [1 0.4 1];
thrustColor = [1 0 0];

n  = size(xout,1); % number of instances of the simulation results
q  = xout(:,1:7);  % first 3 elements provide the position and next 4 elements provide the orientation of the base

q_noBpos = q;
q_noBpos(:,1:3)  = repmat([0 0 0.7],size(q(:,1:3),1),1);

qj = xout(:,8:32);  % joint positions

% time span vector of the simulation
%tSpan   = linspace(  config.sim_start_time,  config.sim_start_time+config.sim_duration,  config.sim_duration/config.sim_step);



[~,qjInit,~,~] = stateDemux(config.state0,config);

[R,p] = wbm_getWorldFrameFromFixedLink('l_sole',qjInit);
wbm_setWorldFrame(R,p,[ 0,0,-9.81]');
wbm_updateState(qjInit,zeros(config.ndof,1),zeros(6,1));



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
n_lin  = 13;  % number of lines to be plotted (virtual links)

kin        = zeros(size(xout,1),7,n_plot);
kin_noBpos =  zeros(size(xout,1),7,n_plot);

if config.visualiser.computeKinematics
    for jj=1:n_plot
        for ii=1:n % at each instance
            [pB,R] = frame2posrot(squeeze(q(ii,:)'));
            [pB_noBpos,R_noBpos] = frame2posrot(squeeze(q_noBpos(ii,:)'));
            kin(ii,:,jj) = (wbm_forwardKinematics(R,pB,qj(ii,:)',L{jj}))'; % forward kinematics for the list of joints/links
            kin_noBpos(ii,:,jj) = (wbm_forwardKinematics(R_noBpos,pB_noBpos,qj(ii,:)',L{jj}))'; % forward kinematics for the list of joints/links
        end
    end
else
    load(config.fileName,'kin','kin_noBpos');
end
if config.visualiser.saveKinematics
    save([config.fileName '_kinematics'],'kin','kin_noBpos');
end

kin(:,:,1)= q; %use base data instead of fwdkin rootlink

% clear and reset the plots
for ii=2:-1:1
    axes(config.plot_main(ii));
    grid on
    axis([-0.5 0.5 -0.42 0.58 0 1]);
    set(gca,'XGrid','off','YGrid','off','ZGrid','off','XTick', [],'YTick', [],'ZTick', []);
    set(gca,'LineWidth',5);
    set(gca, 'drawmode', 'fast');
    cla;
    drawnow
end

axes(config.plot_main(1));

%% INITIAL PLOTS

% allocate memory
x = zeros(1,n_plot);
y = zeros(1,n_plot);
z = zeros(1,n_plot);
pos = zeros(1,n_plot);
xyzpairs = zeros(n_lin,6);



% plot the position of the center of mass
jj = n_plot;
comPosition    = kin(:,1:3,jj);


% Change axis of the inertial frame visualization differently
% from that of the body frame
axes(config.plot_main(1));
minimumAxisVariation = [1 1 1];
maxComPos = max(comPosition)+minimumAxisVariation/2;
minComPos = min(comPosition)-minimumAxisVariation/2;

axesInertial = zeros(3,1);
for indexSetAxis = 1:3
    if abs(maxComPos(indexSetAxis) - minComPos(indexSetAxis)) < minimumAxisVariation(indexSetAxis)
        axesInertial(2*(indexSetAxis -1)+1) = minComPos(indexSetAxis) - minimumAxisVariation(indexSetAxis)/2;
        axesInertial(2*(indexSetAxis -1)+2) = minComPos(indexSetAxis) + minimumAxisVariation(indexSetAxis)/2;
    else
        axesInertial(2*(indexSetAxis -1)+1) = minComPos(indexSetAxis);
        axesInertial(2*(indexSetAxis -1)+2) = maxComPos(indexSetAxis);
    end
end

axis(axesInertial);
%%
% plot the base position
x(1)=kin(1,1,1);
y(1)=kin(1,2,1);
z(1)=kin(1,3,1);
pos(1)=plot3(x(1),y(1),z(1),'w*');
% plot the joints and CoM
for jj=2:n_plot
    [Ptemp,~] = frame2posrot(kin(1,:,jj)');
    x(jj) = Ptemp(1);
    y(jj) = Ptemp(2);
    z(jj) = Ptemp(3);
    col = 'w*';
    if jj == n_plot
        col = 'r*';
    end
    pos(jj)=plot3(x(jj),y(jj),z(jj),col);
end

%% RELATED TO WORLD REFERENCE FRAME ISSUE (see line 44)

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

%% PLOT LINES DEPICTING LINKS
for jj=1:n_lin
    
    lin(jj) = line(xaxis,xyzpairs(jj,1:2),yaxis,xyzpairs(jj,3:4),zaxis,xyzpairs(jj,5:6),'erasemode','normal','linewidth',3,'color','red');
    
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
    
    lnkpatch(jj) = patch('vertices',xyzpatch.vertices,'faces',xyzpatch.faces,...
        'FaceAlpha',0.2,'FaceColor',robotColor);
end

%% FEET PATCHES
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


lnkpatch(jj) = patch('vertices',xyzpatch.vertices,'faces',xyzpatch.faces,'FaceAlpha',0.2,...
    'FaceColor',robotColor);

% left foot patch
jj=n_lin+2;

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

lnkpatch(jj) = patch('vertices',xyzpatch.vertices,'faces',xyzpatch.faces,'FaceAlpha',0.2,...
    'FaceColor',robotColor);
%% PLOT THRUSTS
F   = 0.01;
delta = 5*pi/180;
nT = 2;
tCy = 0:0.01:tan(delta);
[xT, yT, zT] = cylinder(1.2*F.*(1+tCy));
Coord_Thrust = roty(-pi/2)*fromMesh2sortedVector(xT,yT,zT);
xT = sortedVector2mesh(Coord_Thrust(1,:),nT);
yT = sortedVector2mesh(Coord_Thrust(2,:),nT);
zT = sortedVector2mesh(Coord_Thrust(3,:),nT);


if config.plotThusts
    handlerThrust(1)  = surf(xT,yT,zT);
    colormap(thrustColor)
    set(handlerThrust(1),'EdgeColor',thrustColor,'LineStyle','none');
    
    handlerThrust(2)  = surf(xT,yT,zT);
    set(handlerThrust(2),'EdgeColor',thrustColor,'LineStyle','none');
    handlerThrust(3)  = surf(xT,yT,zT);
    set(handlerThrust(3),'EdgeColor',thrustColor,'LineStyle','none');
    handlerThrust(4)  = surf(xT,yT,zT);
    set(handlerThrust(4),'EdgeColor',thrustColor,'LineStyle','none');
    %%
    % store axes objects' handles to a vector
    config.plot_objs{1}=[lnkpatch';lin';pos';handlerThrust'];
    
else
    config.plot_objs{1}=[lnkpatch';lin';pos'];
end

if config.plotComTrajectories
    DeltaI = 50;
    stepI = 1;
    temp =  1:stepI:DeltaI;
    
    colorComTrajectory    = copper(length(temp));
    colorComDesTrajectory = bone(length(temp));
    
    for j = 1: length(temp)
        handlerTrajectoryCom(j)   = plot3([0 0],[0 0],[0 0],'-','Color',colorComTrajectory(j,:),'Linewidth',1);
        handlerTrajectoryComDes(j)= plot3([0 0],[0 0],[0 0],'-','Color',colorComDesTrajectory(j,:),'Linewidth',1);
    end
    config.plot_objs{1} = [config.plot_objs{1};handlerTrajectoryCom';handlerTrajectoryComDes'];
end

% copy all objects to other axes with different views
axes(config.plot_main(2));
config.plot_objs{2} = copyobj(config.plot_objs{1},config.plot_main(2));

%% WRITE INFO ON IMAGES
% text.time   = text(0.5,0.4,['$t = 0  [s]$'],'Color',text.color ,'Interpreter','LaTex','FontSize',text.fontSize);

%% SETTINGS FOR MAKING VIDEO
if config.visualiser.makeVideo
    aviobj = VideoWriter(config.visualiser.video.filename);%,'fps',1000/di);
    aviobj.Quality = 100;
    open(aviobj)
else
    aviobj = 0;
end

%% UPDATING THE PLOTS
for ii=2:n % the visualization instance
    %%The following for accounts for the two figures
    tic;  % visualizer step timer start (to setting the visualizer speed)
    
    for num_gigure=2:-1:1
        % Retrieving the handlers for the ithe figure
        lnkpatch       = config.plot_objs{num_gigure}(1:n_plot);
        
        lin            = config.plot_objs{num_gigure}(1+n_plot:n_plot+n_lin);
        
        pos            = config.plot_objs{num_gigure}(1+n_plot+n_lin:n_plot+n_lin+n_plot);
        
        %% UPDATING THRUST FORCES
        if config.plotThusts
            handlerThrust  = config.plot_objs{num_gigure}(1+n_plot+n_lin+n_plot:size(config.jets.frames,1)+n_plot+n_lin+n_plot);
            for ithJet = 1:size(config.jets.frames,1);
                ith_axis = abs(config.jets.axes(ithJet));
                ith_dir  = sign(config.jets.axes(ithJet));
                if num_gigure == 1
                    [w_p_b,w_R_b] = frame2posrot(squeeze(q(ii,:)'));
                else
                    [w_p_b,w_R_b] = frame2posrot(squeeze(q_noBpos(ii,:)'));
                end
                
                kinTmp = (wbm_forwardKinematics(w_R_b,w_p_b,qj(ii,:)',config.jets.frames{ithJet}))'; % forward kinematics for the list of joints/links
                
                [w_p_link,w_R_link] = frame2posrot(kinTmp);
                
                [xT, yT, zT] = cylinder(1.3*F+jetsIntensities(ii,ithJet)*tCy);
                CoordT_i = roty(pi/2)*fromMesh2sortedVector(xT,yT,zT);
                
                CoordT_i(ith_axis,:) = CoordT_i(ith_axis,:)*jetsIntensities(ii,ithJet) ;
                CoordT_iC = -ith_dir*w_R_link*CoordT_i + repmat(w_p_link', 1, size(CoordT_i,2));
                
                xTM = sortedVector2mesh(CoordT_iC(1,:),nT);
                yTM = sortedVector2mesh(CoordT_iC(2,:),nT);
                zTM = sortedVector2mesh(CoordT_iC(3,:),nT);
                
                set(handlerThrust(ithJet), 'xdata', xTM, 'ydata', yTM, 'zdata', zTM);
            end
        end
        
        %% UPDATING COM TRAJECTORIES
        if config.plotComTrajectories
            if num_gigure == 1
                handlerTrajectoryCom     = config.plot_objs{num_gigure}(1+size(config.jets.frames,1)+n_plot+n_lin+n_plot:...
                    size(config.jets.frames,1)+n_plot+n_lin+n_plot+length(temp));
                handlerTrajectoryComDes  = config.plot_objs{num_gigure}(1+size(config.jets.frames,1)+n_plot+n_lin+n_plot+length(temp):...
                    size(config.jets.frames,1)+n_plot+n_lin+n_plot+2*length(temp));
                j2 = 1;
                for j = max(stepI+1,ii-DeltaI+1):stepI:ii
                    set(handlerTrajectoryCom(j2),'xdata',comPosition(j-stepI:j,1),...
                        'ydata',comPosition(j-stepI:j,2),...
                        'zdata',comPosition(j-stepI:j,3),'Color',colorComTrajectory(j2,:));
                    set(handlerTrajectoryComDes(j2),'xdata',comDes(j-stepI:j,1),...
                        'ydata',comDes(j-stepI:j,2),...
                        'zdata',comDes(j-stepI:j,3),'Color',colorComDesTrajectory(j2,:));
                    j2 = j2 +1;
                end
            end
        end
        
        %% UPDATING MAIN BODY
        % Update joint position (* in the plots) and CoM
        for jj=1:n_plot
            if num_gigure == 1
                kin_tmp = kin(ii,:,jj);
            else
                kin_tmp = kin_noBpos(ii,:,jj);
            end
            [Ptemp,~] = frame2posrot(kin_tmp);
            x(jj) = Ptemp(1);
            y(jj) = Ptemp(2);
            z(jj) = Ptemp(3);
            set(pos(jj),xaxis,x(jj),yaxis,y(jj),zaxis,z(jj));
        end
        
        
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
            
            set(lnkpatch(jj),'vertices',xyzpatch.vertices);
        end
        
        %% UPDATE FEET
        % right foot
        jj=n_lin+1;
        
        vectlnk   =  [xyzpairs(4,2)-xyzpairs(4,1),xyzpairs(4,4)-xyzpairs(4,3),xyzpairs(4,6)-xyzpairs(4,5)];
        orthlnk   =  null(vectlnk);
        
        orthlnk1  =  mult_patch(4,1)*orthlnk(:,1);
        orthlnk2  =  mult_patch(4,2)*orthlnk(:,2);
        
        qq1 =  orthlnk1+2*orthlnk2;
        qq2 = -orthlnk1+2*orthlnk2;
        qq3 = -orthlnk1-orthlnk2;
        qq4 =  orthlnk1-orthlnk2;
        
        xyzpatch.vertices = [xyzpairs(4,2)+qq1(1)      , xyzpairs(4,4)+qq1(2) , xyzpairs(4,6)+qq1(3);
            xyzpairs(4,2)+qq2(1)      , xyzpairs(4,4)+qq2(2) , xyzpairs(4,6)+qq2(3);
            xyzpairs(4,2)+qq3(1)      , xyzpairs(4,4)+qq3(2) , xyzpairs(4,6)+qq3(3);
            xyzpairs(4,2)+qq4(1)      , xyzpairs(4,4)+qq4(2) , xyzpairs(4,6)+qq4(3);
            xyzpairs(4,2)+qq1(1)-0.04 , xyzpairs(4,4)+qq1(2) , xyzpairs(4,6)+qq1(3);
            xyzpairs(4,2)+qq2(1)-0.04 , xyzpairs(4,4)+qq2(2) , xyzpairs(4,6)+qq2(3);
            xyzpairs(4,2)+qq3(1)-0.04 , xyzpairs(4,4)+qq3(2) , xyzpairs(4,6)+qq3(3);
            xyzpairs(4,2)+qq4(1)-0.04 , xyzpairs(4,4)+qq4(2) , xyzpairs(4,6)+qq4(3)];
        
        set(lnkpatch(jj),'vertices',xyzpatch.vertices);
        
        % left foot
        jj=n_lin+2;
        
        vectlnk   =  [xyzpairs(7,2)-xyzpairs(7,1),xyzpairs(7,4)-xyzpairs(7,3),xyzpairs(7,6)-xyzpairs(7,5)];
        orthlnk   =  null(vectlnk);
        
        orthlnk1  =  mult_patch(7,1)*orthlnk(:,1);
        orthlnk2  =  mult_patch(7,2)*orthlnk(:,2);
        qq1 =  orthlnk1+2*orthlnk2;
        qq2 = -orthlnk1+2*orthlnk2;
        qq3 = -orthlnk1-orthlnk2;
        qq4 =  orthlnk1-orthlnk2;
        
        xyzpatch.vertices = [xyzpairs(7,2)+qq1(1)      , xyzpairs(7,4)+qq1(2) , xyzpairs(7,6)+qq1(3);
            xyzpairs(7,2)+qq2(1)      , xyzpairs(7,4)+qq2(2) , xyzpairs(7,6)+qq2(3);
            xyzpairs(7,2)+qq3(1)      , xyzpairs(7,4)+qq3(2) , xyzpairs(7,6)+qq3(3);
            xyzpairs(7,2)+qq4(1)      , xyzpairs(7,4)+qq4(2) , xyzpairs(7,6)+qq4(3);
            xyzpairs(7,2)+qq1(1)-0.04 , xyzpairs(7,4)+qq1(2) , xyzpairs(7,6)+qq1(3);
            xyzpairs(7,2)+qq2(1)-0.04 , xyzpairs(7,4)+qq2(2) , xyzpairs(7,6)+qq2(3);
            xyzpairs(7,2)+qq3(1)-0.04 , xyzpairs(7,4)+qq3(2) , xyzpairs(7,6)+qq3(3);
            xyzpairs(7,2)+qq4(1)-0.04 , xyzpairs(7,4)+qq4(2) , xyzpairs(7,6)+qq4(3)];
        
        
        set(lnkpatch(jj),'vertices',xyzpatch.vertices);
        
        %% UPDATE TEXT INFO ON IMAGES
        
        
        drawnow;
    end
    % to update the visualizer speed to keep it close to real simulation time
    if toc() < config.visualiser.timeStep  && toc() > 0.01
        pause(config.visualiser.timeStep-toc());
    end
    if config.visualiser.makeVideo == 1 && ii > 2
        saveFramesForVideo(config.visualiser.video.filename,config.figure_main,aviobj,1);
    end
end
if config.visualiser.makeVideo == 1
    saveFramesForVideo(config.visualiser.video.filename,config.figure_main,aviobj,90);
    close(aviobj);
end

end



