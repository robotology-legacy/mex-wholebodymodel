function visualizeForwardDynamics2(xout,params,jetsIntensities)
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

if nargin > 2
    
    params.plotThusts = true;
    
    % the list of link names that are used for identifying the jets positions                  
    params.jets.frames     = cell(4,1);
    params.jets.frames{1}  = 'l_hand_dh_frame';
    params.jets.frames{2}  = 'r_hand_dh_frame';
    params.jets.frames{3}  = 'l_foot_dh_frame';
    params.jets.frames{4}  = 'r_foot_dh_frame';
    
else
    
    params.plotThusts = false;
    
end

COLOR = [1 0.1 0];

n     = size(xout,1); % number of instances of the simulation results
q     = xout(:,1:7);  % first 3 elements provide the position and next 4 elements provide the orientation of the base 

q_noBpos         = q;
q_noBpos(:,1:3)  = repmat([0 0 0.7],size(q(:,1:3),1),1);

qj               = xout(:,8:32);  % joint positions

vis_speed        = 1;             % this variable is set to change the visualization speed, 
                                  % to make its speed close to the real time in case 
                                  % the simulation time step is changed.
   
% the list of link/joint names that are used to construct the robot in the visualizer                  
L     = cell(15,1);
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

% since for now the world reference frame is that of the codyco_balancing_world, 
% the z-axis is the vertical axis for the robot.
xaxis = 'xdata';
yaxis = 'ydata';
zaxis = 'zdata';

n_plot = 15; % number of points to be plotted (virtual joints)
n_lin  = 13;  % number of lines to be plotted (virtual links)

kin         =  zeros(size(xout,1),7,n_plot);
kin_noBpos  =  zeros(size(xout,1),7,n_plot);

for jj=1:n_plot
    
    for ii=1:n % at each instance
             
       % convert base state to rotation
       [pB,R]               = frame2posrot(squeeze(q(ii,:)'));
       [pB_noBpos,R_noBpos] = frame2posrot(squeeze(q_noBpos(ii,:)'));

       % set world to base
       wbm_setWorldFrame(R,pB,[0,0,-9.81]');
       kin(ii,:,jj)        = (wbm_forwardKinematics(R,pB,qj(ii,:)',L{jj}))'; % forward kinematics for the list of joints/links
       kin_noBpos(ii,:,jj) = (wbm_forwardKinematics(R_noBpos,pB_noBpos,qj(ii,:)',L{jj}))'; % forward kinematics for the list of joints/links
       
    end

end

kin(:,:,1) = q; % use base data instead of fwdkin rootlink

% clear and reset the plots

for ii=2:-1:1 
    
    axes(params.plot_main(ii));
    axis([-0.5 0.5 -0.42 0.58 0 1]);
    set(gca,'XGrid','off','YGrid','off','ZGrid','off','XTick', [],'YTick', [],'ZTick', []);  
    set(gca,'LineWidth',5);
    set(gca, 'drawmode', 'fast');
    cla;
    view([45 25 25])
    drawnow
    
end

axes(params.plot_main(1));

%% INITIAL PLOTS

% allocate memory
x = zeros(1,n_plot);
y = zeros(1,n_plot);
z = zeros(1,n_plot);

pos      = zeros(1,n_plot);
xyzpairs = zeros(n_lin,6);

% plot the base position

x(1)   = kin(1,1,1);
y(1)   = kin(1,2,1);
z(1)   = kin(1,3,1);
pos(1) = plot3(x(1),y(1),z(1),'w.');

% plot the joints
for jj=2:n_plot-1
    
    [Ptemp,~] = frame2posrot(kin(1,:,jj)');
    x(jj)     = Ptemp(1);
    y(jj)     = Ptemp(2);
    z(jj)     = Ptemp(3);
   
    col       = 'w.';
    pos(jj)   = plot3(x(jj),y(jj),z(jj),col);
    
end

% plot the position of the center of mass
jj            = n_plot;

    [Ptemp,~] = frame2posrot(kin(1,:,jj)');
    x(jj)     = Ptemp(1);
    y(jj)     = Ptemp(2);
    z(jj)     = Ptemp(3);
 
pos(jj)       = plot3(x(jj),y(jj),z(jj),'g*');

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
lin               = zeros(1,n_lin);
lnkpatch          = zeros(1,n_lin);
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
    
    lin(jj)  = line(xaxis,xyzpairs(jj,1:2),yaxis,xyzpairs(jj,3:4),zaxis,xyzpairs(jj,5:6),'erasemode','normal','linewidth',3,'color','red');
    
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
            

    xyzpatch.faces = [ 1 2 3 4;
                       1 4 8 5;
                       5 8 7 6;
                       7 3 2 6;
                       2 6 5 1;
                       3 7 8 4];

    lnkpatch(jj)   = patch('vertices',xyzpatch.vertices,'faces',xyzpatch.faces,...
                           'FaceAlpha',0.2,'FaceColor',COLOR);
                       
end

%% FEET PATCHES
% right foot patch
jj       =  n_lin+1; 

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
               'FaceColor',COLOR);

% left foot patch
jj       =  n_lin+2; 


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

lnkpatch(jj) = patch('vertices',xyzpatch.vertices,'faces',xyzpatch.faces,'FaceAlpha',0.2,...
               'FaceColor',COLOR);
           
%% PLOT THRUSTS
if params.plotThusts

    F      = 0.01;
    delta  = 5*pi/180;
    nT     = 2;
    tCy    = 0:0.01:tan(delta);
    
    [xT, yT, zT] = cylinder(1.2*F.*(1+tCy));
    Coord_Thrust = roty(-pi/2)*fromMesh2sortedVector(xT,yT,zT);
    xT           = sortedVector2mesh(Coord_Thrust(1,:),nT);
    yT           = sortedVector2mesh(Coord_Thrust(2,:),nT);
    zT           = sortedVector2mesh(Coord_Thrust(3,:),nT);

    hThI(1)      = surf(xT,yT,zT);
    set(hThI(1),'EdgeColor',[1 0 0],'LineStyle','none');  

    hold on;
    hThI(2)  = surf(xT,yT,zT);
    set(hThI(2),'EdgeColor',[1 0 0],'LineStyle','none');  
    hThI(3)  = surf(xT,yT,zT);
    set(hThI(3),'EdgeColor',[1 0 0],'LineStyle','none');  
    hThI(4)  = surf(xT,yT,zT);
    set(hThI(4),'EdgeColor',[1 0 0],'LineStyle','none');  
    hold off
    
end
   
% store axes objects' handles to a vector
params.plot_objs{1} = [lnkpatch';lin';pos'];%;plot_traj];

% copy all objects to other axes with different views
axes(params.plot_main(2));
params.plot_objs{2} = copyobj(params.plot_objs{1},params.plot_main(2));

if params.plotThusts

    hThB = copyobj(hThI,params.plot_main(2));
    
end

%% UPDATING THE PLOTS
ii=2;        

    while ii<n+1 % the visualization instance
        
        for num_gigure=2:-1:1
        
            lnkpatch = params.plot_objs{num_gigure}(1:n_plot);
            lin      = params.plot_objs{num_gigure}(1+n_plot:1+n_plot+n_lin);
            pos      = params.plot_objs{num_gigure}(1+n_plot+n_lin:n_plot+n_lin+n_plot);
            
            tic;  % visualizer step timer start (to setting the visualizer speed)

            %% PLOTTING THRUST FORCES
          
            if params.plotThusts
                
                for ithJet        = 1:size(params.jets.frames,1);
                    
                    ith_axis      = abs(params.jets.axes(ithJet));
                    ith_dir       = sign(params.jets.axes(ithJet));
                    [w_p_b,w_R_b] = frame2posrot(squeeze(q(ii,:)'));

                    kinTmp = (wbm_forwardKinematics(w_R_b,w_p_b,qj(ii,:)',params.jets.frames{ithJet}))'; % forward kinematics for the list of joints/links

                    [w_p_link,w_R_link] = frame2posrot(kinTmp);

                    [xT, yT, zT] = cylinder(1.3*F+jetsIntensities(ii,ithJet)*tCy);
                    CoordT_i     = roty(pi/2)*fromMesh2sortedVector(xT,yT,zT);

                    CoordT_i(ith_axis,:) = CoordT_i(ith_axis,:)*jetsIntensities(ii,ithJet) ;
                    CoordT_iC            = -ith_dir*w_R_link*CoordT_i + repmat(w_p_link', 1, size(CoordT_i,2));

                    xTM = sortedVector2mesh(CoordT_iC(1,:),nT);
                    yTM = sortedVector2mesh(CoordT_iC(2,:),nT);
                    zTM = sortedVector2mesh(CoordT_iC(3,:),nT);

                    set(hThI(ithJet), 'xdata', xTM, 'ydata', yTM, 'zdata', zTM); 
                  
                    [w_p_b,w_R_b] = frame2posrot(squeeze(q_noBpos(ii,:)'));

                    kinTmp        = (wbm_forwardKinematics(w_R_b,w_p_b,qj(ii,:)',params.jets.frames{ithJet}))'; % forward kinematics for the list of joints/links

                    [w_p_link,w_R_link] = frame2posrot(kinTmp);

                    CoordT_iC           = -ith_dir*w_R_link*CoordT_i + repmat(w_p_link', 1, size(CoordT_i,2));

                    xTM = sortedVector2mesh(CoordT_iC(1,:),nT);
                    yTM = sortedVector2mesh(CoordT_iC(2,:),nT);
                    zTM = sortedVector2mesh(CoordT_iC(3,:),nT);

                    set(hThB(ithJet), 'xdata', xTM, 'ydata', yTM, 'zdata', zTM);
                    
                end
            end
            
%% UPDATE MAIN BODY

% get the positions for the current instance
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
jj = n_lin+1; 
            
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
                     xyzpairs(4,2)+qq1(1)+0.03 , xyzpairs(4,4)+qq1(2) , xyzpairs(4,6)+qq1(3);
                     xyzpairs(4,2)+qq2(1)+0.03 , xyzpairs(4,4)+qq2(2) , xyzpairs(4,6)+qq2(3);
                     xyzpairs(4,2)+qq3(1)+0.03 , xyzpairs(4,4)+qq3(2) , xyzpairs(4,6)+qq3(3);
                     xyzpairs(4,2)+qq4(1)+0.03 , xyzpairs(4,4)+qq4(2) , xyzpairs(4,6)+qq4(3)];

                 
set(lnkpatch(jj),'vertices',xyzpatch.vertices);
            
% left foot
jj = n_lin+2; 

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
                     xyzpairs(7,2)+qq1(1)+0.03 , xyzpairs(7,4)+qq1(2) , xyzpairs(7,6)+qq1(3);
                     xyzpairs(7,2)+qq2(1)+0.03 , xyzpairs(7,4)+qq2(2) , xyzpairs(7,6)+qq2(3);
                     xyzpairs(7,2)+qq3(1)+0.03 , xyzpairs(7,4)+qq3(2) , xyzpairs(7,6)+qq3(3);
                     xyzpairs(7,2)+qq4(1)+0.03 , xyzpairs(7,4)+qq4(2) , xyzpairs(7,6)+qq4(3)];
                             
set(lnkpatch(jj),'vertices',xyzpatch.vertices);

% store axes objects to a vector
params.plot_objs{num_gigure} = [lnkpatch;lin;pos];%;plot_traj]; 

drawnow;

% to update the visualizer speed to keep it close to real simulation time
time_dif = vis_speed*params.sim_step-toc();
            
if time_dif>0
            
                pause(time_dif);
                
else
   
                vis_speed = vis_speed+1;
                
end
               
        end
           
ii = ii+vis_speed;
         
    end
       
end
