function visualize_forwardDyn(xout,params)
%% visalize_forwardDyn 
%   Visualize the simulation results obtained from integration 
%   of the forward dynamics of the iCub. 
% 
%   xout -  The output vector of the integration. Containing [qbase qjoints
%           vbase vjoint] along a time span.
%   params - 

tSpan   = linspace(  params.sim_start_time,  params.sim_start_time+params.sim_duration,  params.sim_duration/params.sim_step);

n = size(xout,1);
q=xout(:,1:7);
qj=xout(:,8:32);

draw_meshes = 0;
vis_speed = 1;

L = cell(15,1);
L{1} = 'root_link';
L{2} = 'r_hip_1';
% L{3} = 'r_shank';
L{3} = 'r_thigh';
L{4} = 'r_sole';
L{5} = 'l_hip_1';
% L{6} = 'l_shank';
L{6} = 'l_thigh';
L{7} = 'l_sole';
% L{8} = 'neck_1';
L{8} = 'chest';
L{9} = 'r_shoulder_1';
L{10} = 'r_elbow_1';
L{11} = 'r_gripper';
L{12} = 'l_shoulder_1';
L{13} = 'l_elbow_1';
L{14} = 'l_gripper';
L{15} = 'com';

xaxis = 'zdata';
yaxis = 'ydata';
zaxis = 'xdata';

n_plot = 15;
n_lin = 13;

kin = zeros(size(xout,1),7,n_plot);
for jj=2:n_plot
    %     kin(:,:,jj)=zeros(n,7);
    for ii=1:n
        kin(ii,:,jj) = (wholeBodyModel('forward-kinematics',qj(ii,:)',L{jj}))';
    end
end

kin(:,:,1)= q; %use base data instead of fwdkin rootlink

for ii=4:-1:1
    axes(params.plot_main(ii));
        cla;
%     set(gca,'Color',[0.8 0.8 0.8]);
%     set(gca,'XColor',[0.8 0.8 0.8]);
%     set(gca,'YColor',[0.8 0.8 0.8]);
%     set(gca,'ZColor',[0.8 0.8 0.8]);
%     set(gca,'xdir','reverse')
%     set(gca, 'drawmode', 'fast');
%     rotate3d(gca,'on');
% 
% %     plot_main(ii) = subplot('Position', plot_pos(ii,:));
% %     delete(params.plot_objs(ii));
% %     params.plot_objs(ii)=plot3(0,0,0,'.');
% %     axis([-0.425 0.575 -0.5 0.5 0 1]); hold on;
%     patch([-0.425 -0.425 0.575 0.575],[-0.5 0.5 0.5 -0.5],[0 0 0 0],[0.6 0.6 0.8]);
    patch([-0.45 -0.45 0.45 0.45],[-0.37 0.53 0.53 -0.37],[0 0 0 0],[0.6 0.6 0.8]);
%     
%     figure(figure_main);
drawnow
% pause
end


axes(params.plot_main(1));
% get(gca,'Children')


% plot3(0,0,0,'.');
% axis([-0.425 0.575 -0.5 0.5 0 1]); hold on;
% patch([-0.425 -0.425 0.575 0.575],[-0.5 0.5 0.5 -0.5],[0 0 0 0],[0.6 0.6 0.8],'FaceAlpha',0.8);
% 
% set(gca,'Color',[0.8 0.8 0.8]);
% set(gca,'XColor',[0.8 0.8 0.8]);
% set(gca,'YColor',[0.8 0.8 0.8]);
% set(gca,'ZColor',[0.8 0.8 0.8]);
% set(gca,'xdir','reverse')
% set(gca, 'drawmode', 'fast');

traj_dat = zeros(size(tSpan,1),3);
ll=1;
for kk = tSpan
    temp = params.controller.Desired_x_dx_ddx_CoM(kk);
    traj_dat(ll,:) = temp(:,1)';ll=ll+1;
end

%%% INITIAL PLOTS
x = zeros(1,n_plot);
y = zeros(1,n_plot);
z = zeros(1,n_plot);
R = zeros(3,3,n_plot);
pos = zeros(1,n_plot);
xyzpairs = zeros(n_lin,6);

% if params.draw_init == 1
plot_traj = plot3(traj_dat(:,1),traj_dat(:,2),traj_dat(:,3),'r');

% root_link
x(1)=kin(1,1,1);y(1)=kin(1,2,1);z(1)=kin(1,3,1);
pos(1)=plot3(x(1),y(1),z(1),'w.');

% joints
for jj=2:n_plot-1
    
    [x(jj),y(jj),z(jj),R(:,:,jj)] = quat2rot(kin(1,:,jj));
    pos(jj)=plot3(x(jj),y(jj),z(jj),'w.');
    
end
% COM
jj=n_plot;
[x(jj),y(jj),z(jj),R(:,:,jj)] = quat2rot(kin(1,:,jj));
pos(jj)=plot3(x(jj),y(jj),z(jj),'g*');

xyzpairs(1,:) = [x(1) x(8) y(1) y(8) z(1) z(8)];
xyzpairs(2,:) = [x(1) x(2) y(1) y(2) z(1) z(2)];
xyzpairs(3,:) = [x(3) x(2) y(3) y(2) z(3) z(2)];
xyzpairs(4,:) = [x(3) x(4) y(3) y(4) z(3) z(4)];
xyzpairs(5,:) = [x(1) x(5) y(1) y(5) z(1) z(5)];
xyzpairs(6,:) = [x(5) x(6) y(5) y(6) z(5) z(6)];
xyzpairs(7,:) = [x(6) x(7) y(6) y(7) z(6) z(7)];
xyzpairs(8,:) = [x(9) x(8) y(9) y(8) z(9) z(8)];
xyzpairs(9,:) = [x(9) x(10) y(9) y(10) z(9) z(10)];
xyzpairs(10,:) = [x(10) x(11) y(10) y(11) z(10) z(11)];
xyzpairs(11,:) = [x(8) x(12) y(8) y(12) z(8) z(12)];
xyzpairs(12,:) = [x(13) x(12) y(13) y(12) z(13) z(12)];
xyzpairs(13,:) = [x(13) x(14) y(13) y(14) z(13) z(14)];


lin = zeros(1,n_lin);
lnkpatch = zeros(1,n_lin);
xyzpatch.vertices = zeros(8,3);
xyzpatch.faces = zeros(6,4);

mult_patch = [0.07,0.03 ;
              0.04,0.02 ;
              0.03,0.02 ;
              0.025,0.02 ;
              0.04,0.02 ;
              0.03,0.02 ;
              0.025,0.02 ;
              0.03,0.02 ;
              0.025,0.02 ;
              0.02,0.02 ;
              0.03,0.02 ;
              0.025,0.02 ;
              0.02,0.02 ];

for jj=1:n_lin
    
    lin(jj) = line(xaxis,xyzpairs(jj,1:2),yaxis,xyzpairs(jj,3:4),zaxis,xyzpairs(jj,5:6),'erasemode','normal','linewidth',3,'color','black');
    
    % for patch
    vectlnk= [xyzpairs(jj,2)-xyzpairs(jj,1),xyzpairs(jj,4)-xyzpairs(jj,3),xyzpairs(jj,6)-xyzpairs(jj,5)];
    orthlnk= null(vectlnk);
    orthlnk1 = mult_patch(jj,1)*orthlnk(:,1);
    orthlnk2 = mult_patch(jj,2)*orthlnk(:,2);
    qq1 = orthlnk1+orthlnk2;
    qq2 = -orthlnk1+orthlnk2;
    qq3 = -orthlnk1-orthlnk2;
    qq4 = orthlnk1-orthlnk2;
    
    
    
    xyzpatch.vertices = [xyzpairs(jj,2)+qq1(1) , xyzpairs(jj,4)+qq1(2) , xyzpairs(jj,6)+qq1(3);
                xyzpairs(jj,2)+qq2(1) , xyzpairs(jj,4)+qq2(2) , xyzpairs(jj,6)+qq2(3);
                xyzpairs(jj,2)+qq3(1) , xyzpairs(jj,4)+qq3(2) , xyzpairs(jj,6)+qq3(3);
                xyzpairs(jj,2)+qq4(1) , xyzpairs(jj,4)+qq4(2) , xyzpairs(jj,6)+qq4(3);              
                xyzpairs(jj,1)+qq1(1) , xyzpairs(jj,3)+qq1(2) , xyzpairs(jj,5)+qq1(3);
                xyzpairs(jj,1)+qq2(1) , xyzpairs(jj,3)+qq2(2) , xyzpairs(jj,5)+qq2(3);
                xyzpairs(jj,1)+qq3(1) , xyzpairs(jj,3)+qq3(2) , xyzpairs(jj,5)+qq3(3);
                xyzpairs(jj,1)+qq4(1) , xyzpairs(jj,3)+qq4(2) , xyzpairs(jj,5)+qq4(3)];
            
            
    temp = xyzpatch.vertices(:,1);        
    xyzpatch.vertices(:,1) = xyzpatch.vertices(:,3);
    xyzpatch.vertices(:,3) = temp;
    
    xyzpatch.faces = [ 1 2 3 4;
                        1 4 8 5;
                        5 8 7 6;
                        7 3 2 6;
                        2 6 5 1;
                        3 7 8 4];

    lnkpatch(jj) = patch('vertices',xyzpatch.vertices,'faces',xyzpatch.faces,'FaceAlpha',0.2);
    % end for patch
end
%%% for the feet

% feet patches    
jj=n_lin+1; %rightfeet

orthlnk1 = [0 0.03 0]';
orthlnk2 = [0 0 0.03]';

qq1 = orthlnk1+2*orthlnk2;
qq2 = -orthlnk1+2*orthlnk2;
qq3 = -orthlnk1-orthlnk2;
qq4 = orthlnk1-orthlnk2;

xyzpatch.vertices = [xyzpairs(4,2)+qq1(1) , xyzpairs(4,4)+qq1(2) , xyzpairs(4,6)+qq1(3);
    xyzpairs(4,2)+qq2(1) , xyzpairs(4,4)+qq2(2) , xyzpairs(4,6)+qq2(3);
    xyzpairs(4,2)+qq3(1) , xyzpairs(4,4)+qq3(2) , xyzpairs(4,6)+qq3(3);
    xyzpairs(4,2)+qq4(1) , xyzpairs(4,4)+qq4(2) , xyzpairs(4,6)+qq4(3);
    xyzpairs(4,2)+qq1(1)+0.03 , xyzpairs(4,4)+qq1(2) , xyzpairs(4,6)+qq1(3);
    xyzpairs(4,2)+qq2(1)+0.03 , xyzpairs(4,4)+qq2(2) , xyzpairs(4,6)+qq2(3);
    xyzpairs(4,2)+qq3(1)+0.03 , xyzpairs(4,4)+qq3(2) , xyzpairs(4,6)+qq3(3);
    xyzpairs(4,2)+qq4(1)+0.03 , xyzpairs(4,4)+qq4(2) , xyzpairs(4,6)+qq4(3)];

temp = xyzpatch.vertices(:,1);        
    xyzpatch.vertices(:,1) = xyzpatch.vertices(:,3);
    xyzpatch.vertices(:,3) = temp;

lnkpatch(jj) = patch('vertices',xyzpatch.vertices,'faces',xyzpatch.faces,'FaceAlpha',0.2);

jj=n_lin+2; %leftfeet


orthlnk1 = [0 0.03 0]';
orthlnk2 = [0 0 0.03]';

qq1 = orthlnk1+2*orthlnk2;
qq2 = -orthlnk1+2*orthlnk2;
qq3 = -orthlnk1-orthlnk2;
qq4 = orthlnk1-orthlnk2;

xyzpatch.vertices = [xyzpairs(7,2)+qq1(1) , xyzpairs(7,4)+qq1(2) , xyzpairs(7,6)+qq1(3);
    xyzpairs(7,2)+qq2(1) , xyzpairs(7,4)+qq2(2) , xyzpairs(7,6)+qq2(3);
    xyzpairs(7,2)+qq3(1) , xyzpairs(7,4)+qq3(2) , xyzpairs(7,6)+qq3(3);
    xyzpairs(7,2)+qq4(1) , xyzpairs(7,4)+qq4(2) , xyzpairs(7,6)+qq4(3);
    xyzpairs(7,2)+qq1(1)+0.03 , xyzpairs(7,4)+qq1(2) , xyzpairs(7,6)+qq1(3);
    xyzpairs(7,2)+qq2(1)+0.03 , xyzpairs(7,4)+qq2(2) , xyzpairs(7,6)+qq2(3);
    xyzpairs(7,2)+qq3(1)+0.03 , xyzpairs(7,4)+qq3(2) , xyzpairs(7,6)+qq3(3);
    xyzpairs(7,2)+qq4(1)+0.03 , xyzpairs(7,4)+qq4(2) , xyzpairs(7,6)+qq4(3)];

temp = xyzpatch.vertices(:,1);        
    xyzpatch.vertices(:,1) = xyzpatch.vertices(:,3);
    xyzpatch.vertices(:,3) = temp;
lnkpatch(jj) = patch('vertices',xyzpatch.vertices,'faces',xyzpatch.faces,'FaceAlpha',0.2);


%%%

lin_mirror = zeros(1,n_lin);
for jj=1:n_lin
    lin_mirror(jj) = line(xaxis,-0.5*xyzpairs(jj,1:2),yaxis,xyzpairs(jj,3:4),zaxis,xyzpairs(jj,5:6),'erasemode','normal','linewidth',2,'color',[0.5 0.5 0.5]);
end

ii=1;
for jj=1:n_plot
    [x(jj),y(jj),z(jj),R(:,:,jj)] = quat2rot(kin(ii,:,jj));
    set(pos(jj),xaxis,x(jj),yaxis,y(jj),zaxis,z(jj));
end

% copy objects to other views

% copyobj([lnkpatch';lin'],params.plot_main(2));
% view(3);
% copyobj([lnkpatch';lin'],params.plot_main(3));
% view(3);
% copyobj([lnkpatch';lin'],params.plot_main(4));
% view(3);
% delete(params.plot_objs(2));    
% params.plot_objs(2) = copyobj([lnkpatch';lin'],params.plot_main(2));
% view(3);
% delete(params.plot_objs(3));
% params.plot_objs(3) = copyobj([lnkpatch';lin'],params.plot_main(3));
% view(3);
% delete(params.plot_objs(4));    
% params.plot_objs(4) = copyobj([lnkpatch';lin'],params.plot_main(4));
% view(3);


if exist('params.plot_objs')    
    delete(params.plot_objs{2});
    delete(params.plot_objs{3});
    delete(params.plot_objs{4});    
end
    

params.plot_objs{1}=[lnkpatch';lin';pos';lin_mirror';plot_traj];

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











% ADD MESHES
% %     if draw_meshes==1
% %         % h_patch(1) = patch(stlread('icub_head.stl'));
% %         h_patch(1) = patch(stlread('icub_head.stl'),'FaceColor',       [0.5 0.8 1.0], ...
% %             'EdgeColor',       'none',        ...
% %             'FaceLighting',    'gouraud',     ...
% %             'AmbientStrength', 0.15, ...
% %             'Visible','on');
% %         p_add1 = ones(3,size(get(h_patch(1),'xdata'),2));
% %
% %         rotate(h_patch(1),[0,0,1],90,[0 0 0]);
% %
% %         x1i = get(h_patch(1),xaxis);
% %         y1i = get(h_patch(1),yaxis);
% %         z1i = get(h_patch(1),zaxis);
% %
% %         set(h_patch(1),xaxis,x1i+p_add1*x(8));
% %         set(h_patch(1),yaxis,y1i+p_add1*y(8));
% %         set(h_patch(1),zaxis,z1i+p_add1*z(8));
% %
% %         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %         % h_patch(2) = patch(stlread('icub_chest.stl'));
% %         h_patch(2) = patch(stlread('icub_chest.stl'),'FaceColor',       [0.5 0.8 1.0], ...
% %             'EdgeColor',       'none',        ...
% %             'FaceLighting',    'gouraud',     ...
% %             'AmbientStrength', 0.15, ...
% %             'Visible','on');
% %         p_add2 = ones(3,size(get(h_patch(2),'xdata'),2));
% %
% %         rotate(h_patch(2),[0,0,1],90,[0 0 0]);
% %         rotate(h_patch(2),[1,0,0],180,[0 0 0]);
% %         set(h_patch(2),xaxis,get(h_patch(2),xaxis)-p_add2*0.1);
% %
% %         x2i = get(h_patch(2),xaxis);
% %         y2i = get(h_patch(2),yaxis);
% %         z2i = get(h_patch(2),zaxis);
% %
% %         vertic1 = get(h_patch(2),'vertices');
% %
% %         set(h_patch(2),xaxis,x2i+p_add2*(x(1)+x(8))*0.5);
% %         set(h_patch(2),yaxis,y2i+p_add2*(y(1)+y(8))*0.5);
% %         set(h_patch(2),zaxis,z2i+p_add2*(z(1)+z(8))*0.5);
% %
% %         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %         % h_patch(3) = patch(stlread('icub_lap_belt_1.stl'));
% %         h_patch(3) = patch(stlread('icub_lap_belt_1.stl'),'FaceColor',       [0.5 0.8 1.0], ...
% %             'EdgeColor',       'none',        ...
% %             'FaceLighting',    'gouraud',     ...
% %             'AmbientStrength', 0.15, ...
% %             'Visible','on');
% %         p_add3 = ones(3,size(get(h_patch(3),'xdata'),2));
% %
% %         rotate(h_patch(3),[0,1,0],-90,[0 0 0]);
% %         rotate(h_patch(3),[0,0,1],-90,[0 0 0]);
% %
% %         x3i = get(h_patch(3),xaxis);
% %         y3i = get(h_patch(3),yaxis);
% %         z3i = get(h_patch(3),zaxis);
% %
% %
% %         set(h_patch(3),xaxis,x3i+p_add3*(x(2)+x(5))*0.5);
% %         set(h_patch(3),yaxis,y3i+p_add3*(y(2)+y(5))*0.5);
% %         set(h_patch(3),zaxis,z3i+p_add3*(z(2)+z(5))*0.5);
% %
% %         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %         % h_patch(4) = patch(stlread('icub_r_foot.stl'));
% %         h_patch(4) = patch(stlread('icub_r_foot.stl'),'FaceColor',       [0.5 0.8 1.0], ...
% %             'EdgeColor',       'none',        ...
% %             'FaceLighting',    'gouraud',     ...
% %             'AmbientStrength', 0.15, ...
% %             'Visible','on');
% %
% %         p_add4 = ones(3,size(get(h_patch(4),'xdata'),2));
% %
% %         rotate(h_patch(4),[0,1,0],-90,[0 0 0]);
% %         % rotate(h_patch(4),[0,0,1],-90,[0 0 0]);
% %
% %         x4i = get(h_patch(4),xaxis);
% %         y4i = get(h_patch(4),yaxis);
% %         z4i = get(h_patch(4),zaxis);
% %
% %         set(h_patch(4),xaxis,x4i+p_add4*(x(4)));
% %         set(h_patch(4),yaxis,y4i+p_add4*(y(4)));
% %         set(h_patch(4),zaxis,z4i+p_add4*(z(4)));
% %
% %         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %         % h_patch(5) = patch(stlread('icub_l_foot.stl'));
% %         h_patch(5) = patch(stlread('icub_l_foot.stl'),'FaceColor',       [0.5 0.8 1.0], ...
% %             'EdgeColor',       'none',        ...
% %             'FaceLighting',    'gouraud',     ...
% %             'AmbientStrength', 0.15, ...
% %             'Visible','on');
% %         camlight('headlight');
% %         material('dull');
% %         p_add5 = ones(3,size(get(h_patch(5),'xdata'),2));
% %
% %         rotate(h_patch(5),[0,1,0],-90,[0 0 0]);
% %         rotate(h_patch(5),[0,0,1],180,[0 0 0]);
% %
% %         x5i = get(h_patch(5),xaxis);
% %         y5i = get(h_patch(5),yaxis);
% %         z5i = get(h_patch(5),zaxis);
% %
% %
% %         set(h_patch(5),xaxis,x5i+p_add4*(x(7)));
% %         set(h_patch(5),yaxis,y5i+p_add4*(y(7)));
% %         set(h_patch(5),zaxis,z5i+p_add4*(z(7)));
% %     end
% %     % END INIT_DRAW MESHES

% params.draw_init = 0;





% CONTINOUS PLOT

set(plot_traj,xaxis,traj_dat(:,1),yaxis,traj_dat(:,2),zaxis,traj_dat(:,3));
ii=1;
while ii<n+1
    tic;  % visualizer step time start
    for jj=1:n_plot
        [x(jj),y(jj),z(jj),R(:,:,jj)] = quat2rot(kin(ii,:,jj));
        set(pos(jj),xaxis,[x(jj)],yaxis,[y(jj)],zaxis,[z(jj)]);
        
    end
    
    if x(8)<=0
        break;
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
    
    
    for jj=1:n_lin
        set(lin(jj),xaxis,xyzpairs(jj,1:2),yaxis,xyzpairs(jj,3:4),zaxis,xyzpairs(jj,5:6));
        
        vectlnk= [xyzpairs(jj,2)-xyzpairs(jj,1),xyzpairs(jj,4)-xyzpairs(jj,3),xyzpairs(jj,6)-xyzpairs(jj,5)];
        orthlnk= null(vectlnk);
        orthlnk1 = mult_patch(jj,1)*orthlnk(:,1);
        orthlnk2 = mult_patch(jj,2)*orthlnk(:,2);
        qq1 = orthlnk1+orthlnk2;
        qq2 = -orthlnk1+orthlnk2;
        qq3 = -orthlnk1-orthlnk2;
        qq4 = orthlnk1-orthlnk2;
    
    
    
    xyzpatch.vertices = [xyzpairs(jj,2)+qq1(1) , xyzpairs(jj,4)+qq1(2) , xyzpairs(jj,6)+qq1(3);
                xyzpairs(jj,2)+qq2(1) , xyzpairs(jj,4)+qq2(2) , xyzpairs(jj,6)+qq2(3);
                xyzpairs(jj,2)+qq3(1) , xyzpairs(jj,4)+qq3(2) , xyzpairs(jj,6)+qq3(3);
                xyzpairs(jj,2)+qq4(1) , xyzpairs(jj,4)+qq4(2) , xyzpairs(jj,6)+qq4(3);              
                xyzpairs(jj,1)+qq1(1) , xyzpairs(jj,3)+qq1(2) , xyzpairs(jj,5)+qq1(3);
                xyzpairs(jj,1)+qq2(1) , xyzpairs(jj,3)+qq2(2) , xyzpairs(jj,5)+qq2(3);
                xyzpairs(jj,1)+qq3(1) , xyzpairs(jj,3)+qq3(2) , xyzpairs(jj,5)+qq3(3);
                xyzpairs(jj,1)+qq4(1) , xyzpairs(jj,3)+qq4(2) , xyzpairs(jj,5)+qq4(3)];
            
            
    temp = xyzpatch.vertices(:,1);        
    xyzpatch.vertices(:,1) = xyzpatch.vertices(:,3);
    xyzpatch.vertices(:,3) = temp;
    
    set(lnkpatch(jj),'vertices',xyzpatch.vertices);
    
    end
% feet patches    
jj=n_lin+1; %rightfeet

orthlnk1 = [0 0.03 0]';
orthlnk2 = [0 0 0.03]';

qq1 = orthlnk1+2*orthlnk2;
qq2 = -orthlnk1+2*orthlnk2;
qq3 = -orthlnk1-orthlnk2;
qq4 = orthlnk1-orthlnk2;

xyzpatch.vertices = [xyzpairs(4,2)+qq1(1) , xyzpairs(4,4)+qq1(2) , xyzpairs(4,6)+qq1(3);
    xyzpairs(4,2)+qq2(1) , xyzpairs(4,4)+qq2(2) , xyzpairs(4,6)+qq2(3);
    xyzpairs(4,2)+qq3(1) , xyzpairs(4,4)+qq3(2) , xyzpairs(4,6)+qq3(3);
    xyzpairs(4,2)+qq4(1) , xyzpairs(4,4)+qq4(2) , xyzpairs(4,6)+qq4(3);
    xyzpairs(4,2)+qq1(1)+0.03 , xyzpairs(4,4)+qq1(2) , xyzpairs(4,6)+qq1(3);
    xyzpairs(4,2)+qq2(1)+0.03 , xyzpairs(4,4)+qq2(2) , xyzpairs(4,6)+qq2(3);
    xyzpairs(4,2)+qq3(1)+0.03 , xyzpairs(4,4)+qq3(2) , xyzpairs(4,6)+qq3(3);
    xyzpairs(4,2)+qq4(1)+0.03 , xyzpairs(4,4)+qq4(2) , xyzpairs(4,6)+qq4(3)];

temp = xyzpatch.vertices(:,1);        
    xyzpatch.vertices(:,1) = xyzpatch.vertices(:,3);
    xyzpatch.vertices(:,3) = temp;

set(lnkpatch(jj),'vertices',xyzpatch.vertices);

jj=n_lin+2; % left feet

orthlnk1 = [0 0.03 0]';
orthlnk2 = [0 0 0.03]';

qq1 = orthlnk1+2*orthlnk2;
qq2 = -orthlnk1+2*orthlnk2;
qq3 = -orthlnk1-orthlnk2;
qq4 = orthlnk1-orthlnk2;

xyzpatch.vertices = [xyzpairs(7,2)+qq1(1) , xyzpairs(7,4)+qq1(2) , xyzpairs(7,6)+qq1(3);
    xyzpairs(7,2)+qq2(1) , xyzpairs(7,4)+qq2(2) , xyzpairs(7,6)+qq2(3);
    xyzpairs(7,2)+qq3(1) , xyzpairs(7,4)+qq3(2) , xyzpairs(7,6)+qq3(3);
    xyzpairs(7,2)+qq4(1) , xyzpairs(7,4)+qq4(2) , xyzpairs(7,6)+qq4(3);
    xyzpairs(7,2)+qq1(1)+0.03 , xyzpairs(7,4)+qq1(2) , xyzpairs(7,6)+qq1(3);
    xyzpairs(7,2)+qq2(1)+0.03 , xyzpairs(7,4)+qq2(2) , xyzpairs(7,6)+qq2(3);
    xyzpairs(7,2)+qq3(1)+0.03 , xyzpairs(7,4)+qq3(2) , xyzpairs(7,6)+qq3(3);
    xyzpairs(7,2)+qq4(1)+0.03 , xyzpairs(7,4)+qq4(2) , xyzpairs(7,6)+qq4(3)];

temp = xyzpatch.vertices(:,1);        
    xyzpatch.vertices(:,1) = xyzpatch.vertices(:,3);
    xyzpatch.vertices(:,3) = temp;

set(lnkpatch(jj),'vertices',xyzpatch.vertices);

% end feet patches    
    
    % copy objects to other views
% delete(plot2objs);    
% plot2objs = copyobj([lnkpatch';lin'],params.plot_main(2));
% view(3);
% delete(plot3objs);
% plot3objs = copyobj([lnkpatch';lin'],params.plot_main(3));
% view(3);
% delete(plot4objs);
% plot4objs = copyobj([lnkpatch';lin'],params.plot_main(4));
% view(3);

% params.plot_objs{1}=[lnkpatch';lin'];
params.plot_objs{1}=[lnkpatch';lin';pos';lin_mirror';plot_traj];


delete(params.plot_objs{2});    
params.plot_objs{2} = copyobj(params.plot_objs{1},params.plot_main(2));

delete(params.plot_objs{3});
params.plot_objs{3} = copyobj(params.plot_objs{1},params.plot_main(3));

delete(params.plot_objs{4});    
params.plot_objs{4} = copyobj(params.plot_objs{1},params.plot_main(4));


    
    for jj=1:n_lin
        set(lin_mirror(jj),xaxis,-0.5*xyzpairs(jj,1:2),yaxis,xyzpairs(jj,3:4),zaxis,xyzpairs(jj,5:6));
    end
    
    
    
    % %     % update meshes
    % %     if draw_meshes == 1
    % %         set(h_patch(1),xaxis,x1i+x(8));
    % %         set(h_patch(1),yaxis,y1i+y(8));
    % %         set(h_patch(1),zaxis,z1i+z(8));
    % %
    % %         set(h_patch(2),xaxis,x2i+(x(1)+x(8))*0.5);
    % %         set(h_patch(2),yaxis,y2i+(y(1)+y(8))*0.5);
    % %         set(h_patch(2),zaxis,z2i+(z(1)+z(8))*0.5);
    % %
    % %         %       set(h_patch(2),'vertices',[vertic1(:,1)+(z(1)+z(8))*0.5,vertic1(:,2)+(y(1)+y(8))*0.5,vertic1(:,3)+(x(1)+x(8))*0.5]);
    % %
    % %         set(h_patch(3),xaxis,x3i+(x(2)+x(5))*0.5);
    % %         set(h_patch(3),yaxis,y3i+(y(2)+y(5))*0.5);
    % %         set(h_patch(3),zaxis,z3i+(z(2)+z(5))*0.5);
    % %
    % %         set(h_patch(4),xaxis,x4i+(x(4)));
    % %         set(h_patch(4),yaxis,y4i+(y(4)));
    % %         set(h_patch(4),zaxis,z4i+(z(4)));
    % %
    % %         set(h_patch(5),xaxis,x5i+(x(7)));
    % %         set(h_patch(5),yaxis,y5i+(y(7)));
    % %         set(h_patch(5),zaxis,z5i+(z(7)));
    % %     end
    
    %     title(num2str(ii*params.sim_step));
    
    
    if ishghandle(params.gui.gui_edit_currentTime)
        set(params.gui.gui_edit_currentTime,'String',num2str(ii*params.sim_step));
    end
    %     pauseus(simstep-0.005);
    drawnow;
    time_dif = vis_speed*params.sim_step-toc();
    if time_dif>0
        %         1
        pause(time_dif);
    else
        %         2
        vis_speed=vis_speed+1;
    end
    
%     if params.visualize_stop == 1;
%         params.visualize_stop=0;
%         break;
%     end
    
%     if ii+vis_speed > n  && params.visualize_loop==1;
%         ii=1;
%     end
    
    ii=ii+vis_speed;
    
end

% set(params.gui.gui_edit_currentTime,'String','-');

end

