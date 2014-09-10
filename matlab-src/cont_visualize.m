function cont_visualize(xout,simtime,simstep)

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

n_plot = 15;
n_lin = 13;

n = size(xout,1);

q=xout(:,1:7);
qj=xout(:,8:32);


for jj=2:n_plot
    kin(:,:,jj)=zeros(n,7);
    for ii=1:n
        kin(ii,:,jj) = (wholeBodyModel('forward-kinematics',qj(ii,:)',L{jj}))';
    end
end

kin(:,:,1)= q; %use base data instead of fwdkin rootlink

figure;

plot3(0,0,0,'.');
axis([0 1 -0.6 0.6 -0.6 0.6]);
hold on;
grid on;
xlabel('x');
ylabel('y');
zlabel('z');
set(gca,'color',[0.6 0.7 0.6]);
set(gca,'drawmode','fast');

%%
x = zeros(1,n_plot);
y = zeros(1,n_plot);
z = zeros(1,n_plot);
R = zeros(3,3,n_plot);
pos = zeros(1,n_plot);

%initial plot
x(1)=kin(1,1,1);y(1)=kin(1,2,1);z(1)=kin(1,3,1);
pos(1)=plot3([x(jj)],[y(jj)],[z(jj)],'r*');

for jj=2:n_plot

[x(jj),y(jj),z(jj),R(:,:,jj)] = quat2rot(kin(1,:,jj));
pos(jj)=plot3([x(jj)],[y(jj)],[z(jj)],'*');

end
% 
% xp = zeros(n_lin,2);
% yp = zeros(n_lin,2);
% zp = zeros(n_lin,2);

xyzpairs = zeros(n_lin,6);

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
for jj=1:n_lin
    lin(jj) = line('xdata',xyzpairs(jj,1:2),'ydata',xyzpairs(jj,3:4),'zdata',xyzpairs(jj,5:6),'erasemode','normal','linewidth',2);
end    


pause;
tic;
%draw 

for ii=1:n
%     tic;
    for jj=1:n_plot
    
        [x(jj),y(jj),z(jj),R(:,:,jj)] = quat2rot(kin(ii,:,jj));
        set(pos(jj),'xdata',[x(jj)],'ydata',[y(jj)],'zdata',[z(jj)]);
        
        
    end
    
    if x(8)<=0
        break;
    end
    
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
     
    for jj=1:n_lin
        set(lin(jj),'xdata',xyzpairs(jj,1:2),'ydata',xyzpairs(jj,3:4),'zdata',xyzpairs(jj,5:6));
    end
    
    title(num2str(ii));
    
%     tic;
    pause(0.001);
%     toc;
    
%     pauseus(simstep-0.005);
%     toc
%     pause;
%     drawnow;
end
