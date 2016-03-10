function plotQuat(q)
% q is nx7

n = length(q);

figure;

q_ang = acos(q(:,4))*2;
rx = q(:,5)./sqrt(1-q(:,4).*q(:,4));
ry = q(:,6)./sqrt(1-q(:,4).*q(:,4));
rz = q(:,7)./sqrt(1-q(:,4).*q(:,4));

x = q(:,1);
y = q(:,2);
z = q(:,3);

R = vrrotvec2mat([rx(1) ry(1) rz(1) q_ang(1)]);

pos3 = plot3(x(1),y(1),z(1),'erasemode','normal');
axis([-2 2 -2 2 -2 2]);
hold on;
xlabel('x');
ylabel('y');
zlabel('z');
legend;

grid on;
orix = line('xdata',[x(1) x(1)+R(1,1)],'ydata',[y(1) y(1)+R(2,1)],'zdata',[z(1) z(1)+R(3,1)],'erasemode','normal','LineWidth', 2,'color', 'red');
oriy = line('xdata',[x(1) x(1)+R(1,2)],'ydata',[y(1) y(1)+R(2,2)],'zdata',[z(1) z(1)+R(3,2)],'erasemode','normal','LineWidth', 2,'color', 'green');
oriz = line('xdata',[x(1) x(1)+R(1,3)],'ydata',[y(1) y(1)+R(2,3)],'zdata',[z(1) z(1)+R(3,3)],'erasemode','normal','LineWidth', 2,'color', 'blue');


set(gca,'drawmode','fast');

pause;
for ii=1:n
    
    R = vrrotvec2mat([rx(ii) ry(ii) rz(ii) q_ang(ii)]);
    
    
    
    plot3(x(ii),y(ii),z(ii),'.');
    set(pos3,'xdata',[x(ii)],'ydata',[y(ii)],'zdata',[z(ii)]);
    set(orix,'xdata',[x(ii) x(ii)+R(1,1)],'ydata',[y(ii) y(ii)+R(2,1)],'zdata',[z(ii) z(ii)+R(3,1)],'erasemode','normal','LineWidth', 2,'color', 'blue');
    set(oriy,'xdata',[x(ii) x(ii)+R(1,2)],'ydata',[y(ii) y(ii)+R(2,2)],'zdata',[z(ii) z(ii)+R(3,2)],'erasemode','normal','LineWidth', 2,'color', 'green');
    set(oriz,'xdata',[x(ii) x(ii)+R(1,3)],'ydata',[y(ii) y(ii)+R(2,3)],'zdata',[z(ii) z(ii)+R(3,3)],'erasemode','normal','LineWidth', 2,'color', 'red');
    
    
    
%     set(ori3,'xdata',[x(ii) x(ii)+rx(ii)],'ydata',[y(ii) y(ii)+ry(ii)],'zdata',[z(ii) z(ii)+rz(ii)]);
    title(num2str(ii));
    
    pause(0.0005);
    
%     pause;
%     drawnow;
end
