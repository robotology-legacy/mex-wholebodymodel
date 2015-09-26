function plotQuat(q)
    % check if the matrix-dimension is m-by-7:
    [~,n] = size(q);
    if (n ~= 7)
        error('iCubWBM::plotQuat: %s', obj.wb_strWrongDimErr);
    end
    len = length(q);

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
    for i = 1:len
        R = vrrotvec2mat([rx(i) ry(i) rz(i) q_ang(i)]);

        plot3(x(i),y(i),z(i),'.');
        set(pos3,'xdata',[x(i)],'ydata',[y(i)],'zdata',[z(i)]);
        set(orix,'xdata',[x(i) x(i)+R(1,1)],'ydata',[y(i) y(i)+R(2,1)],'zdata',[z(i) z(i)+R(3,1)],'erasemode','normal','LineWidth', 2,'color', 'blue');
        set(oriy,'xdata',[x(i) x(i)+R(1,2)],'ydata',[y(i) y(i)+R(2,2)],'zdata',[z(i) z(i)+R(3,2)],'erasemode','normal','LineWidth', 2,'color', 'green');
        set(oriz,'xdata',[x(i) x(i)+R(1,3)],'ydata',[y(i) y(i)+R(2,3)],'zdata',[z(i) z(i)+R(3,3)],'erasemode','normal','LineWidth', 2,'color', 'red');

        title(num2str(i));
        pause(0.0005);
    end
end
