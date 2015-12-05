function plotQuat(vqT)
    % the matrix-dimension must be of m-by-7:
    [m,n] = size(vqT);
    if (n ~= 7)
        error('plotQuat: %s', WBM.wbmErrMsg.WRONG_MAT_DIM);
    end
    len = m;

    figure;

    x = vqT(:,1);
    y = vqT(:,2);
    z = vqT(:,3);

    angle = acos(vqT(:, 4))*2;
    rx = vqT(:,5)./sqrt(1 - vqT(:,4).*vqT(:,4));
    ry = vqT(:,6)./sqrt(1 - vqT(:,4).*vqT(:,4));
    rz = vqT(:,7)./sqrt(1 - vqT(:,4).*vqT(:,4));

    R = vrrotvec2mat([rx(1) ry(1) rz(1) angle(1)]);

    pos3 = plot3(x(1), y(1), z(1), 'erasemode', 'normal');
    axis([-2 2 -2 2 -2 2]);
    hold on;
    xlabel('x');
    ylabel('y');
    zlabel('z');
    legend;

    grid on;
    orix = line('xdata', [x(1) (x(1)+R(1,1))], 'ydata', [y(1) (y(1)+R(2,1))], 'zdata', [z(1) (z(1)+R(3,1))], ...
                'erasemode', 'normal', 'LineWidth', 2, 'color', 'red');
    oriy = line('xdata', [x(1) (x(1)+R(1,2))], 'ydata', [y(1) (y(1)+R(2,2))], 'zdata', [z(1) (z(1)+R(3,2))], ...
                'erasemode', 'normal', 'LineWidth', 2, 'color', 'green');
    oriz = line('xdata', [x(1) (x(1)+R(1,3))], 'ydata', [y(1) (y(1)+R(2,3))], 'zdata', [z(1) (z(1)+R(3,3))], ...
                'erasemode', 'normal', 'LineWidth', 2, 'color', 'blue');

    if verLessThan('matlab', '8.4.0')
        % for Matlab R2014b and later ...
        set(gca, 'SortMethod', 'childorder');
    else
        % for older Matlab versions (<= R2014a) ...
        set(gca, 'DrawMode', 'fast');
    end

    pause;
    for i = 1:len
        R = vrrotvec2mat([rx(i) ry(i) rz(i) angle(i)]);

        plot3(x(i), y(i), z(i), '.');
        set(pos3, 'xdata', x(i), 'ydata', y(i), 'zdata', z(i));
        
        set(orix, 'xdata', [x(i) (x(i)+R(1,1))], 'ydata', [y(i) (y(i)+R(2,1))], 'zdata', [z(i) (z(i)+R(3,1))], ...
                  'erasemode', 'normal', 'LineWidth', 2, 'color', 'blue');
        set(oriy, 'xdata', [x(i) (x(i)+R(1,2))], 'ydata', [y(i) (y(i)+R(2,2))], 'zdata', [z(i) (z(i)+R(3,2))], ...
                  'erasemode', 'normal', 'LineWidth', 2, 'color', 'green');
        set(oriz, 'xdata', [x(i) (x(i)+R(1,3))], 'ydata', [y(i) (y(i)+R(2,3))], 'zdata', [z(i) (z(i)+R(3,3))], ...
                  'erasemode', 'normal', 'LineWidth', 2, 'color', 'red');

        title(num2str(i));
        pause(0.0005);
    end
end
