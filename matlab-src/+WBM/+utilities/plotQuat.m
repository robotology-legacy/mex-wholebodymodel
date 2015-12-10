function plotQuat(vqT)
    % the matrix-dimension must be of m-by-7:
    [m,n] = size(vqT);
    if (n ~= 7)
        error('plotQuat: %s', WBM.wbmErrMsg.WRONG_MAT_DIM);
    end
    len = m;
    ln_width = 1;

    figure;

    x = vqT(1:len,1);
    y = vqT(1:len,2);
    z = vqT(1:len,3);

    angle = acos(vqT(:,4))*2;
    rx = vqT(1:len,5)./sqrt(1 - vqT(1:len,4).*vqT(1:len,4));
    ry = vqT(1:len,6)./sqrt(1 - vqT(1:len,4).*vqT(1:len,4));
    rz = vqT(1:len,7)./sqrt(1 - vqT(1:len,4).*vqT(1:len,4));

    R = vrrotvec2mat( horzcat(rx(1), ry(1), rz(1), angle(1)) );

    hPos3D = plot3(x(1), y(1), z(1), 'LineStyle', '-', 'Marker', 'o', 'MarkerEdgeColor', 'k');
    if verLessThan('matlab', '8.4.0')
        % Matlab <= R2014a:
        set(hPos3D, 'EraseMode', 'normal');
    end

    axis( repmat([-2 2], 1, 3) );

    hold on;
    
    xlabel('x');
    ylabel('y');
    zlabel('z');
    legend;

    grid on;

    title(sprintf('Quaternion (%d):', 1));

    if ~verLessThan('matlab', '8.4.0')
        % for Matlab R2014b and later:
        set(gca, 'SortMethod', 'childorder');

        hLn_orX = animatedline(horzcat(x(1), x(1)+R(1,1)), horzcat(y(1), y(1)+R(2,1)), ...
                               horzcat(z(1), z(1)+R(3,1)), 'LineWidth', ln_width, 'Color', 'red');
        hLn_orY = animatedline(horzcat(x(1), x(1)+R(1,2)), horzcat(y(1), y(1)+R(2,2)), ...
                               horzcat(z(1), z(1)+R(3,2)), 'LineWidth', ln_width, 'Color', 'green');
        hLn_orZ = animatedline(horzcat(x(1), x(1)+R(1,3)), horzcat(y(1), y(1)+R(2,3)), ...
                               horzcat(z(1), z(1)+R(3,3)), 'LineWidth', ln_width, 'Color', 'blue');
        pause;
    
        for i = 2:len
            title(sprintf('Quaternion (%d):', i));

            set(hPos3D, 'XData', x(i), 'YData', y(i), 'ZData', z(i));

            R = vrrotvec2mat( horzcat(rx(i), ry(i), rz(i), angle(i)) );

            addpoints(hLn_orX, horzcat(x(i), x(i)+R(1,1)), horzcat(y(i), y(i)+R(2,1)), horzcat(z(i), z(i)+R(3,1)));
            addpoints(hLn_orY, horzcat(x(i), x(i)+R(1,2)), horzcat(y(i), y(i)+R(2,2)), horzcat(z(i), z(i)+R(3,2)));
            addpoints(hLn_orZ, horzcat(x(i), x(i)+R(1,3)), horzcat(y(i), y(i)+R(2,3)), horzcat(z(i), z(i)+R(3,3)));

            pause(0.0005);
        end
    else
        % for older Matlab versions (<= R2014a):
        set(gca, 'DrawMode', 'fast');

        hLn_orX = line('XData', horzcat(x(1), x(1)+R(1,1)), 'YData', horzcat(y(1), y(1)+R(2,1)), ...
                       'ZData', horzcat(z(1), z(1)+R(3,1)), 'EraseMode', 'normal', ...
                       'LineWidth', ln_width, 'Color', 'red');
        hLn_orY = line('XData', horzcat(x(1), x(1)+R(1,2)), 'YData', horzcat(y(1), y(1)+R(2,2)), ...
                       'ZData', horzcat(z(1), z(1)+R(3,2)), 'EraseMode', 'normal', ...
                       'LineWidth', ln_width, 'Color', 'green');
        hLn_orZ = line('XData', horzcat(x(1), x(1)+R(1,3)), 'YData', horzcat(y(1), y(1)+R(2,3)), ...
                       'ZData', horzcat(z(1), z(1)+R(3,3)), 'EraseMode', 'normal', ...
                       'LineWidth', ln_width, 'Color', 'blue');
        pause;

        for i = 2:len
            title(sprintf('Quaternion (%d):', i));

            set(hPos3D, 'XData', x(i), 'YData', y(i), 'ZData', z(i));
            
            R = vrrotvec2mat( horzcat(rx(i), ry(i), rz(i), angle(i)) );

            set(hLn_orX, 'XData', horzcat(x(i), x(i)+R(1,1)), 'YData', horzcat(y(i), y(i)+R(2,1)), ...
                         'ZData', horzcat(z(i), z(i)+R(3,1)), 'EraseMode', 'normal', ...
                         'LineWidth', ln_width, 'Color', 'blue');
            set(hLn_orY, 'XData', horzcat(x(i), x(i)+R(1,2)), 'YData', horzcat(y(i), y(i)+R(2,2)), ...
                         'ZData', horzcat(z(i), z(i)+R(3,2)), 'EraseMode', 'normal', ...
                         'LineWidth', ln_width, 'Color', 'green');
            set(hLn_orZ, 'XData', horzcat(x(i), x(i)+R(1,3)), 'YData', horzcat(y(i), y(i)+R(2,3)), ...
                         'ZData', horzcat(z(i), z(i)+R(3,3)), 'EraseMode', 'normal', ...
                         'LineWidth', ln_width, 'Color', 'red');
            pause(0.0005);
        end
    end
end
