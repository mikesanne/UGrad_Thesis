function Plot3D (R1, t1, R2, t2, R3, t3, R4, t4, cameraParams, points1_12, points2_12, points1_34, points2_34, path)    
%% Plot
    
    % Plot the most recent camera poses (3) with their respective world
    % points
    P1 = cameraMatrix(cameraParams, R1, t1);
    P2 = cameraMatrix(cameraParams, R2, t2);
    P3 = cameraMatrix(cameraParams, R3, t3);
    P4 = cameraMatrix(cameraParams, R4, t4);
    wpi_12 = triangulate(points1_12, points2_12, P1, P2);
    wpi_34 = triangulate(points1_34, points2_34, P3, P4);
    figure;  %axis([-10, 10, -10, 10, 0, 20]);
    set(0,'DefaultTextInterpreter', 'latex');
    view(gca, 3);
    set(gca, 'CameraUpVector', [0,-1, 0]); camorbit(gca, -120, 0, 'data', [0, 1, 0]);
    grid on;  xlabel('X');  ylabel('Y');  zlabel('Z');  hold on;
    pcsize = 0.5;
    set(0,'DefaultTextInterpreter', 'none');
    a = plotCamera('Size', pcsize, 'Location', t1, 'Orientation', R1, 'Color', 'g', 'Opacity', 0);
    b = plotCamera('Size', pcsize, 'Location', t2, 'Orientation', R2, 'Color', 'k', 'Opacity', 0);
    c = plotCamera('Size', pcsize, 'Location', t3, 'Orientation', R3, 'Color', 'k', 'Opacity', 0);
    d = plotCamera('Size', pcsize, 'Location', t4, 'Orientation', R4, 'Color', 'r', 'Opacity', 0);
    scatter3(wpi_12(:,1)',wpi_12(:,2)',wpi_12(:,3)',100,'g.');
    scatter3(wpi_34(:,1)',wpi_34(:,2)',wpi_34(:,3)',100,'r.');
    set(0,'DefaultTextInterpreter', 'latex');
    title('3 Cameras and Matched Points With Scale Adjustment');
    print(gcf, '-depsc', path);