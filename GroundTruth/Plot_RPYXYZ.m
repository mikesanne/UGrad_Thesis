% Plot Graphs
close all;
frame_step = 5;
[tCam, xCam, yCam, zCam, RCam, rollCam, pitchCam, yawCam] = RPYXYZ_Cam(frame_step);
[rollVicon, pitchVicon, yawVicon, xVicon, yVicon, zVicon] = RPYXYZ_Vicon(frame_step);
start = 1;
finish = size(xCam,1);
step = 1;



set(0,'DefaultTextInterpreter', 'latex');
figure;
    hold on;    
    a = gca;
    a.TickLabelInterpreter = 'latex';
    plot((start:step:finish), -rollCam(start:step:finish), 'b', 'LineWidth', 0.3);
    plot((start:step:finish),rollVicon(start:step:finish), 'k', 'LineWidth', 0.1);
    title('Roll');
    xlabel('Frame Number');
    ylabel('Change in Roll');
    axis([1, finish, -0.1, 0.15]);
    l = legend('Estimated Roll', 'Ground Truth Roll');
    set(l, 'Interpreter', 'latex');
    hold off;
    print(gcf, '-depsc', '/Users/michaelsanne/Documents/UCT/4th Year/Theses/Thesis Write-up/figures/roll_fivepoint');

figure;
    hold on;
    a = gca;
    a.TickLabelInterpreter = 'latex';
    plot((start:step:finish), -pitchCam(start:step:finish), 'b', 'LineWidth', 0.3);
    plot((start:step:finish),pitchVicon(start:step:finish), 'k', 'LineWidth', 0.1);
    title('Pitch', 'Interpreter', 'latex');
    xlabel('Frame Number', 'Interpreter', 'latex');
    ylabel('Change in Pitch', 'Interpreter', 'latex');
    axis([1, finish, -0.15, 0.1]);
    l = legend('Estimated Pitch', 'Ground Truth Pitch');
    set(l, 'Interpreter', 'latex');
    hold off;
    print(gcf, '-depsc', '/Users/michaelsanne/Documents/UCT/4th Year/Theses/Thesis Write-up/figures/pitch_fivepoint');

figure;
    hold on;
    a = gca;
    a.TickLabelInterpreter = 'latex';
    plot((start:step:finish), yawCam(start:step:finish), 'b', 'LineWidth', 0.3);
    plot((start:step:finish),yawVicon(start:step:finish), 'k', 'LineWidth', 0.1);
    title('Yaw', 'Interpreter', 'latex');
    xlabel('Frame Number', 'Interpreter', 'latex');
    ylabel('Change in Yaw', 'Interpreter', 'latex');
    axis([1, finish, -0.2, 0.15]);
    l = legend('Estimated Yaw', 'Ground Truth Yaw');
    set(l, 'Interpreter', 'latex');
    hold off;
    print(gcf, '-depsc', '/Users/michaelsanne/Documents/UCT/4th Year/Theses/Thesis Write-up/figures/yaw_fivepoint');
    
pitch = figure;
    hold on;    
    a = gca;
    a.TickLabelInterpreter = 'latex';
    plot((start:step:finish), -xCam(start:step:finish), 'b', 'LineWidth', 0.3);
    plot((start:step:finish),xVicon(start:step:finish), 'k', 'LineWidth', 0.1);
    title('X');
    xlabel('Frame Number');
    ylabel('Change in X');
    axis([1, finish, -1, 1]);
    l = legend('Estimated X', 'Ground Truth X');
    set(l, 'Interpreter', 'latex');
    hold off;
    print(gcf, '-depsc', '/Users/michaelsanne/Documents/UCT/4th Year/Theses/Thesis Write-up/figures/x_fivepoint');

figure;
    hold on;
    a = gca;
    a.TickLabelInterpreter = 'latex';
    plot((start:step:finish), -yCam(start:step:finish), 'b', 'LineWidth', 0.3);
    plot((start:step:finish),yVicon(start:step:finish), 'k', 'LineWidth', 0.1);
    title('Y', 'Interpreter', 'latex');
    xlabel('Frame Number', 'Interpreter', 'latex');
    ylabel('Change in Y', 'Interpreter', 'latex');
    axis([1, finish, -1, 1]);
    l = legend('Estimated Y', 'Ground Truth Y');
    set(l, 'Interpreter', 'latex');
    hold off;
    print(gcf, '-depsc', '/Users/michaelsanne/Documents/UCT/4th Year/Theses/Thesis Write-up/figures/y_fivepoint');

figure;
    hold on;
    a = gca;
    a.TickLabelInterpreter = 'latex';
    plot((start:step:finish), -zCam(start:step:finish), 'b', 'LineWidth', 0.3);
    plot((start:step:finish),zVicon(start:step:finish), 'k', 'LineWidth', 0.1);
    title('Z', 'Interpreter', 'latex');
    xlabel('Frame Number', 'Interpreter', 'latex');
    ylabel('Change in Z', 'Interpreter', 'latex');
    axis([1, finish, -1, 1]);
    l = legend('Estimated Z', 'Ground Truth Z');
    set(l, 'Interpreter', 'latex');
    hold off;
    print(gcf, '-depsc', '/Users/michaelsanne/Documents/UCT/4th Year/Theses/Thesis Write-up/figures/z_fivepoint');    
    
alphaYaw = cosineSimilarity(yawVicon(1:step:finish),-yawCam(1:step:finish))
alphaRoll = cosineSimilarity(rollVicon(1:step:finish),-rollCam(1:step:finish))
alphaPitch = cosineSimilarity(pitchVicon(1:step:finish),-pitchCam(1:step:finish))
    