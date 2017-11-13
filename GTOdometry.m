% Ground Truth Only
% Generates a visual odometry using every vicon ground truth file available

ViconMatlabImport;

%% Create a View Set Containing the First View of the Sequence
% Use a |viewSet| object to store and manage the image points and the
% camera pose associated with each view, as well as point matches between
% pairs of views. Once you populate a |viewSet| object, you can use it to
% find point tracks across multiple views and retrieve the camera poses to
% be used by |triangulateMultiview| and |bundleAdjustment| functions.

% Create an empty viewSet object to manage the data associated with each view.
vSet = viewSet;

% Read and display the first image.

%% Plot Initial Camera Pose
% Create two graphical camera objects representing the estimated and the
% actual camera poses based on ground truth data from the New Tsukuba
% dataset.

% Setup axes.
% figure
% axis([-5, 5, -5, 5, -5, 5]);
% 
% grid on
% xlabel('X (cm)');
% ylabel('Y (cm)');
% zlabel('Z (cm)');
% hold on
% 
% % Plot estimated camera pose. 
% cameraSize = 0.2;
% 
% % Plot actual camera pose.
% camActual = plotCamera('Size', cameraSize, 'Location', ...
%     groundTruthPoses.Location{1}, 'Orientation', ...
%     groundTruthPoses.Orientation{1}, 'Color', 'b', 'Opacity', 0);
% 
% % Initialize camera trajectories.
% trajectoryActual    = plot3(0, 0, 0, 'b-');
% 
% legend('Actual Trajectory');
% title('Camera Trajectory');

%%
% Update camera trajectory plots using 
% <matlab:edit('helperUpdateCameraPlots.m') helperUpdateCameraPlots> and
% <matlab:edit('helperUpdateCameraTrajectories.m') helperUpdateCameraTrajectories>.

% Move the actual camera in the plot.
% camActual.Location = groundTruthPoses.Location{2};
% camActual.Orientation = groundTruthPoses.Orientation{2};
% 
% % Plot the ground truth trajectory
% locationsActual = cat(1, groundTruthPoses.Location{1:2});
% set(trajectoryActual, 'XData', locationsActual(:,1), 'YData', ...
%     locationsActual(:,2), 'ZData', locationsActual(:,3));

%% Remaining Views
% for viewId = height(groundTruthPoses)
%     
%     % Move the actual camera in the plot.
%     camActual.Location = groundTruthPoses.Location{viewId};
%     camActual.Orientation = groundTruthPoses.Orientation{viewId};
% 
%     
%     % Plot the ground truth trajectory
%     locationsActual = cat(1, groundTruthPoses.Location{1:viewId});
%     set(trajectoryActual, 'XData', locationsActual(:,1), 'YData', ...
%         locationsActual(:,2), 'ZData', locationsActual(:,3));
% end
% 
% 
% hold off

%% Display Camera Poses
% Display the refined camera poses and 3-D world points.
set(0, 'DefaultTextInterpreter', 'none');
% Display camera poses.
figure;
%axis([-0.5, 0.5, -0.5, 0.5, -0.5, 0.5]);
plotCamera(groundTruthPoses, 'Size', 0.1);
hold on

hold off

% Specify the viewing volume.
loc1 = groundTruthPoses.Location{1};
xlim([loc1(1)-5, loc1(1)+10]);
ylim([loc1(2)-10, loc1(2)+10]);
zlim([loc1(3)-1, loc1(3)+10]);
%camorbit(0, -120, -90);
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
hold on

title('Refined Camera Poses');

displayEndOfDemoMessage(mfilename)
