% Michael Sanne 13/11/2017
% UGrad_Thesis 2017
% Plot3D
% Plotting function for plotting 4 cameras with 2 sets of point matches
% onto a common figure
% INPUTS
%   v = viewSet containing all of the camera views
%   path = location to where the figure should be printed

% Plot Views
function Plot(v, path)
% Plot all camera poses from the viewset in one image
figure;
set(0,'DefaultTextInterpreter', 'latex');
grid on;  %xlabel('X');  ylabel('Y');
set(gca,'XTickLabel',[]);
set(gca,'YTickLabel',[]);
title('Estimated Odometry'); hold on;
pcsize = 0.5;
camPoses = poses(v);
set(0, 'DefaultTextInterpreter', 'none');
plotCamera(camPoses, 'Size', 0.2);
hold off;
print(gcf, '-depsc', path);