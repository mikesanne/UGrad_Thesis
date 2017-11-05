% Plot Views
function Plot(v)
% Plot all camera poses from the viewset in one image
figure;
set(0,'DefaultTextInterpreter', 'latex');
grid on;  xlabel('X');  ylabel('Y');  title('Camera Poses with Bundle Adjustment'); hold on;
pcsize = 0.5;
camPoses = poses(v);
set(0, 'DefaultTextInterpreter', 'none');
plotCamera(camPoses, 'Size', 0.2);
hold off;