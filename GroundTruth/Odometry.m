%
% This function generates an odometry of a set of poses represented by
% euler angles as well as x, y, z translation vectors
% INPUTS
%   rollVicon = a set of roll values corresponding to a set of poses
%   pitchVicon = a set of pitch values corresponding to a set of poses
%   yawVicon = a set of yaw values corresponding to a set of poses
%   xVicon = a set of x values corresponding to a set of poses
%   yVicon = a set of y values corresponding to a set of poses
%   zVicon = a set of z values corresponding to a set of poses
% RESULT
%   3D Scatter Plot - each dot represents a camera position
%% %% Plot Odometry GT
function Odometry(rollVicon, pitchVicon, yawVicon, xVicon, yVicon, zVicon)
%   
% Start at origin with identity 
T_Pre = eye(4);
t = [0 0 0];
% Loop through each onr
for i=1:size(rollVicon,1)
    T_PreCur = [euler_to_R([rollVicon(i), pitchVicon(i), yawVicon(i)]) [xVicon(i) yVicon(i) zVicon(i)]'; 0 0 0 1];
    T_Cur = T_Pre*T_PreCur;
    t_cur = T_Cur(1:3,4)';
    t = [t; t_cur];
end
scatter3(t(:,1), t(:,2), t(:,3), 'b.');
end

  