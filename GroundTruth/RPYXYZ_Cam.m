% Michael Sanne 13/12/2017 
% RPYXYZ_Cam
% This function aims to determine the changes in roll pitch and yaw  as
% well as changes in x y and z between corresponding views of a scene
% INPUTS
%   step = the number of frames to skip between corresponding views
% OUTPUTS
%   tCam = translation vector containing all of the translation changes of
%   the camera between frames
%   xCam = vector containing all x locations of the poses
%   yCam = vector containing all y locations of the poses
%   zCam = vector containing all z locations of the poses
%   RCAm = N 3x3 rotation matrices containing every rotational change
%   possible
%   rollCam = vector containing all roll positions of the poses
%   pitchCam = vector containing all pitch positions of the poses
%   yawCam = vector containing all yaw positions of the poses
%   matches = vector returning the number of matches per correspondence set
%   v = viewSet containing all relative R and t values
%

%%
function [tCam, xCam, yCam, zCam, RCam, rollCam, pitchCam, yawCam, matches, v] = RPYXYZ_Cam(step)
matches = 0;
%RPY relative camera between poses
images = imageDatastore('../Data/LoopDown/imagesUndistort_1loopDown');
K = load('../Data/1LoopDown/camIntrinsicDown.txt')';
cameraParams = cameraParameters('IntrinsicMatrix', K);
tCam = [0 0 0];
xCam = 0;
yCam = 0;
zCam = 0;
RCam = eye(3);
rollCam = 0;
pitchCam = 0;
yawCam = 0;
invalid = 0;

R = eye(3);
t = [0 0 0];
v = viewSet;
vId = 1;
[orientation,location] = extrinsicsToCameraPose(R,t);
    v = addView(v, vId, 'Orientation', ...
        orientation, 'Location', location);
    vId = vId + 1;
for i = 1+step:step:numel(images.Files)
    rng('default');
    i
% Load
I1 = rgb2gray(readimage(images,i-step));
I2 = rgb2gray(readimage(images,i));

% Features
mtsparm = 1000;
points1 = detectSURFFeatures(I1, 'MetricThreshold', mtsparm);
feat1 = extractFeatures(I1, points1);%, 'Upright', true);
points2 = detectSURFFeatures(I2, 'MetricThreshold', mtsparm);
feat2 = extractFeatures(I2, points2);%, 'Upright', true);

% Putative matches
mpairs = matchFeatures(feat1, feat2, 'Unique', true);
mpoints1 = points1(mpairs(:,1));  mpoints2 = points2(mpairs(:,2));  
matches = [matches ; size(mpairs,1)];
%fh = figure; showMatchedFeatures(I1, I2, mpoints1, mpoints2, 'falsecolor'); 
%close all;
try
% Pose estimation
for j=1:1000
  % Estimate the essential matrix
  [E, minf] = estimateEssentialMatrix(mpoints1, mpoints2, cameraParams);
  if sum(minf)/numel(minf)<0.3, continue; end
  
  % Camera pose
  [orient,loc,vpf] = relativeCameraPose(E, cameraParams, mpoints1(minf), mpoints2(minf));
  if vpf>0.8, break; end
end
if vpf<=0.8, warning('Problem with camera pose estimate?'); 
    invalid = [invalid ; i]; end
catch
    disp('Image fail');
    i
   continue;
end

% Camera matrices
R1 = eye(3);  t1 = zeros(1,3);
P1 = cameraMatrix(cameraParams, R1, t1);
[R2,t2] = cameraPoseToExtrinsics(orient, loc);
P2 = cameraMatrix(cameraParams, R2, t2);
[orientation,location] = extrinsicsToCameraPose(R2,t2);
R = R*R2;
t = ((R2*t')')+t2;
v = addView(v, vId, 'Points', points1, 'Orientation', ...
     orientation, 'Location', location);
vId = vId + 1;

tCam = [tCam; t2(1:3)];
xCam = [xCam; t2(1)];
yCam = [yCam; t2(2)];
zCam = [zCam; t2(3)];

[yaw, pitch, roll] = dcm2angle(R2);
RCam = [RCam ; R2];
rollCam = [rollCam; roll];
pitchCam = [pitchCam; pitch];
yawCam = [yawCam; -yaw];

if vpf<= 0.8
    subplot(1,2,1);  ih = imagesc(I1);  colormap(gray);  hold on;  ph = plot(points1);  hold off
    subplot(1,2,2);  ih = imagesc(I2);  colormap(gray);  hold on;  ph = plot(points2);  hold off;
    fh = figure; showMatchedFeatures(I1, I2, mpoints1, mpoints2, 'falsecolor');
    ph1 = plotCamera('Size', 0.2, 'Location', t1, 'Orientation', R1, 'Color', 'g', 'Opacity', 0);
    ph2 = plotCamera('Size', 0.2, 'Location', t2, 'Orientation', R2, 'Color', 'r', 'Opacity', 0);
end


end

% Plot Camera Estimates
% figure;
% plot(1:step:numel(images.Files), rollCam, 'r');
% figure;
% plot(1:step:numel(images.Files), pitchCam, 'g');
% figure;
% plot(1:step:numel(images.Files), yawCam, 'b');
% figure;
% plot(1:step:numel(images.Files), xCam, 'b');
% figure;
% plot(1:step:numel(images.Files), yCam, 'g');
% figure;
% plot(1:step:numel(images.Files), zCam, 'r');