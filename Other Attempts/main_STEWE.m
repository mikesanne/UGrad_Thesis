close all;
clear;
rng('default');

    rollDiff = 1;
    pitchDiff = 1;
    yawDiff = 1;
    step = 5;
for i = 1:step:600
    i
% Load
images = imageDatastore('Data/1LoopDown/imagesUndistort_1loopDown');
I1 = rgb2gray(readimage(images,i));
I2 = rgb2gray(readimage(images,i+step));
K = load('Data/1LoopDown/camIntrinsicDown.txt')';
cameraParams = cameraParameters('IntrinsicMatrix', K);

% Features
mtsparm = 100;
%points1 = detectSURFFeatures(I1, 'MetricThreshold', mtsparm);
points1 = detectSURFFeatures(I1);
feat1 = extractFeatures(I1, points1);%, 'Upright', true);
%points2 = detectSURFFeatures(I2, 'MetricThreshold', mtsparm);
points2 = detectSURFFeatures(I2);
feat2 = extractFeatures(I2, points2);%, 'Upright', true);
%fh = figure;  
%subplot(1,2,1);  ih = imagesc(I1);  colormap(gray);  hold on;  ph = plot(points1);  hold off
% subplot(1,2,2);  ih = imagesc(I2);  colormap(gray);  hold on;  ph = plot(points2);  hold off;

% Putative matches
mpairs = matchFeatures(feat1, feat2, 'Unique', true);
mpoints1 = points1(mpairs(:,1));  mpoints2 = points2(mpairs(:,2));  
% fh = figure; showMatchedFeatures(I1, I2, mpoints1, mpoints2, 'falsecolor'); 


%% Nister's 5-point implementation
homogeneous = ones(mpoints1.Count,1);
mpoints1_h = [mpoints1.Location homogeneous]
mpoints2_h = [mpoints2.Location homogeneous]
Evec = calibrated_fivepoint(mpoints1_h', mpoints2_h');
 
 numEs = size(Evec, 2);
 
 for i = 1: numEs
     EList{i} = reshape(Evec(:,i), 3, 3);
 end

%% Check error

minSumError = inf;

for i=1:length(EList)
  E = EList{i};
  i
  % Check determinant constraint! 
  error_determinate = det( E);
  % Check trace constraint
  error_trace = 2 *E*transpose(E)*E -trace( E*transpose(E))*E;

  % Check reprojection errors
  error_reprojection = diag( h_pts2 * E * h_pts1');
  sum_error_proj = sum(abs(error_reprojection))
  
  % Find E with the smallest error
  if (sum_error_proj < minSumError)
      minSumError = sum_error_proj;
      bestE = E;
  end
  
end


try
% Pose estimation
for i=1:1000
  % Estimate the essential matrix
  [E, minf] = estimateEssentialMatrix(mpoints1, mpoints2, cameraParams);
  if sum(minf)/numel(minf)<0.3, continue; end
  
  % Camera pose
  [orient,loc,vpf] = relativeCameraPose(E, cameraParams, mpoints1(minf), mpoints2(minf));
  if vpf>0.8, break; end
end
if vpf<=0.8, warning('Problem with camera pose estimate?'); end
% fh = figure; showMatchedFeatures(I1, I2, mpoints1(minf), mpoints2(minf), 'falsecolor'); 
catch
   continue;
end

% Camera matrices
R1 = eye(3);  t1 = zeros(1,3);
P1 = cameraMatrix(cameraParams, R1, t1);
[R2,t2] = cameraPoseToExtrinsics(orient, loc);
P2 = cameraMatrix(cameraParams, R2, t2);

% Triangulate
% wpi = triangulate(mpoints1(minf), mpoints2(minf), P1, P2);
%fh = figure;  sh = scatter3(wpi(:,1)',wpi(:,2)',wpi(:,3)','r.');  axis equal;

% Reprojected points
% xp1 = worldToImage(cameraParams,R1,t1,wpi)';  xp2 = worldToImage(cameraParams,R2,t2,wpi)';
% fh = figure;  
% subplot(1,2,1);  ih = imagesc(I1);  colormap(gray);  hold on;  sh = scatter(xp1(1,:),xp1(2,:),'r+');  hold off;
% subplot(1,2,2);  ih = imagesc(I2);  colormap(gray);  hold on;  sh = scatter(xp2(1,:),xp2(2,:),'r+');  hold off;

% Plot
% figure;  %axis([-10, 10, -10, 10, 0, 40]);  
% view(gca, 3);
% set(gca, 'CameraUpVector', [0, -1, 0]);  camorbit(gca, -120, 0, 'data', [0, 1, 0]);
% grid on;  xlabel('X');  ylabel('Y');  zlabel('Z');  hold on;
% pcsize = 0.5;
% ph1 = plotCamera('Size', pcsize, 'Location', t1, 'Orientation', R1, 'Color', 'g', 'Opacity', 0);
% ph2 = plotCamera('Size', pcsize, 'Location', t2, 'Orientation', R2, 'Color', 'r', 'Opacity', 0);
% sh = scatter3(wpi(:,1)',wpi(:,2)',wpi(:,3)','r.');


[yaw, pitch, roll] = dcm2angle(R2);
rollDiff = [rollDiff; roll];
pitchDiff = [pitchDiff; pitch];
yawDiff = [yawDiff; yaw];
end

figure;
plot(1:600, rollDiff, 'r');
figure;
plot(1:600, pitchDiff, 'g');
figure;
plot(1:600, yawDiff, 'b');