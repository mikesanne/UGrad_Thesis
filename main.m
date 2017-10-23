    %% Matlab code to perform relative pose estimation
% Generated by Michael Sanne SNNMIC003
% 4th Year Engineering Thesis at The University of Cape Town
% 2017/10/20
rng('default');
clear;
close all;

    %% Loads Images from Data/1LoopDown Folder
% Load
images = imageDatastore('Data/1LoopDown/imagesUndistort_1loopDown');
K = load('Data/1LoopDown/camIntrinsicDown.txt')';

%images = imageDatastore('Data/rgbd_dataset_freiburg1_xyz/rgb');
%K = load('Data/rgbd_dataset_freiburg1_xyz/camIntrinsics.txt')';
cameraParams = cameraParameters('IntrinsicMatrix', K);
K_h = eye(4,3)*K;
ViconMatlabImport;
v = viewSet;
vId = 1;

    %% Import first 2 images
I1 = rgb2gray(readimage(images,1));
I2 = rgb2gray(readimage(images,2));

    %% Estimate Pose of first 2 images and retrieve P Matrices
% Features
mtsparm = 250;
points1 = detectSURFFeatures(I1, 'MetricThreshold', mtsparm);
feat1 = extractFeatures(I1, points1, 'Upright', true);

% Detect Features Image 2
points2 = detectSURFFeatures(I2, 'MetricThreshold', mtsparm);
feat2 = extractFeatures(I2, points2, 'Upright', true);

% Putative matches
mpairs_12 = matchFeatures(feat1, feat2, 'Unique', true);
mpoints1_12 = points1(mpairs_12(:,1));  mpoints2_12 = points2(mpairs_12(:,2));  

% Pose estimation
for i=1:1000
  % Estimate the essential matrix
  [E_12, minf_12] = estimateEssentialMatrix(mpoints1_12, mpoints2_12, cameraParams);
  if sum(minf_12)/numel(minf_12)<0.3, continue; end
  
  % Camera pose
  [orient_12,loc_12,vpf_12] = relativeCameraPose(E_12, cameraParams, mpoints1_12(minf_12), mpoints2_12(minf_12));
  if vpf_12>0.8, break; end
end
if vpf_12<=0.8, warning('Problem with camera pose estimate?'); end

% Camera matrices
R1_12 = eye(3);  t1_12 = zeros(1,3);
P1_12 = cameraMatrix(cameraParams, R1_12, t1_12);
[R2_12,t2_12] = cameraPoseToExtrinsics(orient_12, loc_12);
P2_12 = cameraMatrix(cameraParams, R2_12, t2_12);

[orientation,location] = extrinsicsToCameraPose(R1_12,t1_12);
v = addView(v, vId, 'Points', points1, 'Orientation', ...
    orientation, 'Location', location);
vId = vId + 1;

[orientation,location] = extrinsicsToCameraPose(R2_12,t2_12);
v = addView(v, vId, 'Points', points2, 'Orientation', ...
    orientation, 'Location', location);
vId = vId + 1;

v = addConnection(v, vId-2, vId-1, 'Matches', mpairs_12(minf_12, :));

mpointsMin1_12 = mpoints1_12(minf_12);
mpointsMin2_12 = mpoints2_12(minf_12);


    %% Loop
for i = 3:38%numel(images.Files)
close all;
    %% Import next image and Determine Relative Pose Between 2 and 3
    I3 = rgb2gray(readimage(images,i));

    % Detect Features Image 2
    points3 = detectSURFFeatures(I3, 'MetricThreshold', mtsparm);
    feat3 = extractFeatures(I3, points3, 'Upright', true);
    subplot(1,2,1);  ih = imagesc(I2);  colormap(gray);  hold on;  ph = plot(points2);  title('Image 1 Points 1');  hold off;
    subplot(1,2,2);  ih = imagesc(I3);  colormap(gray);  hold on;  ph = plot(points3);  title('Image 2 Points 2');  hold off;
    
    % Putative matches
    mpairs_23 = matchFeatures(feat2, feat3, 'Unique', true);
    mpoints2_23 = points2(mpairs_23(:,1));  mpoints3_23 = points3(mpairs_23(:,2));
    fh = figure; showMatchedFeatures(I2, I3, mpoints2_23, mpoints3_23, 'falsecolor'); title('Matched Points'); 

    % Pose estimation
    for i=1:1000
      % Estimate the essential matrix
      [E_23, minf_23] = estimateEssentialMatrix(mpoints2_23, mpoints3_23, cameraParams);
      if sum(minf_23)/numel(minf_23)<0.3, continue; end

      % Camera pose
      [orient_23,loc_23,vpf_23] = relativeCameraPose(E_23, cameraParams, mpoints2_23(minf_23), mpoints3_23(minf_23));
      if vpf_23>0.8, break; end
    end
    if vpf_23<=0.8, warning(sprintf('Problem with camera pose estimate? %s', num2str(vId))); 
        subplot(1,2,1);  ih = imagesc(I2);  colormap(gray);  hold on;  ph = plot(points2);  hold off;
        subplot(1,2,2);  ih = imagesc(I3);  colormap(gray);  hold on;  ph = plot(points3);  hold off;
        fh = figure; showMatchedFeatures(I2, I3, mpoints2_23(minf_23), mpoints3_23(minf_23), 'falsecolor'); 
        fh = figure; showMatchedFeatures(I2, I3, mpoints2_23, mpoints3_23, 'falsecolor');
        continue;
    end
    
    % Only compare inlier points
    mpointsMin2_23 = mpoints2_23(minf_23);
    mpointsMin3_23 = mpoints3_23(minf_23);
    mpairs_23 = mpairs_23(minf_23,:);
    fh = figure; showMatchedFeatures(I2, I3, mpointsMin2_23, mpointsMin2_23, 'falsecolor'); grid on; title('Matched Points Inliers'); 
    
    % Camera matrices
    R2_23 = eye(3);  t2_23 = zeros(1,3);
    P2_23 = cameraMatrix(cameraParams, R2_23, t2_23);
    [R3_23,t3_23] = cameraPoseToExtrinsics(orient_23, loc_23);
    P3_23 = cameraMatrix(cameraParams, R3_23, t3_23);

     %% Implement RANSAC to remove 3D outliers
%     wpa_23 = triangulate(mpointsMin2_23, mpointsMin3_23, P2_23, P3_23);
%     fh = figure;  sh = scatter3(wpa_23(:,1)',wpa_23(:,2)',wpa_23(:,3)','r.');  axis equal; title('3D Points');
%     
%     % Max Distance a point is allowed to be away from the plane
%     maxDistance = 0.5;
%     
%     referenceVector = [0, 0, 1];
%     % Computes the model equation for the line and outputs an array of
%     % inlier and outlierIndices
%     ptCloud = pointCloud(wpa_23);
%     [model, inlierIndices, outlierIndices] = pcfitplane(ptCloud, maxDistance, referenceVector);
%     wp_23 = wpa_23(inlierIndices,:);
%     
%     mpointsMin2_23 = mpointsMin2_23(inlierIndices,:);
%     mpointsMin3_23 = mpointsMin3_23(inlierIndices,:);
%     
%     wpa_23 = triangulate(mpointsMin2_23, mpointsMin3_23, P2_23, P3_23);
%     fh = figure;  sh = scatter3(wpa_23(:,1)',wpa_23(:,2)',wpa_23(:,3)','r.');  axis equal; title('Refined 3D Points');

    %% Match points in 2D between respective image planes using common features
   
    % Find intersecting points in common image
    [~, ia, ib] = intersect(mpointsMin2_12.Location,mpointsMin2_23.Location, 'rows');
    points1_123 = mpointsMin1_12(ia,:);
    points2_123 = mpointsMin2_12(ia,:);
    points2b_123 = mpointsMin2_23(ib,:);
    points3_123 = mpointsMin3_23(ib,:);

    %% Get X' in homogeneous form from inlier points
    wpa_23 = triangulate(points2b_123, points3_123, P2_23, P3_23);
    wph_23 = [wpa_23 zeros(size(wpa_23,1),1)+1]';
    fh = figure;  sh = scatter3(wpa_23(:,1)',wpa_23(:,2)',wpa_23(:,3)','r.');  axis equal; title('Matched 3D points Image 2&3');

    wpa_23 = triangulate(points1_123, points2_123, P1_12, P2_12);
    fh = figure;  sh = scatter3(wpa_23(:,1)',wpa_23(:,2)',wpa_23(:,3)','r.');  axis equal; title('Matched 3D points Image 1&2');

     %% Implement RANSAC to remove 3D outliers
%     % Max Distance a point is allowed to be away from the plane
%     maxDistance = 0.5;
%     
%     % Reference Vector for Plane
% %   referenceVector = [0, 0, 1];
%     
%     % Computes the model equation for the line and outputs an array of
%     % inlier and outlierIndices
%     ptCloud = pointCloud(wpa_23);
% %    [model, inlierIndices, outlierIndices] = pcfitplane(ptCloud, maxDistance, referenceVector);
%     [model, inlierIndices, outlierIndices] = pcfitplane(ptCloud, maxDistance);
%     wp_23 = wpa_23(inlierIndices,:);
%     %fh = figure;  sh = scatter3(wp_23(:,1)',wp_23(:,2)',wp_23(:,3)','r.');  axis equal;
%     
%     points2b_123 = points2b_123(inlierIndices,:);
% 
%     [~, ia, ib] = intersect(points2_123.Location,points2b_123.Location, 'rows');
%     points1_123 = mpointsMin1_12(ia,:);
%     points2_123 = mpointsMin2_12(ia,:);
%     points2b_123 = mpointsMin2_23(ib,:);
%     points3_123 = mpointsMin3_23(ib,:);
%     
%     wpa_23 = triangulate(points2b_123, points3_123, P2_23, P3_23);
%     fh = figure;  sh = scatter3(wp_23(:,1)',wp_23(:,2)',wp_23(:,3)','r.');  axis equal; title('Refined 3D Points');
     
    %% Determine Scale using distance between 2 points
    % Find distance between 2 points in world plane X from plane 1 and 2 
    
    scaleVector = zeros(1, length(points1_123)-1);
    for j = 1:length(points1_123)-1
         wp1_12 = triangulate(points1_123(j), points2_123(j), P1_12, P2_12);
         wp2_12 = triangulate(points1_123(j+1), points2_123(j+1), P1_12, P2_12);
         distance1_1 = sqrt ((wp2_12(1)-wp1_12(1))^2+(wp2_12(2)-wp1_12(2))^2+(wp2_12(3)-wp1_12(3))^2);
         
         wp1_23 = triangulate(points2b_123(j), points3_123(j), P2_23, P3_23);
         wp2_23 = triangulate(points2b_123(j+1), points3_123(j+1), P2_23, P3_23);
         distance2_1 = sqrt ((wp2_23(1)-wp1_23(1))^2+(wp2_23(2)-wp1_23(2))^2+(wp2_23(3)-wp1_23(3))^2);
         
        % Alternative Distance Formulation (Distance from camera 2 to point)
%        wp1_12 = triangulate(points1_123(j), points2_123(j), P1_12, P2_12);
%        c2_Location = v.Views.Location{vId-1};
%        distance1_1 = sqrt ((wp1_12(1)-c2_Location(1))^2+(wp1_12(2)-c2_Location(2))^2+(wp1_12(2)-c2_Location(3))^2);
%        
%        wp1_23 = triangulate(points2b_123(j), points3_123(j), P2_23, P3_23);
%        distance2_1 = sqrt ((wp1_23(1))^2+(wp1_23(2))^2+(wp1_23(2))^2);
        

        scale1 = distance1_1/distance2_1;

        scaleVector(1, j) = scale1;
    end
    
    % Compute scale difference between 2 points
    %scale = median(scaleVector);
    scale = trimmean(scaleVector, 25);
    if (scale > 5)
        disp('Hello');   
    end
    scaleMatrix = eye(4);
    scaleMatrix(1:3,1:3) = scale*scaleMatrix(1:3,1:3);

    %% Perform Rotation from X" to X
    R2_23 = R2_12;
    t2_23 = t2_12;
    P2_23 = cameraMatrix(cameraParams, R2_23, t2_23);
    
    t3_23 = double((R3_23*t2_23')'+scale*t3_23);
    R3_23 = R3_23*R2_23;
    P3_23 = cameraMatrix(cameraParams, R3_23, t3_23);
    
    [orientation,location] = extrinsicsToCameraPose(R3_23,t3_23);
    v = addView(v, vId, 'Points', points3, 'Orientation', ...
        orientation, 'Location', location);
    vId = vId + 1;
    
    v = addConnection(v, vId-2, vId-1, 'Matches', mpairs_23);
    
    %% Find Tracks across Multiple Views to perform Bundle Adjustment
    % TO DO: Convert orientation and rotation of previous 2 views into R
    % and t values after bundle adjustment
%     tracks = findTracks(v);
%     
%     camPoses = poses(v);
%     xyzPoints = triangulateMultiview(tracks, camPoses, cameraParams);
%     [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
%         tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
%         'PointsUndistorted', true);
%     v = updateView(v, camPoses);
%     
%     % Display camera poses.
%     camPoses = poses(v);
%     figure;
%     plotCamera(camPoses, 'Size', 0.2);
%     hold on
% 
%     % Exclude noisy 3-D points.
%     goodIdx = (reprojectionErrors < 5);
%     xyzPoints = xyzPoints(goodIdx, :);
% 
%     % Display the 3-D points.
%     pcshow(xyzPoints, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
%         'MarkerSize', 45);
%     grid on
%     title('Bundle Adjusted 3D Points');
%     hold off

    %% Compute world points
    wpi_12 = triangulate(points1_123, points2_123, P1_12, P2_12);
    wpi_23 = triangulate(points2b_123, points3_123, P2_23, P3_23);
    fh = figure;  sh = scatter3(wpi_23(:,1)',wpi_23(:,2)',wpi_23(:,3)','r.');  axis equal; title('3D Points Image 2&3 Scaled');
    
    %% Plot
    figure;  %axis([-10, 10, -10, 10, 0, 20]);  
    view(gca, 3);
    set(gca, 'CameraUpVector', [0, -1, 0]);  camorbit(gca, -120, 0, 'data', [0, 1, 0]);
    grid on;  xlabel('X');  ylabel('Y');  zlabel('Z');  hold on;
    pcsize = 0.5;
    %ph1 = 
    plotCamera('Size', pcsize, 'Location', t2_23, 'Orientation', R2_23, 'Color', 'g', 'Opacity', 0);
    %ph2 = 
    plotCamera('Size', pcsize, 'Location', t3_23, 'Orientation', R3_23, 'Color', 'r', 'Opacity', 0);
    %ph3 = 
    plotCamera('Size', pcsize, 'Location', t1_12, 'Orientation', R1_12, 'Color', 'b', 'Opacity', 0);
    sh = scatter3(wpi_12(:,1)',wpi_12(:,2)',wpi_12(:,3)','r.');
    sh = scatter3(wpi_23(:,1)',wpi_23(:,2)',wpi_23(:,3)','g.');
    %sh = scatter3(wph_23(:,1),wph_23(:,2),wph_23(:,3),'b.');
    title('Previous 3 Cameras and matched points');

    figure;  %axis([-10, 10, -10, 10, 0, 20]);  
    view(gca, 3);
    set(gca, 'CameraUpVector', [0, -1, 0]);  camorbit(gca, -120, 0, 'data', [0, 1, 0]);
    grid on;  xlabel('X');  ylabel('Y');  zlabel('Z');  hold on;
    pcsize = 0.5;
    camPoses = poses(v);
    plotCamera(camPoses, 'Size', 0.2);
    hold on
%     sh = scatter3(xyzPoints(:,1)', xyzPoints(:,2)', xyzPoints(:,3)', 'b.');
    title('Camera Poses with Bundle Adjustment');
    
     %% Remap Variables for loop
     
    I1 = I2;
    I2 = I3;

    points1 = points2;
    feat1 = feat2;

    points2 = points3;
    feat2 = feat3;

    mpairs_12 = mpairs_23;
    mpointsMin1_12 = mpointsMin2_23;
    mpointsMin2_12 = mpointsMin3_23;
    
    R1_12 = R2_23;
    t1_12 = t2_23;
    P1_12 = P2_23;
    R2_12 = R3_23;
    t2_12 = t3_23;
    P2_12 = P3_23;

end

    %% Display Camera Poses
% Display the refined camera poses and 3-D world points.

% Display camera poses.
camPoses = poses(v);
figure;
plotCamera(camPoses, 'Size', 0.2);
hold on
