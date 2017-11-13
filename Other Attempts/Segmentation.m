%% Matlab code to perform relative pose estimation

%% Segmentation Method
% Michael Sanne
% 24/10/2017
% Testing with 10 images per segment

rng('default');
clear;
close all;

    %% Loads Images from Data/1LoopDown Folder
    % Load images and camera intrinsics from file
    images = imageDatastore('Data/1LoopDown/imagesUndistort_1loopDown_copy');
    K = load('Data/1LoopDown/camIntrinsicDown.txt')';
    cameraParams = cameraParameters('IntrinsicMatrix', K);
    
    % Load ground truth Data as groundTruthPoses
    ViconMatlabImport;
    
    % Instantiate a view set to keep track of poses
    v = viewSet;
    vId = 1;
    
for x = 1:10:100   
        %% Import first 2 images
        I1 = rgb2gray(readimage(images,x));

        %% Estimate Pose of first 2 images and retrieve P Matrices

        % Detect Features for Image 1
        mtsparm = 250;
        points1 = detectSURFFeatures(I1, 'MetricThreshold', mtsparm);
        feat1 = extractFeatures(I1, points1);

        % Generate Camera matrices
        R1_12 = eye(3);  t1_12 = zeros(1,3);
        P1_12 = cameraMatrix(cameraParams, R1_12, t1_12);

        % Add estimated pose values to the view set
        [orientation,location] = extrinsicsToCameraPose(R1_12,t1_12);
        v = addView(v, vId, 'Points', points1, 'Orientation', ...
            orientation, 'Location', location);
        vId = vId + 1;

        % Camera matrices from the estimated pose
        R1 = eye(3);  t1 = zeros(1,3);
        P1 = cameraMatrix(cameraParams, R1, t1);

        %% Loop
    for i = (x+1):(x+9)
    rng('default');
        %% Import next image and Determine Relative Pose Between 2 and 3
        I2 = rgb2gray(readimage(images,i));

        % Detect Features Image i and extract features
        points2 = detectSURFFeatures(I2, 'MetricThreshold', mtsparm);
        feat2 = extractFeatures(I2, points2);

        % Putative matches between image i and image i-1
        mpairs = matchFeatures(feat1, feat2, 'Unique', true);
        mpoints1 = points1(mpairs(:,1));  mpoints2 = points2(mpairs(:,2));

        % Pose estimation
        for j=1:1000

          % Estimate the essential matrix with minimum 30% inliers
          [E, minf] = estimateEssentialMatrix(mpoints1, mpoints2, cameraParams);
          if sum(minf)/numel(minf)<0.3, continue; end

          % Calculate Camera pose from the essential matrix
          [orient,loc,vpf] = relativeCameraPose(E, cameraParams, mpoints1(minf), mpoints2(minf));
          if vpf>0.8, break; end
        end
        if vpf<=0.8, warning(sprintf('Problem with camera pose estimate? %s', num2str(vId))); 
            subplot(1,2,1);  ih = imagesc(I2);  colormap(gray);  hold on;  ph = plot(points1);  hold off;
            subplot(1,2,2);  ih = imagesc(I2);  colormap(gray);  hold on;  ph = plot(points2);  hold off;
            fh = figure; showMatchedFeatures(I1, I2, mpoints1(minf), mpoints2(minf), 'falsecolor'); 
            fh = figure; showMatchedFeatures(I1, I2, mpoints1, mpoints2, 'falsecolor');
            continue;
        end

        % Only compare inlier points
        mpointsMin1 = mpoints1(minf);
        mpointsMin2 = mpoints2(minf);
        mpairs = mpairs(minf,:);

        % Camera matrices from the estimated pose
        [R2,t2] = cameraPoseToExtrinsics(orient, loc);
        P2 = cameraMatrix(cameraParams, R2, t2);

        v = addView(v, vId, 'Points', points2, 'Orientation', ...
            orient, 'Location', loc);
        vId = vId + 1;

        v = addConnection(v, x, i, 'Matches', mpairs);

        %% Find Tracks across Multiple Views and perform Bundle Adjustment

        % Find Tracks in the viewSet across multiple views
        tracks = findTracks(v, x:i);

        % Get camera poses from the viewset
        camPoses = poses(v, x:i);

        % Compute world points triangulated from multiple camera views
        xyzPoints = triangulateMultiview(tracks, camPoses, cameraParams);

        % Perform bundle adjustment on the current set in the viewset
        [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
            tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
            'PointsUndistorted', true);

        % Update the viewset with the new camera poses
        v = updateView(v, camPoses);
        
        % Exclude noisy 3-D points.
        goodIdx = (reprojectionErrors < 2);
        xyzPoints = xyzPoints(goodIdx, :);
    end
        %% Plot

        % Plot all camera poses from the viewset in one image
        figure;  axis([-5, 5, -5, 5, 0, 10]);  
        view(gca, 3);
        set(gca, 'CameraUpVector', [0, -1, 0]);  camorbit(gca, -120, 0, 'data', [0, 1, 0]);
        grid on;  xlabel('X');  ylabel('Y');  zlabel('Z');  hold on;
        pcsize = 0.2;
        camPoses = poses(v,x:x+9);
        plotCamera(camPoses, 'Size', 0.2);
        hold on
        scatter3(xyzPoints(:,1)', xyzPoints(:,2)', xyzPoints(:,3)', 'b.');
        title('Camera Poses with Bundle Adjustment');

    
end
