%% Extra Code that can be implemented in main

     %% Implement RANSAC to remove 3D outliers
%     wpa_23 = triangulate(mpointsMin2_23, mpointsMin3_23, P2_23, P3_23);
%     fh = figure;  sh = scatter3(wpa_23(:,1)',wpa_23(:,2)',wpa_23(:,3)','r.');  axis equal; title('3D Points');
%     
%     % Max Distance a point is allowed to be away from the plane
%     maxDistance = 5;
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

     %% Remove Outliers based on distance
%     distanceVector = zeros(1, length(mpointsMin2_23));
%     Inliers = 0;
%     for j = 1:length(mpointsMin2_23)
%         wp1_23 = triangulate(mpointsMin2_23(j), mpointsMin3_23(j), P2_23, P3_23);
%         distance2_1 = sqrt ((wp1_23(1))^2+(wp1_23(2))^2+(wp1_23(2))^2);
%         distanceVector(1, j) = distance2_1;
%     end
%     meanDistance = trimmean(distanceVector, 25);
%     for j = 1:length(mpointsMin2_23)
%         if (distanceVector(1, j) < meanDistance*2 && distanceVector(1, j) > meanDistance*0.5)
%             Inliers = [Inliers; j];
%         end
%     end
%     Inliers = Inliers(2:length(Inliers));
%    mpointsMin2_23 = mpointsMin2_23(Inliers,:);
%    mpointsMin3_23 = mpointsMin3_23(Inliers,:);
 
     %% Implement RANSAC to remove 3D outliers
%     % Max Distance a point is allowed to be away from the plane
%     maxDistance = 5;
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
%    mpointsMin2_23 = mpoints2_23(inlierIndices);
%    mpointsMin3_23 = mpoints3_23(inlierIndices);
%    mpairs_23 = mpairs_23(inlierIndices,:);
% 
%     wpa_23 = triangulate(points2b_123, points3_123, P2_23, P3_23);
%     fh = figure;  sh = scatter3(wp_23(:,1)',wp_23(:,2)',wp_23(:,3)','r.');  axis equal; title('Refined 3D Points');
     
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
%     [R2_23,t2_23] = cameraPoseToExtrinsics(v.Views.Orientation{vId-2}, v.Views.Location{vId-2});
%     P2_23 = cameraMatrix(cameraParams, R2_23, t2_23);
%     [R3_23,t3_23] = cameraPoseToExtrinsics(v.Views.Orientation{vId-1}, v.Views.Location{vId-1});
%     P3_23 = cameraMatrix(cameraParams, R3_23, t3_23);

     %% Find distance between corresponding points in world plane X from plane 1 and 2 
%     scaleVector = zeros(1, length(points1_123)-1);
%     for j = 1:length(points1_123)-1
%          wp1_12 = triangulate(points1_123(j), points2_123(j), P1_12, P2_12);
%          wp2_12 = triangulate(points1_123(j+1), points2_123(j+1), P1_12, P2_12);
%          distance1_1 = sqrt ((wp2_12(1)-wp1_12(1))^2+(wp2_12(2)-wp1_12(2))^2+(wp2_12(3)-wp1_12(3))^2);
%          
%          wp1_23 = triangulate(points2b_123(j), points3_123(j), P2_23, P3_23);
%          wp2_23 = triangulate(points2b_123(j+1), points3_123(j+1), P2_23, P3_23);
%          distance2_1 = sqrt ((wp2_23(1)-wp1_23(1))^2+(wp2_23(2)-wp1_23(2))^2+(wp2_23(3)-wp1_23(3))^2);
% 
%         scale1 = distance1_1/distance2_1;
% 
%         scaleVector(1, j) = scale1;
%     end
%     
%     % Compute scale average between 2 planes
%     scale = trimmean(scaleVector, 50);