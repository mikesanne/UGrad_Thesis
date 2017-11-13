% Windowed Global Bundle Adjustment
% SNNMIC003 Update 13/11/2017
% This function aims to perform windowed bundle adjustment ona set of poses
% INPUT_PARAMS
%   v = A viewset containing the original poses
%   cameraParams = The intrinsic values of the camera in the form of a
%   cameraParams object
%   windowSize = the number of frames to incorporate on every iteration of
%   the bundle adjustment
%
% OUTPUT_PARAMS
%   adjustedV = The resulting viewSet containing the bundle adjusted poses
%   xyzPoints = Points triangulated from all views
%
%%
function [adjustedV, xyzPoints] = Windowed_Bundle_Adjustment(v, cameraParams, windowSize)
         %% Find Tracks across Multiple Views to perform Bundle Adjustment
    
    adjustedV = v;     
    for i = windowSize:adjustedV.NumViews
        i
        tracks = findTracks(adjustedV, i-windowSize:i);
        camPoses = poses(adjustedV, i-windowSize:i);
        xyzPoints = triangulateMultiview(tracks, camPoses, cameraParams);
        [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
            tracks, camPoses, cameraParams, 'FixedViewId', [1,2], ...
            'PointsUndistorted', true);
         adjustedV = updateView(adjustedV, camPoses);
    end
    
    %Plot(adjustedV);
end