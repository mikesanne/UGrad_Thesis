%RPYXYZ_Cam_STEWE

function [tCam, xCam, yCam, zCam, RCam, rollCam, pitchCam, yawCam] = RPYXYZ_Cam_RANSAC(step)

%RPY relative camera between poses
images = imageDatastore('../Data/1LoopDown/imagesUndistort_1loopDown');
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

for i = 1+step:step:numel(images.Files)
    rng('default');
    i
% Load
I1 = rgb2gray(readimage(images,i-step));
I2 = rgb2gray(readimage(images,i));

% Detect Features for Image 1
    detector = cv.FeatureDetector('SIFT');
    extractor = cv.DescriptorExtractor('SIFT');
    points1 = detector.detect(I1);
    feat1 = extractor.compute(I1, points1);
    
    % Detect Features for Image 2
    points2 = detector.detect(I2);
    feat2 = extractor.compute(I2, points2);
    
    % Match Features between image 1 and image 2 - Putative matches
    mpairs = matchFeatures(feat1, feat2, 'Unique', true);
    mpoints1 = points1(mpairs(:,1));  mpoints2 = points2(mpairs(:,2));
    mpoints1_loc = struct2table(mpoints1); mpoints2_loc = struct2table(mpoints2);
    mpoints1_loc = mpoints1_loc{:,1}; mpoints2_loc = mpoints2_loc{:,1};
    %fh = figure; showMatchedFeatures(I1, I2, mpoints1_loc, mpoints2_loc, 'falsecolor'); 

    for j=1:100
    
    [mpoints1_loc, mpoints2_loc, ~, ~, ~, E] = ransac5pE(mpoints1_loc', mpoints2_loc', 0.8, K');
    mpoints1_loc = mpoints1_loc';
    mpoints1_loc = [mpoints1_loc(:,1:2)];
    mpoints2_loc = mpoints2_loc';
    mpoints2_loc = [mpoints2_loc(:,1:2)];
    [orient,loc,vpf] = relativeCameraPose(E, cameraParams, mpoints1_loc, mpoints2_loc);
    if vpf>0.8, break; end
    end
if vpf<=0.8, warning('Problem with camera pose estimate?'); end
%    [R, t, good, mask] = cv.recoverPose(E, mpoints1_loc, mpoints2_loc, 'CameraMatrix', K');
     
%      R2 = R';
%      t2 = t';
%      P2 = cameraMatrix(cameraParams, R2, t2);
%     
    R1 = eye(3);  t1 = zeros(1,3);
    P1 = cameraMatrix(cameraParams, R1, t1);
    [R2,t2] = cameraPoseToExtrinsics(orient, loc);
    P2 = cameraMatrix(cameraParams, R2, t2);

     R1 = eye(3);
     t1 = zeros(1,3);
     P1 = cameraMatrix(cameraParams, R1, t1);

tCam = [tCam; t2(1:3)];
xCam = [xCam; t2(1)];
yCam = [yCam; t2(2)];
zCam = [zCam; t2(3)];

[yaw, pitch, roll] = dcm2angle(R2);
RCam = [RCam ; R2];
rollCam = [rollCam; roll];
pitchCam = [pitchCam; pitch];
yawCam = [yawCam; -yaw];

end