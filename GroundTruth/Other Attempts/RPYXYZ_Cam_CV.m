function [tCam, xCam, yCam, zCam, RCam, rollCam, pitchCam, yawCam] = RPYXYZ_Cam(step)

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
    


% % Features
% mtsparm = 250;
% points1 = detectSURFFeatures(I1);
% feat1 = extractFeatures(I1, points1, 'Upright', true);
% points2 = detectSURFFeatures(I2);
% feat2 = extractFeatures(I2, points2, 'Upright', true);

    % Putative matches
    mpairs = matchFeatures(feat1, feat2, 'Unique', true);
    mpoints1 = points1(mpairs(:,1));  mpoints2 = points2(mpairs(:,2));
    mpoints1_loc = struct2table(mpoints1); mpoints2_loc = struct2table(mpoints2);
    mpoints1_loc = mpoints1_loc{:,1}; mpoints2_loc = mpoints2_loc{:,1};

 try
% % Pose estimation
% for i=1:1000
%   % Estimate the essential matrix
%   [E, minf] = estimateEssentialMatrix(mpoints1, mpoints2, cameraParams);
%   if sum(minf)/numel(minf)<0.3, continue; end
%   
%   % Camera pose
%   [orient,loc,vpf] = relativeCameraPose(E, cameraParams, mpoints1(minf), mpoints2(minf));
%   if vpf>0.8, break; end
% end
% if vpf<=0.9, warning('Problem with camera pose estimate?'); end
 catch
%     disp('Image fail');
%     i
%    continue;
 end

    [E, mask] = cv.findEssentialMat(mpoints1_loc, mpoints2_loc, 'CameraMatrix', K');
    mask = logical(mask);
    mpairs = mpairs(mask,:); mpoints1_loc = mpoints1_loc(mask,:); mpoints2_loc = mpoints2_loc(mask,:);
    
    [R, t, good, mask] = cv.recoverPose(E, mpoints1_loc, mpoints2_loc, 'CameraMatrix', K');
    mask = logical(mask);
    mpairs = mpairs(mask,:); mpoints1_loc = mpoints1_loc(mask,:); mpoints2_loc = mpoints2_loc(mask,:);    
   
    R2 = R';
    t2 = t';
    P2 = cameraMatrix(cameraParams, R2, t2);

% Camera matrices
R1 = eye(3);  t1 = zeros(1,3);
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