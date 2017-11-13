function [tCam, xCam, yCam, zCam, RCam, rollCam, pitchCam, yawCam] = RPYXYZ_Cam_SIFT(step)

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
%Reads in images I1 and I2 in Grayscale
I1 = rgb2gray(readimage(images,i-step));
I2 = rgb2gray(readimage(images,i));

%Recomputes images (Review SIFT algorithm and paper to understand why)
% I1=I1-min(I1(:)) ;
% I1=I1/max(I1(:)) ;
% I2=I2-min(I2(:)) ;
% I2=I2/max(I2(:)) ;

fprintf('Computing frames and descriptors.\n') ;
[frames1,descr1,gss1,dogss1] = sift( I1, 'Verbosity', 0 ) ;
[frames2,descr2,gss2,dogss2] = sift( I2, 'Verbosity', 0 ) ;

descr1=uint8(512*descr1) ;
descr2=uint8(512*descr2) ;
matches=siftmatch( descr1, descr2, 3 ) ;

%plotmatches(I1,I2,frames1(1:2,:),frames2(1:2,:),matches) ;
mpoints1 = frames1(1:2,matches(1,:))';
mpoints2 = frames2(1:2,matches(2,:))';  

try
% Pose estimation
for j=1:1000
  % Estimate the essential matrix
  [E, minf] = estimateEssentialMatrix(mpoints1, mpoints2, cameraParams);
  if sum(minf)/numel(minf)<0.3, continue; end
  
  % Camera pose
  [orient,loc,vpf] = relativeCameraPose(E, cameraParams, mpoints1(minf,:), mpoints2(minf,:));
  if vpf>0.8, break; end
end
if vpf<=0.9, warning('Problem with camera pose estimate?'); end
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
