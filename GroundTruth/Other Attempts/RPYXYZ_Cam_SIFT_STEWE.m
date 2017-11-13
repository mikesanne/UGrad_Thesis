%RPYXYZ_Cam_STEWE

function [tCam, xCam, yCam, zCam, RCam, rollCam, pitchCam, yawCam] = RPYXYZ_Cam_STEWE(step)

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
fprintf('Computing frames and descriptors.\n') ;
[frames1,descr1,gss1,dogss1] = sift( I1, 'Verbosity', 0 ) ;
[frames2,descr2,gss2,dogss2] = sift( I2, 'Verbosity', 0 ) ;

descr1=uint8(512*descr1) ;
descr2=uint8(512*descr2) ;
matches=siftmatch( descr1, descr2, 3 ) ;

%plotmatches(I1,I2,frames1(1:2,:),frames2(1:2,:),matches) ;
mpoints1 = frames1(1:2,matches(1,:))';
mpoints2 = frames2(1:2,matches(2,:))';  

    h_pts1 = [mpoints1 ones(size(mpoints1,1),1)];
    h_pts2 = [mpoints2 ones(size(mpoints2,1),1)];
    
    h_pts1 = K'\h_pts1';
    h_pts2 = K'\h_pts2';
    h_pts1 = h_pts1';
    h_pts2 = h_pts2';
    
    Evec = calibrated_fivepoint(h_pts2', h_pts1');
    
     numEs = size(Evec, 2);
 
     for i = 1: numEs
        EList{i} = reshape(Evec(:,i), 3, 3);
     end
     
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
    [R, t, good, mask] = cv.recoverPose(bestE, mpoints1, mpoints2, 'CameraMatrix', K');
     
     R2 = R';
     t2 = t';
     P2 = cameraMatrix(cameraParams, R2, t2);
%     
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