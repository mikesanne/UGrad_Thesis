function [rollVicon, pitchVicon, yawVicon, xVicon, yVicon, zVicon] = RPYXYZ_Vicon(step)
%function correlationFactors = Benchmark(filename)

% get Vicon data
%vicon = load ('../Data/1LoopDown/vicon_1loopDown_copy.txt');
vicon = LinearTime();
[nrow, ncol] = size(vicon);

% Transformation matrix from Quadrotor to Camera Frame (downward looking camera)
% units: millimeters
% T = [ R t; 0 0 0 1]
T_VicCam = [0.008597565454506  -0.991125267392933   0.002125523552400    -92.980069288466 ; ...
 -0.995859015266209   0.002879412699678   0.003052403580956     26.672207606184 ; ...
  0.010278931642942   0.011669516433584  -0.995463464052292  -255.416164272468 ; ...
  0.000000000000000   0.000000000000000   0.000000000000000       1.000000000000];

% compute the relative changes

rollVicon = 0;
yawVicon = 0;
pitchVicon = 0;
xVicon = 0;
yVicon = 0;
zVicon = 0;

for i=1+step:step:nrow
    T_0Pre = T_VicCam*[euler_to_R(vicon(i-step,5:7)) vicon(i-1,2:4)'; 0 0 0 1];
    T_0Cur = T_VicCam*[euler_to_R(vicon(i,5:7)) vicon(i,2:4)'; 0 0 0 1];
    T_PreCur = inv(T_0Pre)*T_0Cur;
    
    tVicon = T_PreCur(1:3,4)/norm(T_PreCur(1:3,4));
    y = tVicon(1);
    x = tVicon(2);
    z = tVicon(3);
    xVicon = [xVicon; -x];
    yVicon = [yVicon; -y];
    zVicon = [zVicon; -z];
    
    [yaw, roll, pitch] = dcm2angle(T_PreCur(1:3,1:3));
    rollVicon = [rollVicon; roll];
    pitchVicon = [pitchVicon; pitch];
    yawVicon = [yawVicon; yaw];

end

% Plot Ground Truth Values
% figure;
%     plot(1:step:nrow,xVicon, 'b');
% figure;
%     plot(1:step:nrow,yVicon, 'g');
% figure;
%     plot(1:step:nrow,zVicon, 'r');
% figure;
%     plot(1:step:nrow,yawVicon, 'b');
% figure;
%     plot(1:step:nrow,pitchVicon, 'g');
% figure;
%     plot(1:step:nrow,rollVicon, 'r');
end

  