yawError = 0;

for i = 2:size(rollCam)
    yawError = yawError + ((yawCam(i) - yawVicon(i))/yawVicon(i))^2;
end
yawError = sqrt(yawError/size(yawVicon,1))

rollError = 0;

for i = 2:size(rollCam)
    rollError = rollError + ((rollCam(i) - rollVicon(i))/rollVicon(i))^2;
end
rollError = sqrt(rollError/size(rollCam,1))

pitchError = 0;
for i = 2:size(pitchCam)
    pitchError = pitchError + ((pitchCam(i) - pitchVicon(i))/pitchVicon(i))^2;
end
pitchError = sqrt(pitchError/size(pitchCam,1))


xError = 0;
for i = 2:size(xCam)
    xError = xError + ((xCam(i) - xVicon(i))/xVicon(i))^2;
end
xError = sqrt(xError/size(xCam,1))

yError = 0;
for i = 2:size(yCam)
    yError = yError + ((yCam(i) - yVicon(i))/yVicon(i))^2;
end
yError = sqrt(yError/size(yCam,1))

zError = 0;
for i = 2:size(zCam)
    zError = zError + ((zCam(i) - zVicon(i))/zVicon(i))^2;
end
zError = sqrt(zError/size(zCam,1))