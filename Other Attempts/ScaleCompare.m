% Scale Compare
scale_vecGT = 1;
[~, ~, ~, xVicon, yVicon, zVicon] = RPYXYZ_Vicon(1);
    %% Determine Scale from GT
    
for 3:size(xVicon)    
    tGT = [xVicon(i)-xVicon(i-frame_step) yVicon(i)-yVicon(i-frame_step) zVicon(i)-zVicon(i-frame_step)];
    t2GT = [xVicon(frame_step+1)-xVicon(1) yVicon(frame_step+1)-yVicon(1) zVicon(frame_step+1)-zVicon(1)];
    scale = norm(tGT)/norm(t2GT);
    scale_vecGT = [scale_vecGT, scale]; 
end