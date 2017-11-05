function viconNew = LinearTime()

%% Import Vicon Data
vicon = load ('../Data/1LoopDown/vicon_1loopDown.txt');

%% Import Image Data
filename = '../Data/1LoopDown/imagesUndistort_1loopDown.txt';
delimiter = ' ';
startRow = 3;
formatSpec = '%f%*s%*s%[^\n\r]';
fileID = fopen(filename,'r');
textscan(fileID, '%[^\n\r]', startRow-1, 'WhiteSpace', '', 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true, 'ReturnOnError', false);
fclose(fileID);
imageTimes = dataArray{:, 1};
clearvars filename delimiter startRow formatSpec fileID dataArray ans;

viconNew = zeros(size(imageTimes,1), 10);

%% Linearise
frequency = 0.0050;
viconNew(1,:) = vicon(1,:);
for i = 2:size(imageTimes)
    timer = imageTimes(i);
    for j = 1:size(vicon,1)-1
        if (vicon(j,1) <= timer && vicon(j+1) > timer)
            time1 = vicon(j,1);
            ratio = (timer-time1)/frequency;
            viconNew(i,:) = (vicon(j+1,:)-vicon(j,:))*ratio+vicon(j,:);
            break;
        end
    end
end