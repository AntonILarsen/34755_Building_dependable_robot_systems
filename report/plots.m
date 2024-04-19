%% basepath
basePath = "C:/Users/nicol/OneDrive/Documents/34755-Building dependable robot systems/";
%%
close all;
% Define the filename
filename = 'log_irdist.txt'; % Adjust the path if the file is in a different directory

% Load the data, skipping the header rows
data = load(filename, '-ascii'); % Adjust if your header has a different number of lines

% Extract columns into variables for clarity
timeSec = data(:, 1); % Time in seconds
timeSec = timeSec - timeSec(1);
distanceSensor2 = data(:, 3); % Distance measured by sensor 2 in meters

% Plotting distance from sensor 2
figure(1); % Creates a new figure
plot(timeSec, distanceSensor2);
title('Distance from Sensor 2 over Time');
xlabel('Time (sec)');
ylabel('Distance (m)');
grid on;

%% pose abs using IMU
filename = "log_pose_abs.txt";
figure(3);
hold on;
for i = 1:5

filepath = fullfile(basePath, sprintf('log_IMU_%d', i), filename);
pose_abs = load(filepath);
plot(pose_abs(:,2), pose_abs(:,3))
end 
grid on
axis equal
xlabel('position x');
ylabel('position y');
title('Position')
%% pose abs using purely pose
filename = "log_pose_abs.txt";
figure(4);
hold on;
for i = 1:5

filepath = fullfile(basePath, sprintf('log_%d', i), filename);
pose_abs = load(filepath);
plot(pose_abs(:,2), pose_abs(:,3))
end 
grid on
axis equal
xlabel('position x');
ylabel('position y');
title('Position Using Encoders');
%% Driving on only IMU
filename = "IMU_Pitch.txt";
figure(5);
hold on;

for i = 1:5  
    filepath = fullfile(basePath, sprintf('log_IMU_%d', i), filename);
    IMU_pitch = load(filepath);
    
    rawTime = IMU_pitch(:,3); % Original time data
    IMU_Pitch_data = IMU_pitch(:,2);

    % Adjust time data to handle resets
    adjustedTime = zeros(size(rawTime)); % Initialize adjusted time array
    adjustedTime(1) = rawTime(1); % Start with the first time value
    offset = 0; % Initialize offset for time resets
    for j = 2:length(rawTime)
        if rawTime(j) < rawTime(j-1)
            % Time reset detected, adjust by adding the last max time to subsequent times
            offset = adjustedTime(j-1);
        end
        adjustedTime(j) = rawTime(j) + offset;
    end

    % Normalize adjusted time to start at 0
    adjustedTime = adjustedTime - adjustedTime(1);

    % Plot the data with adjusted time
    plot(adjustedTime, IMU_Pitch_data);
    xlim([0, max(adjustedTime)]);
end

grid on;
xlabel('time');
ylabel('read pitch');
title('IMU Pitch');
hold off;
%% Marius iniitial IMU data plot 
filename = "IMU_Pitch.txt";
figure(5);
hold on;

for i = 1:5  
    filepath = fullfile(basePath, sprintf('M_log_%d', i), filename);
    IMU_pitch = load(filepath);
    
    rawTime = IMU_pitch(1:600,3); % Original time data
    IMU_Pitch_data = IMU_pitch(1:600,2);

    % Adjust time data to handle resets
    adjustedTime = zeros(size(rawTime)); % Initialize adjusted time array
    adjustedTime(1) = rawTime(1); % Start with the first time value
    offset = 0; % Initialize offset for time resets
    for j = 2:length(rawTime)
        if rawTime(j) < rawTime(j-1)
            % Time reset detected, adjust by adding the last max time to subsequent times
            offset = adjustedTime(j-1);
        end
        adjustedTime(j) = rawTime(j) + offset;
    end

    % Normalize adjusted time to start at 0
    adjustedTime = adjustedTime - adjustedTime(1);

    % Plot the data with adjusted time
    plot(adjustedTime, IMU_Pitch_data);
    xlim([0, max(adjustedTime)]);
end

grid on;
xlabel('time');
ylabel('read pitch');
title('IMU Pitch');
hold off;

%% pose abs marius plot 
filename = "log_pose_abs.txt";
figure(4);
hold on;
for i = 1:5

filepath = fullfile(basePath, sprintf('M_log_%d', i), filename);
pose_abs = load(filepath);
plot(pose_abs(:,2), pose_abs(:,3))
end 
grid on
axis equal
xlabel('position x');
ylabel('position y');
title('Position bplan3');
%% pose abs rull run 
filename = "log_pose_abs.txt";
figure(4);
hold on;
for i = 1:1

filepath = fullfile(basePath, sprintf('log_fullrun_%d', i), filename);
pose_abs = load(filepath);
plot(pose_abs(:,2), pose_abs(:,3))
end 
grid on
axis equal
xlabel('position x');
ylabel('position y');
title('Position bplan3');
%% pose test
filename = "log_pose_abs.txt";
figure(100);
hold on;
for i = 1:1

filepath = fullfile(basePath, sprintf('test_%d', i), filename);
pose_abs = load(filepath);
plot(pose_abs(:,2), pose_abs(:,3))
end 
grid on
axis equal
xlabel('position x');
ylabel('position y');
title('Position logtest');

