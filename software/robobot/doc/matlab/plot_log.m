close all
clear
%%
% Mixer logfile
% Wheel base used in calculation: 0.22
% 1 	Time (sec)
% 2 	Linear velocity (m/s)
% 3 	Turnrate (rad/sec) positive is CCV
% 4 	Desired left wheel velocity (m/s)
% 5 	Desired right wheel velocity (m/s)
mixer = load("log_mixer.txt");
% Pose related logfile
% 1 	Time (sec)
% 2 	Reference for left and right motor (m/sec)
% 3 	Measured velocity for left and right motor (m/sec)
% 4 	Value after Kp (V)
% 5 	Value after Lead (V)
% 6 	Integrator value (V)
% 7 	Motor voltage output (V)
% 8 	Is output limited (1=limited)
% PID parameters
% 	Kp = 15
% 	tau_d = 0.1, alphs = 1
% 	tau_i = 0.1 (used=1)
% 	sample time = 7.0 ms
% 	(derived values: le0=1, le1=0, lu1=0, ie=0.035)
m0 = load("log_motor_0.txt");
m1 = load("log_motor_1.txt");
% Pose related logfile
% 1 	Time (sec)
% 2,3 	Velocity left, right (m/s)
% 4 	Turnrate (rad/s)
% 5,6 	Position left, right (m)
% 7 	heading (rad)
pose = load("log_pose.txt");
% Gyro logfile
% 1 	Time (sec)
% 2-4 	Gyro (x,y,z)
% Gyro offset 0 0 0
gyro = load('log_gyro.txt');
%gyro2 = load('../log_20230619_154704.486/log_gyro.txt');
acc = load('log_acc.txt');
%% Motor control
figure(10)
hold off
plot(mixer(:,1) - mixer(1,1), mixer(:,2))
hold on
plot(mixer(:,1) - mixer(1,1), mixer(:,3))
%
plot(m0(:,1) - mixer(1,1), m0(:,2))
plot(m1(:,1) - mixer(1,1), m1(:,2))
plot(m0(:,1) - mixer(1,1), m0(:,7))
plot(m1(:,1) - mixer(1,1), m1(:,7))
grid on
legend('lin vel','turnrate','m0 ref', 'm1 ref', 'm0 V', 'm1 V')
title('Motor control')
%% Pose
figure(20)
hold off
plot(pose(:,5), pose(:,6))
grid og
axis equal
title('Position')
%% Heading
figure(50)
hold off
plot(pose(:,1) - pose(1,1), pose(:,4))
hold on
plot(pose(:,1) - pose(1,1), pose(:,2))
plot(pose(:,1) - pose(1,1), pose(:,3))
plot(pose(:,1) - pose(1,1), pose(:,7))
grid on
axis([0,10,-3.5,3.5])
title('heading')
legend('turnrate','vel left', 'vel right', 'heading')
%% IMU
figure(1000)
hold off
plot(gyro2(:,1) - gyro2(1,1), gyro2(:,2))
grid on
hold on
plot(gyro(:,1) - gyro(1,1), gyro(:,2),'r')
title('Gyro with 25 sample averaging')
