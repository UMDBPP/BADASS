clc
% set in software
target_ned = [1.0 1.0 0];
target_ned = target_ned ./ norm(target_ned)

% reading from BNO
quat_ned2imu = [0.1426; 0.9897; -0.0065; 0.0054];

% Calculate transform from IMU to body
%x BNO = -y BADASS
%y BNO = x BADASS
%z BNO = z badass
% 90deg about Z
ang_about_z = 90 * pi/180;
quat_imu2body = [[0; 0; 1]*sin(0.5*ang_about_z); cos(0.5*ang_about_z)]

% use BNO meas to form transform from ned to body
quat_ned2body = qmult(quat_ned2imu,quat_imu2body)

% transform target to body
target_body = qvrot(quat_ned2body,target_ned)
