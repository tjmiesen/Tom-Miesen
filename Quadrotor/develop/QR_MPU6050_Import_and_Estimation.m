%% RAINDrop Quadrotor Simulator: MPU6050 Data Import and Estimation
%Thomas Miesen

% ANGULAR RATE IMPORT
%Filtering only done on r and p values

measurement_buffer.ang(1:2,1) = quad.k_lowpass_ang*measured(5:6,t)+(1-quad.k_lowpass_ang)*measurement_buffer.ang(1:2,2);
measurement_buffer.ang(3,1) = measured(7,t);
%Store old rate
measurement_buffer.ang(1:2,2) = measurement_buffer.ang(1:2,1);

% ANGLE BASED OFF ANGULAR RATE
%Initial angle estimation from angular rate
measurement_buffer.rot(:,1) = measurement_buffer.rot(:,1) + quad.rot_sample_rate*(measurement_buffer.ang(:,1));

% ACCEL IMPORT
%Accelerometer data import (sampled at the same time as gyro data)
%Add noise to accel data
accel_state(:,1) = accel_state(:,1) + noise.accel_measurement*randn();

%Low-pass filter on accelerometer data

measurement_buffer.accel = quad.k_lowpass_accel*accel_state(:,1) + (1-quad.k_lowpass_accel)*accel_state(:,2);
%Store old accel data
accel_state(:,2) = measurement_buffer.accel;

% ACCEL ANGLE
%Angle determination from accelerometer data (note we're using the true yaw angle because the acceleration measurement is with respect to the body frame)
measurement_buffer.accel_rot(1,t) = atan((cos(state(6,t))*measurement_buffer.accel(2,1) + sin(state(6,t))*measurement_buffer.accel(1,1))/((measurement_buffer.accel(3,1) + qsim.g)));
measurement_buffer.accel_rot(2,t) = atan((cos(state(6,t))*measurement_buffer.accel(1,1) - sin(state(6,t))*measurement_buffer.accel(2,1))/(sqrt((cos(state(6,t))*measurement_buffer.accel(2,1) + sin(state(6,t))*measurement_buffer.accel(1,1))^2 + (measurement_buffer.accel(3,1)+qsim.g)^2)));

% FINAL ANGLE ESTIMATION (COMMON FILTER)
%Common filter (for roll and pitch only)
measurement_buffer.rot(1:2,1) = (1-quad.k_common_gain)*measurement_buffer.rot(1:2,1) + quad.k_common_gain*measurement_buffer.accel_rot(1:2,t);