function plot_QR_sim( qsim, measured, state, measurement_buffer, motor_cmd, u, des )
%PLOT_QR_SIM    Plot the results from a quadrotor simulation    
%   Inputs: - qsim
%           - measured
%           - state
%           - measurement_buffer
%           - motor_cmd
%           - u
%           - des
%
% T. Miesen -- RAIN Lab

close all;

figure
[~,n] = size(state);
plot((1:n-1)*qsim.dt, measured(1:3,1:n-1))
xlabel('Time')
ylabel('Position Output')
legend('X','Y','Z')
title('Measured Position (mm)')

%Plot (true) rot dynamics
figure
plot((1:n-1)*qsim.dt, state(4:6,1:n-1)*180/pi())
xlabel('Time')
ylabel('Rotation Output')
legend('R','P','Yaw')
title('True Rotational Dynamics (deg)')
ylim([-20 20])

%Plot rot rate dynamics
figure
plot((1:n-1)*qsim.dt, state(10:12,1:n-1)*180/pi())
xlabel('Time')
ylabel('Rotational Rate Output')
legend('wR','wP','wYaw')
title('True Rotational Rate Dynamics (deg/s)')

%Plot accel rot dynamics
figure
plot((1:n-1)*qsim.dt, measurement_buffer.accel_rot(:,1:n-1)*180/pi())
xlabel('Time')
ylabel('Acceleration Rotational Output')
legend('aR','aP','aYaw')
title('Rotation based on Accel (deg/s)')
ylim([-20 20])

%Plot controls
figure
plot((1:n-1)*qsim.dt, u(:,1:n-1))
xlabel('Time')
ylabel('Motor Force (kg*mm)/(s^2)')
legend('U1','U2','U3','U4')
title('Control Inputs')

%Plot Motor Outputs
figure
plot((1:n-1)*qsim.dt, motor_cmd(:,1:n-1))
xlabel('Time')
ylabel('Motor Command (uS)')
legend('M1','M2','M3','M4')
title('Motor Commands Inputs')

%Plot Trajectory of QR
figure
plot3(state(1,1:n-1),state(2,1:n-1),state(3,1:n-1),'-',state(1,1),state(2,1),state(3,1),'r*',state(1,n-1),state(2,n-1),state(3,n-1),'g*', des(1,1:n-1),des(2,1:n-1),des(3,1:n-1),'-r', des(1,n-1),des(2,n-1),des(3,n-1),'ro')
legend('QR Flight Path','Initial','Final','Desired Trajectory','End-point')
xlabel('X Position (mm)')
ylabel('Y Position (mm)')
zlabel('Z Position (mm)')
title('[True] QR Position (mm)')
xlim([-2500 2500])
ylim([-2500 2500])
zlim([0 2500])
grid on

end

