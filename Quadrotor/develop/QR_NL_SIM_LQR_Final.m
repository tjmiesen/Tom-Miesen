%% QR Project for AA548
% Reference for QR dynamics: http://dx.doi.org/10.1109/ROBOT.2010.5509452
clear all
close all

%Add structure formats once non-linear dynamics are added.
quad   = struct;
qsim   = struct;
QR_LQR = struct;
noise  = struct;

%Quadrotor definitions
quad.m = 0.11;  %mass of QR in kg
quad.d = 55;    %distance from COM to axis connecting motors in mm 
quad.a = 75;    %distance to each motor in mm
quad.h = 10;     %approximate mass moment of inertia height of QR in mm
quad.w = 55;    %approximate mass moment of inertia width
quad.bladepitch = 20*pi()/180;
quad.C_yaw = tan(quad.bladepitch); %approximate conversion of thrust per motor to torsion on *the z axis.
quad.Ix = quad.m*(quad.h^2 + quad.w^2)/12;
quad.Iy = quad.m*(quad.h^2 + quad.w^2)/12;
quad.Iz = quad.m*(quad.w^2 + quad.w^2)/12;
quad.controller_update_rate = 0.007; %Sample rate of microcontroller in s (~7ms)
quad.pos_sample_rate = 0.01; %sample rate of position/vel sensor
quad.rot_sample_rate = 0.007; %sample rate of rotational/rate sensor
quad.k1 = 0.01;    %translational damping terms (drag forces)
quad.k2 = 0.01;
quad.k3 = 0.01;
quad.k4 = 0.012;   %rotational damping terms
quad.k5 = 0.012;
quad.k6 = 0.012;


quad.desired_position = [1000; 1000; 1500]; %Sets a desired position for the trajectory planning algorithm.
quad.previous_desired_position = [0;0;0];
quad.max_velocity = 500; %velocity on each axis: 500mm/s appears to be good

%Simulation Parameters
qsim.g = 9810;  %mm/s^2
qsim.tmax = 10;
qsim.dt = 0.001; %simulation dt in s (NOTE: 10ms is ONLY for to speed up the SETUP process)
qsim.N = qsim.tmax/qsim.dt;
qsim.t_loop = 0;
qsim.t_loop_pos = 0;
qsim.t_loop_rot = 0;

dt = qsim.dt;
tmax = qsim.tmax;
N = qsim.N;

%Variable Init
temp_control = 0;
temp_motor = 1000;
%temp_pos = [0;0;0];
temp_pos = zeros(3,6);
temp_vel = [0;0;0];
temp_vel1 = [0;0;0];
temp_vel2 = zeros(3,6);
vel_est_diff = zeros(3,qsim.N);
temp_rot = [0;0;0];
%temp_ang = [0;0;0];
temp_ang = zeros(3,6);
temp_accel = [0;0;0];
temp_accel_rot = [0;0;0];
acc = zeros(3,6);


%temp_control = 0;
u =     zeros(4, qsim.N); %Postion control inputs
q =     zeros(12, qsim.N); %state
y =     zeros(9, qsim.N); %Output
des =   zeros(12, qsim.N);
mot =   zeros(4, qsim.N); %Motor Command Values
accel_rot = zeros(3,qsim.N);

%Initial Position and rotation
q(:,1) = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]; %LQR Position Controller
u(:,1) = [quad.m*qsim.g/4; quad.m*qsim.g/4; quad.m*qsim.g/4; quad.m*qsim.g/4];

%% Linearized controller with the variable Yaw elements left for LQR recalculation
I = eye(6);
G = @(q,u) [0 0 0 (sum(u(:,1),1)/quad.m)*sin(q(6,1)) (sum(u(:,1),1)/quad.m)*cos(q(6,1)) 0;
            0 0 0 (sum(u(:,1),1)/quad.m)*cos(q(6,1)) -(sum(u(:,1),1)/quad.m)*sin(q(6,1)) 0;
            0 0 0 0 0 0;
            0 0 0 0 0 0;
            0 0 0 0 0 0;
            0 0 0 0 0 0];
A = @(q,u) [I*0 I; G(q,u) I*0];


B = @(q)    [0 0 0 0; %x                 1
             0 0 0 0; %y                 2
             0 0 0 0; %z                 3
             0 0 0 0; %r                 4
             0 0 0 0; %p                 5
             0 0 0 0; %ya                6
             0 0 0 0; %Vx                7
             0 0 0 0; %Vy                8
             1/(quad.m) 1/(quad.m) 1/(quad.m) 1/(quad.m);       %Vz  9
            -quad.d/(quad.Ix)  quad.d/(quad.Ix) -quad.d/(quad.Ix) quad.d/(quad.Ix); %Vr  10
            -quad.d/(quad.Iy) -quad.d/(quad.Iy)  quad.d/(quad.Iy) quad.d/(quad.Iy); %Vp  11
             quad.a/quad.Iz   -quad.a/quad.Iz   -quad.a/quad.Iz   quad.a/quad.Iz];         %Vya 12

%Output matrix, since we only can acquire Position attitude and angle
%Should be a 9x12
C = [1 0 0 0 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 0 0 0 0;
     0 0 0 1 0 0 0 0 0 0 0 0;
     0 0 0 0 1 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 0 0 0 0 1];

%State Noise Matrix Definitions
noise.pos = 0;    %Assume no noise directly affecting positon
noise.rot = 0;  %Assume no noise directly affecting angle
noise.vel = 0;
noise.ang_vel = 0.0005; %Small vibrational noise affecting attitude rates
noise.accel = 0.1*qsim.g; %Noise in mm/s^2 (estimating to be ~ 0.1*g)

F = [eye(3)*noise.pos, zeros(3,9);
     zeros(3,3), eye(3)*noise.rot, zeros(3,6);
     zeros(3,6), eye(3)*noise.vel, zeros(3,3);
     zeros(3,9), eye(3)*noise.ang_vel];

%Measurement Noise Matrix Definitions
noise.pos_measurement = 0.25;      %GPS noise: +/-0.25mm
noise.rot_measurement = 0.1*pi()/180;  %Angle noise: +/- 0.1 deg
noise.vel_measurement = 0;              %Velocity is not measured
noise.ang_vel_measurement = 0.01/sqrt(1/quad.rot_sample_rate);      %Angular rate noise (from datasheet): 0.01dps/sqrt(hz), checked at 142Hz

H = [eye(3)*noise.pos_measurement, zeros(3,9);
     zeros(3,3), eye(3)*noise.rot_measurement, zeros(3,6);
     zeros(3,9), eye(3)*noise.ang_vel_measurement];
 
%% Optimal LQR Controller
%Based off the cost function int[0-inf](||d(t)||^2 + ||u(t)||^2)dt
%k_lqr is found such that u* = k_lqr*x(t)

%Initialize P = Qf, which is 1.
%QR_LQR.Q = eye(12)*0.00035;         %Sets general
%QR_LQR.Q(3,3) = 20;                 %z position
%QR_LQR.Q(7:8,7:8) = eye(2)*0.0005;  %xy velocity
%QR_LQR.Q(9,9) = 10;                 %z velocity
%QR_LQR.Q(10:11,10:11) = eye(2)*10;  %roll,pitch rates
%QR_LQR.Q(12,12) = 10;               %yaw rate

% %Corrected Q matrix for realistic velocity estimation
% QR_LQR.Q = eye(12)*0.001;           %Sets general
% QR_LQR.Q(3,3) = 50;                 %z position
% QR_LQR.Q(4:5,4:5) = eye(2)*10;      %roll,pitch
% QR_LQR.Q(6,6) = 20;                 %yaw
% QR_LQR.Q(7:8,7:8) = eye(2)*0.00001;    %xy velocity
% QR_LQR.Q(9,9) = 0.00001;                 %z velocity
% QR_LQR.Q(10:11,10:11) = eye(2)*100;  %roll,pitch rates
% QR_LQR.Q(12,12) = 50;               %yaw rate
%QR_LQR.R = eye(4)*10;     

%Corrected Q matrix for realistic velocity estimation and angle estimation
QR_LQR.Q = eye(12)*0.005;           %Sets general
QR_LQR.Q(3,3) = 10;                 %z position
QR_LQR.Q(4:5,4:5) = eye(2)*50;      %roll,pitch
QR_LQR.Q(6,6) = 20;                 %yaw
QR_LQR.Q(7:8,7:8) = eye(2)*0.00001;    %xy velocity
QR_LQR.Q(9,9) = 0.00001;                 %z velocity
QR_LQR.Q(10:11,10:11) = eye(2)*5000;  %roll,pitch rates
QR_LQR.Q(12,12) = 100;               %yaw rate


QR_LQR.P = QR_LQR.Q;
QR_LQR.R = eye(4)*10;           

%Import LQR values (note: discrete time ricatti is in the loop)
Q = QR_LQR.Q;
P = QR_LQR.P;
R = QR_LQR.R;

%Descrete time LQR function for Continuous Plant Dyanamics
K_lqr = lqrd(A(q(:,1),u(:,1)),B(q(:,1)),Q,R,quad.pos_sample_rate);
K = K_lqr;

%Trajectory Parameters
omega = 0.075;
r = 0.5;

%Max linearized motor commands allowed
mmax = 2000;
mmin = 1000;

%% System Response
for t = 1:qsim.N-1
%% Sensor data import
%Collect state measurement (note rotation is measurable from vicon system)
%y(:,t) = C*q(:,t);
y(:,t) = C*q(:,t)+H*randn(12,1);

%Update pos and Rot values at set rates
if (qsim.t_loop_pos > quad.pos_sample_rate)
    % IMPORT VICON POSITON DATA
    %Assemble historical position buffer
    temp_pos(:,6) = temp_pos(:,5);
    temp_pos(:,5) = temp_pos(:,4);
    temp_pos(:,4) = temp_pos(:,3);
    temp_pos(:,3) = temp_pos(:,2);
    temp_pos(:,2) = temp_pos(:,1);
    temp_pos(:,1) = round(y(1:3,t)); %Rounding to emulate integer capping on QR
    
    % VELOCITY ESTIMATION (AVERAGE OF FINITE DIFFERENCE APPROX)
    %Dividing by 10 to get into range of working vel estimation
    temp_vel2(:,1) = 1/5*(5*temp_pos(:,1)-temp_pos(:,2)-temp_pos(:,3)-temp_pos(:,4)-temp_pos(:,5)-temp_pos(:,6))/(quad.pos_sample_rate*10);
    temp_vel = round(temp_vel2(:,1)); %Emulate integer rounding error for vel
    
    % YAW MEASUREMENT FROM VICON (simply replaces current yaw value onboard)
    temp_rot(3,1) = y(6,t);
end
if qsim.t_loop_rot > quad.rot_sample_rate  
    % ANGULAR RATE IMPORT
    %Filtering only done on r and p values
    k_lowpass_ang = 0.2; %0.15-0.2
    temp_ang(1:2,1) = k_lowpass_ang*y(7:8,t)+(1-k_lowpass_ang)*temp_ang(1:2,2);
    temp_ang(3,1) = y(9,t);
    %Store old rate
    temp_ang(1:2,2) = temp_ang(1:2,1);
    
    % ANGLE BASED OFF ANGULAR RATE
    %Initial angle estimation from angular rate
    temp_rot(:,1) = temp_rot(:,1) + quad.rot_sample_rate*(temp_ang(:,1));
    
    % ACCEL IMPORT
    %Accelerometer data import (sampled at the same time as gyro data)
    %Add noise to accel data
    acc(:,1) = acc(:,1) + noise.accel*randn();
    %Low-pass filter on accelerometer data
    k_lowpass_accel = 0.02; %0.02-0.075
    temp_accel = k_lowpass_accel*acc(:,1) + (1-k_lowpass_accel)*acc(:,2);
    %Store old accel data
    acc(:,2) = temp_accel;
    
    % ACCEL ANGLE
    %Angle determination from accelerometer data (note we're using the true yaw angle because the acceleration measurement is with respect to the body frame)
    temp_accel_rot(1,1) = atan((cos(q(6,t))*temp_accel(2,1) + sin(q(6,t))*temp_accel(1,1))/((temp_accel(3,1) + qsim.g)));
    temp_accel_rot(2,1) = atan((cos(q(6,t))*temp_accel(1,1) - sin(q(6,t))*temp_accel(2,1))/(sqrt((cos(q(6,t))*temp_accel(2,1) + sin(q(6,t))*temp_accel(1,1))^2 + (temp_accel(3,1)+qsim.g)^2)));
    
    % FINAL ANGLE ESTIMATION (COMMON FILTER)
    %Common filter (for roll and pitch only)
    k_com = 0.1; %0.1-0.75 (works all the way up to 1.0, however that would ignore the gyro entirely for r and p)
    temp_rot(1:2,1) = (1-k_com)*temp_rot(1:2,1) + k_com*temp_accel_rot(1:2,1);
end
accel_rot(:,t) = temp_accel_rot;


%% Trajectory Generation
%Cork-screw
%des(1:3,t) = [r*0.5*dt*t*cos(dt*t*omega);r*0.5*dt*t*sin(dt*t*omega);0.15*dt*t]*1000;

%Test setting a desired yaw
des(6,t) = 10*dt*t*pi()/180;

%Desired location, with simply linear trajectory planning
%This updates every time a new position value has been recieved.
if qsim.t_loop_pos > quad.pos_sample_rate
    %Trajectory generation
    %des(1:3,t) = QR_LinearTrajectory(temp_pos, quad.desired_position, quad.previous_desired_position, quad.max_velocity, quad.controller_update_rate);
    
    %C++ similar trajectory generator
    des(1:3,t) = QR_LinearTrajectory_Ccode_similar(temp_pos, quad.desired_position, quad.previous_desired_position, quad.max_velocity, quad.pos_sample_rate);
    
    quad.previous_desired_position = des(1:3,t);
else
    if (t>1)
        des(1:3,t) = quad.previous_desired_position;
    end
end

%Switch desired locations mid-flight to test response    
if t > N/4 && t <= N/2
  quad.desired_position = [1000; -1000; 1500];  
elseif t > N/2 && t <= 3*N/4
  quad.desired_position = [-1000; -1000; 1500];  
elseif t > 3*N/4
 quad.desired_position = [1000; 1000; 1500];
end

% %Ends flight path preemptively and QR is supposed to stop at point
% if t > 6/dt
%   des(1:3,t) = des(1:3,t-1); 
%   quad.previous_desired_position = des(1:3,t-1);
% end

%% Test momentary GPS communication loss (0.25s) 
% if (t > 5.0/dt && t < 5.25/dt)
%     temp_pos = loss_temp_pos;   %Position is now the last known
%     temp_vel = loss_temp_vel;   %Velocity is now the last known
%     des(1:3,t) = l_des;         %Trajectory path is now last known
%     
%                                 %(Setting to lost position value will cause
%                                 %a controller reaction which is expecting a
%                                 %position and velocity update, so leaving
%                                 %the desired value the same allows the
%                                 %controller to maintain a more stable
%                                 %response for the drop-out period)
% else
%     loss_temp_pos = temp_pos;
%     loss_temp_vel = temp_vel;
%     l_des = des(1:3,t);
% end

%% Control Inputs
% %APPLY LQR CONTROL
%Re-orient the optimal LQR gain to the QR measured yaw angle
K = [ K_lqr(1,1)*(cos(temp_rot(3,1))+sin(temp_rot(3,1))), K_lqr(1,2)*(cos(temp_rot(3,1))-sin(temp_rot(3,1)))     ,K_lqr(1,3:6),      K_lqr(1,7)*(cos(temp_rot(3,1))+sin(temp_rot(3,1))), K_lqr(1,8)*(cos(temp_rot(3,1))-sin(temp_rot(3,1))),        K_lqr(1,9:12);
      K_lqr(2,1)*(cos(temp_rot(3,1))-sin(temp_rot(3,1))), K_lqr(2,2)*(cos(temp_rot(3,1))+sin(temp_rot(3,1)))     ,K_lqr(2,3:6),      K_lqr(2,7)*(cos(temp_rot(3,1))-sin(temp_rot(3,1))), K_lqr(2,8)*(cos(temp_rot(3,1))+sin(temp_rot(3,1))),        K_lqr(2,9:12);
      K_lqr(3,1)*(cos(temp_rot(3,1))-sin(temp_rot(3,1))), K_lqr(3,2)*(cos(temp_rot(3,1))+sin(temp_rot(3,1)))     ,K_lqr(3,3:6),      K_lqr(3,7)*(cos(temp_rot(3,1))-sin(temp_rot(3,1))), K_lqr(3,8)*(cos(temp_rot(3,1))+sin(temp_rot(3,1))),        K_lqr(3,9:12);
      K_lqr(4,1)*(cos(temp_rot(3,1))+sin(temp_rot(3,1))), K_lqr(4,2)*(cos(temp_rot(3,1))-sin(temp_rot(3,1)))     ,K_lqr(4,3:6),      K_lqr(4,7)*(cos(temp_rot(3,1))+sin(temp_rot(3,1))), K_lqr(4,8)*(cos(temp_rot(3,1))-sin(temp_rot(3,1))),        K_lqr(4,9:12)];

%Control value based off measured values (which are each dependent on their
%own sample rates.
u(:,t) = -K*([temp_pos(:,1); temp_rot(:,1); temp_vel(:,1); temp_ang(:,1)] - des(:,t));
%u(:,t) = -K*(q(:,t)-des(:,t));

%Bound the control values to actual control range (1000us-2000us) and
%convert back to force to account for rounding errors.

%(Uniform) Motor Pulse Noise from Interrupts (+0-14us)
motor_pulse_noise = round(10+15*randn(1,1));

for i = 1:4
   %Shift each control thrust input to hover thrust range (mg/4 per motor)
   u(i,t) = u(i,t) + quad.m*qsim.g/4;
   
   %Convert motor speeds to applied thrusts
   %(note: this is done for LQR as well to account for rounding error in the
   %motor speed value)
   %(note: the conversion is actually for the force measured from all four
   %motors running at that uS pulse. So the individual forces are
   %multiplied by 4, then converted to N)
   mot(i,t) = -193.51*(u(i,t)*4/1000)^2 + 640.42*(u(i,t)*4/1000) + 1011.5;
   
   %Round to simulate integer conversion on QR
   %mot(i,t) = round(mot(i,t));
   mot(i,t) = round(10*mot(i,t))/10;    %Simulate single floating point
   
   %Since the max operating range of the command pulse is 1000-1900, the
   %linearized QR dynamics has the command pulse set to ~460. So we shift
   %down the max and min values to account for this.
   if mot(i,t) > mmax
       mot(i,t) = mmax;
   elseif mot(i,t) < mmin
       mot(i,t) = mmin;
   end
   
   %Simulate uS pulse legnth noise from interrupt subroutines
   %Currently modeled as simple variable offset
   mot(i,t) = mot(i,t) + motor_pulse_noise;
   %mot(i,t) = mot(i,t) + motor_noise + sound;
   
   %Convert motor command back to a force (kg*mm/s^2
   u(i,t) = 1000*((2.3159e-6)*mot(i,t)^2 - (3.5209e-3)*mot(i,t) + 1.2079)/4;
end

%DESCRETE SAMPLING CONTROL APPLICATION
%Applies controls at the update rate of the physical control loop.
%Note that this is independent of the sampling rates of the sensors.
if qsim.t_loop >= quad.controller_update_rate
    temp_control = u(:,t);
    temp_motor = mot(:,t);
    qsim.t_loop = 0;
else
    u(:,t) = temp_control;
    mot(:,t) = temp_motor;
end

%% Reset timers
if(qsim.t_loop_pos > quad.pos_sample_rate)
    qsim.t_loop_pos = 0;
end
if(qsim.t_loop_rot > quad.rot_sample_rate)
    qsim.t_loop_rot = 0;
end
if(qsim.t_loop > quad.controller_update_rate)
    qsim.t_loop = 0;
end

%% Progress timers
qsim.t_loop_pos = qsim.t_loop_pos + qsim.dt;
qsim.t_loop_rot = qsim.t_loop_rot + qsim.dt;
qsim.t_loop = qsim.t_loop + qsim.dt;


%% Nonlinear Dynamics
%Non-linear Dynamics (Noise and Noise-less Available)
%Note these are note discrete as the real system is continuous by nature.
%q(:,t+1) = QR_VariableYaw_NL_Dyn((q(:,t)),u(:,t),quad,qsim);
[q(:,t+1),acc(:,1)] = QR_VariableYaw_NL_Dyn((q(:,t)+F*randn(12,1)),u(:,t),quad,qsim);

%Simulates QR on ground (if is at 0 height, then it can't translate in xy)
if (q(3,t+1) <= 0)
    q(1:2,t+1) = q(1:2,t);
    q(3,t+1) = 0;
end
end


%Plot state dynamics
figure
[m,n] = size(q);
plot((1:n-1)*dt, y(1:3,1:n-1))
xlabel('Time')
ylabel('Position Output')
legend('X','Y','Z')
title('Position Dynamics (mm)')

%Plot rot dynamics
figure
plot((1:n-1)*dt, y(4:6,1:n-1)*180/pi())
xlabel('Time')
ylabel('Rotation Output')
legend('R','P','Yaw')
title('Rotational Dynamics (deg)')
ylim([-180 180])

%Plot rot rate dynamics
figure
plot((1:n-1)*dt, y(7:9,1:n-1)*180/pi())
xlabel('Time')
ylabel('Rotational Rate Output')
legend('wR','wP','wYaw')
title('Rotational Rate Dynamics (deg/s)')

%Plot accel rot dynamics
figure
plot((1:n-1)*dt, accel_rot(:,1:n-1)*180/pi())
xlabel('Time')
ylabel('Acceleration Rotational Output')
legend('aR','aP','aYaw')
title('Rotation based on Accel (deg/s)')
ylim([-180 180])

%Plot controls
figure
plot((1:n-1)*dt, u(:,1:n-1))
xlabel('Time')
ylabel('Motor Force (kg*mm)/(s^2)')
legend('U1','U2','U3','U4')
title('Control Inputs')

%Plot Motor Outputs
figure
plot((1:n-1)*dt, mot(:,1:n-1))
xlabel('Time')
ylabel('Motor Command (uS)')
legend('M1','M2','M3','M4')
title('Motor Commands Inputs')

%Plot Trajectory of QR
figure
plot3(y(1,1:n-1),y(2,1:n-1),y(3,1:n-1),'-',y(1,1),y(2,1),y(3,1),'r*',y(1,n-1),y(2,n-1),y(3,n-1),'g*', des(1,1:n-1),des(2,1:n-1),des(3,1:n-1),'-r', des(1,n-1),des(2,n-1),des(3,n-1),'ro')
legend('QR Flight Path','Initial','Final','Desired Trajectory','End-point')
xlabel('X Position (mm)')
ylabel('Y Position (mm)')
zlabel('Z Position (mm)')
title('QR Position (mm)')
xlim([-2500 2500])
ylim([-2500 2500])
zlim([0 2500])
grid on