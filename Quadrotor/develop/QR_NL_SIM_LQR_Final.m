%% QR Project for AA548
% Reference for QR dynamics: http://dx.doi.org/10.1109/ROBOT.2010.5509452
clear all
close all

%Add structure formats once non-linear dynamics are added.
quad   = struct;
measurement_buffer   = struct;
qsim   = struct;
quad_lqr = struct;
quad_pid = struct;
noise  = struct;

%Quadrotor definitions
quad.m = 0.11;  %mass of QR in kg
quad.d = 55;    %distance from COM to axis connecting motors in mm 
quad.a = 75;    %distance to each motor in mm
quad.h = 10;     %approximate mass moment of inertia height of QR in mm
quad.w = 55;    %approximate mass moment of inertia width
quad.ecc_dist = 2.5;      %norm distance to eccentric load 2mm below COM
quad.ecc_load = 0.25*quad.m;   %approximation of eccentric load ~25% of mass
quad.bladepitch = 20*pi()/180;
quad.C_yaw = tan(quad.bladepitch); %approximate conversion of thrust per motor to torsion on *the z axis.
quad.Ix = quad.m*(quad.h^2 + quad.w^2)/12;
quad.Iy = quad.m*(quad.h^2 + quad.w^2)/12;
quad.Iz = quad.m*(quad.w^2 + quad.w^2)/12;
quad.controller_update_rate = 0.007; %Sample rate of microcontroller in s (~7ms)
quad.pos_sample_rate = 0.01; %sample rate of position/vel sensor
quad.rot_sample_rate = 0.007; %sample rate of rotational/rate sensor
quad.desired_position = [1000; 1000; 1500]; %Sets a desired position for the trajectory planning algorithm.
quad.previous_desired_position = [0;0;0];
quad.max_velocity = 500; %velocity on each axis: 500mm/s appears to be good
quad.p_cont_max = 1.0; %1-100
quad.r_cont_max = 2; %1-100

%Simulation Parameters
qsim.g = 9810;  %mm/s^2
qsim.tmax = 10;
qsim.dt = 0.001; %simulation dt in s (NOTE: 10ms is ONLY for to speed up the SETUP process)
qsim.N = qsim.tmax/qsim.dt;
qsim.k1 = 0.01;    %translational damping terms (drag forces) (Ns/m) = kg/s 
qsim.k2 = 0.01;
qsim.k3 = 0.01;
qsim.k4 = 0.04;   %rotational damping terms
qsim.k5 = 0.04;
qsim.k6 = 0.04;
qsim.t_loop = 0;
qsim.t_loop_pos = 0;
qsim.t_loop_rot = 0;

dt = qsim.dt;
tmax = qsim.tmax;
N = qsim.N;

%Initialize the measurement buffer
measurement_buffer.control = 0;
measurement_buffer.motor = 1000;
measurement_buffer.pos = zeros(3,6);
measurement_buffer.vel = [0;0;0];
measurement_buffer.vel1 = [0;0;0];
measurement_buffer.vel2 = zeros(3,6);
measurement_buffer.rot = [0;0;0];
measurement_buffer.ang = zeros(3,6);
measurement_buffer.accel = [0;0;0];
measurement_buffer.accel_rot = zeros(3,qsim.N);

%Initialize other simulation-specific variables
u =     zeros(4, qsim.N); %Postion control inputs
q =     zeros(12, qsim.N); %state
y =     zeros(7, qsim.N); %Output
des =   zeros(12, qsim.N);
mot =   zeros(4, qsim.N); %Motor Command Values
PID =   zeros(4,1);
acc =   zeros(3,6);
q(:,1) = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]; %full state
u(:,1) = [quad.m*qsim.g/4; quad.m*qsim.g/4; quad.m*qsim.g/4; quad.m*qsim.g/4];

%% Linearized Rotational Controller
I_r = eye(3);
A_r = [I_r*0 I_r; I_r*0 I_r*0];
B_r =  [0 0 0 0; %r                 1
        0 0 0 0; %p                 2
        0 0 0 0; %ya                3
        quad.d/(quad.Ix) -quad.d/(quad.Ix) quad.d/(quad.Ix) -quad.d/(quad.Ix); %Vr  4
       -quad.d/(quad.Iy) -quad.d/(quad.Iy) quad.d/(quad.Iy)  quad.d/(quad.Iy); %Vp  5
       -quad.a/quad.Iz    quad.a/quad.Iz   quad.a/quad.Iz   -quad.a/quad.Iz];  %Vya 6

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
             quad.d/(quad.Ix) -quad.d/(quad.Ix) quad.d/(quad.Ix) -quad.d/(quad.Ix); %Vr  10
            -quad.d/(quad.Iy) -quad.d/(quad.Iy) quad.d/(quad.Iy)  quad.d/(quad.Iy); %Vp  11
            -quad.a/quad.Iz    quad.a/quad.Iz   quad.a/quad.Iz   -quad.a/quad.Iz];         %Vya 12

%Output matrix, since we only can acquire position, rate, accel, and yaw
%Should be a 7x12
% C = [1 0 0 0 0 0 0 0 0 0 0 0;
%      0 1 0 0 0 0 0 0 0 0 0 0;
%      0 0 1 0 0 0 0 0 0 0 0 0;
%      0 0 0 1 0 0 0 0 0 0 0 0;
%      0 0 0 0 1 0 0 0 0 0 0 0;
%      0 0 0 0 0 1 0 0 0 0 0 0;
%      0 0 0 0 0 0 0 0 0 1 0 0;
%      0 0 0 0 0 0 0 0 0 0 1 0;
%      0 0 0 0 0 0 0 0 0 0 0 1];

C = [1 0 0 0 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 0 0 0 0;
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
% 
% H = [eye(3)*noise.pos_measurement, zeros(3,9);
%      zeros(3,3), eye(3)*noise.rot_measurement, zeros(3,6);
%      zeros(3,9), eye(3)*noise.ang_vel_measurement];
%  
H = [eye(3)*noise.pos_measurement, zeros(3,9);
     zeros(1,5), noise.rot_measurement, zeros(1,6);
     zeros(3,9), eye(3)*noise.ang_vel_measurement];
%% Optimal LQR Controller
%Based off the cost function int[0-inf](||d(t)||^2 + ||u(t)||^2)dt
%k_lqr is found such that u* = k_lqr*x(t)

%Corrected Q matrix for realistic velocity estimation and angle estimation
quad_lqr.Q = eye(12)*0.00001;           %Sets general
quad_lqr.Q(3,3) = 1;                 %z position
quad_lqr.Q(4:5,4:5) = eye(2)*100;      %roll,pitch
quad_lqr.Q(6,6) = 1000;                 %yaw
quad_lqr.Q(7:8,7:8) = eye(2)*0.000001;    %xy velocity
quad_lqr.Q(9,9) = 0.000001;                 %z velocity
quad_lqr.Q(10:11,10:11) = eye(2)*250;  %roll,pitch rates
quad_lqr.Q(12,12) = 50;               %yaw rate


quad_lqr.P = quad_lqr.Q;
quad_lqr.R = eye(4)*1;           

%Import LQR values (note: discrete time ricatti is in the loop)
Q = quad_lqr.Q;
P = quad_lqr.P;
R = quad_lqr.R;

%Descrete time LQR function for Continuous Plant Dyanamics

% Position LQR
K_lqr = lqrd(A(q(:,1),u(:,1)),B(q(:,1)),Q,R,quad.pos_sample_rate);
K = K_lqr;

% Rotational LQR
Q_r = eye(6);
Q_r(1:2,1:2) = 100*eye(2);
Q_r(3,3) = 1000;
Q_r(4:5,4:5) = 250*eye(2);
Q_r(6,6) = 50;
R_r = 1*eye(4);
quad_lqr.K_lqr_rot = lqrd(A_r,B_r,Q_r,R_r,quad.pos_sample_rate);


%% PD Definitions
% Rotational PD controller Parameters
%Current Parameters on QR

quad_pid.kp = 0.05;
quad_pid.kd = 0.0375;
quad_pid.ki = 0.025;

quad_pid.kp_yaw = 1.5;
quad_pid.kd_yaw = 0.2;
quad_pid.ki_yaw = 0.00025;

err_sum = [0;0;0];

K_p = [ quad_pid.kp -quad_pid.kp -quad_pid.kp_yaw;
       -quad_pid.kp -quad_pid.kp  quad_pid.kp_yaw;
        quad_pid.kp  quad_pid.kp  quad_pid.kp_yaw;
       -quad_pid.kp  quad_pid.kp -quad_pid.kp_yaw];

K_d = [ quad_pid.kd -quad_pid.kd -quad_pid.kd_yaw;
       -quad_pid.kd -quad_pid.kd  quad_pid.kd_yaw;
        quad_pid.kd  quad_pid.kd  quad_pid.kd_yaw;
       -quad_pid.kd  quad_pid.kd -quad_pid.kd_yaw];

K_i = [ quad_pid.ki -quad_pid.ki -quad_pid.ki_yaw;
       -quad_pid.ki -quad_pid.ki  quad_pid.ki_yaw;
        quad_pid.ki  quad_pid.ki  quad_pid.ki_yaw;
       -quad_pid.ki  quad_pid.ki -quad_pid.ki_yaw];

%% Other Definitions
%Trajectory Parameters
omega = 0.075;
r = 0.5;

%Max linearized motor commands allowed
quad.mmax = 2000;
quad.mmin = 1000;

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
    measurement_buffer.pos(:,6) = measurement_buffer.pos(:,5);
    measurement_buffer.pos(:,5) = measurement_buffer.pos(:,4);
    measurement_buffer.pos(:,4) = measurement_buffer.pos(:,3);
    measurement_buffer.pos(:,3) = measurement_buffer.pos(:,2);
    measurement_buffer.pos(:,2) = measurement_buffer.pos(:,1);
    measurement_buffer.pos(:,1) = round(y(1:3,t)); %Rounding to emulate integer capping on QR
    
    % VELOCITY ESTIMATION (AVERAGE OF FINITE DIFFERENCE APPROX)
    %Dividing by 10 to get into range of working vel estimation
    measurement_buffer.vel2(:,1) = 1/5*(5*measurement_buffer.pos(:,1)-measurement_buffer.pos(:,2)-measurement_buffer.pos(:,3)-measurement_buffer.pos(:,4)-measurement_buffer.pos(:,5)-measurement_buffer.pos(:,6))/(quad.pos_sample_rate*10);
    measurement_buffer.vel = round(measurement_buffer.vel2(:,1)); %Emulate integer rounding error for vel
    
    % YAW MEASUREMENT FROM VICON (simply replaces current yaw value onboard)
    measurement_buffer.rot(3,1) = y(4,t);
end
if qsim.t_loop_rot > quad.rot_sample_rate  
    % ANGULAR RATE IMPORT
    %Filtering only done on r and p values
    k_lowpass_ang = 0.2; %0.15-0.2
    measurement_buffer.ang(1:2,1) = k_lowpass_ang*y(5:6,t)+(1-k_lowpass_ang)*measurement_buffer.ang(1:2,2);
    measurement_buffer.ang(3,1) = y(7,t);
    %Store old rate
    measurement_buffer.ang(1:2,2) = measurement_buffer.ang(1:2,1);
    
    % ANGLE BASED OFF ANGULAR RATE
    %Initial angle estimation from angular rate
    measurement_buffer.rot(:,1) = measurement_buffer.rot(:,1) + quad.rot_sample_rate*(measurement_buffer.ang(:,1));
    
    % ACCEL IMPORT
    %Accelerometer data import (sampled at the same time as gyro data)
    %Add noise to accel data
    acc(:,1) = acc(:,1) + noise.accel*randn();
    %Low-pass filter on accelerometer data
    k_lowpass_accel = 0.02; %0.02-0.075
    measurement_buffer.accel = k_lowpass_accel*acc(:,1) + (1-k_lowpass_accel)*acc(:,2);
    %Store old accel data
    acc(:,2) = measurement_buffer.accel;
    
    % ACCEL ANGLE
    %Angle determination from accelerometer data (note we're using the true yaw angle because the acceleration measurement is with respect to the body frame)
    measurement_buffer.accel_rot(1,t) = atan((cos(q(6,t))*measurement_buffer.accel(2,1) + sin(q(6,t))*measurement_buffer.accel(1,1))/((measurement_buffer.accel(3,1) + qsim.g)));
    measurement_buffer.accel_rot(2,t) = atan((cos(q(6,t))*measurement_buffer.accel(1,1) - sin(q(6,t))*measurement_buffer.accel(2,1))/(sqrt((cos(q(6,t))*measurement_buffer.accel(2,1) + sin(q(6,t))*measurement_buffer.accel(1,1))^2 + (measurement_buffer.accel(3,1)+qsim.g)^2)));
    
    % FINAL ANGLE ESTIMATION (COMMON FILTER)
    %Common filter (for roll and pitch only)
    k_com = 0.05; %0.1-0.75 (works all the way up to 1.0, however that would ignore the gyro entirely for r and p)
    measurement_buffer.rot(1:2,1) = (1-k_com)*measurement_buffer.rot(1:2,1) + k_com*measurement_buffer.accel_rot(1:2,t);
end



%% Trajectory Generation
%Cork-screw
%des(1:3,t) = [r*0.5*dt*t*cos(dt*t*omega);r*0.5*dt*t*sin(dt*t*omega);0.15*dt*t]*1000;

%Test setting a desired yaw
des(6,t) = 10*dt*t*pi()/180;

%Desired location, with simply linear trajectory planning
%This updates every time a new position value has been recieved.
if qsim.t_loop_pos > quad.pos_sample_rate
    %Trajectory generation
    %des(1:3,t) = QR_LinearTrajectory(measurement_buffer.pos, quad.desired_position, quad.previous_desired_position, quad.max_velocity, quad.controller_update_rate);
    
    %C++ similar trajectory generator
    des(1:3,t) = QR_LinearTrajectory_Ccode_similar(measurement_buffer.pos, quad.desired_position, quad.previous_desired_position, quad.max_velocity, quad.pos_sample_rate);
    
    quad.previous_desired_position = des(1:3,t);
else
    if (t>1)
        des(1:3,t) = quad.previous_desired_position;
    end
end

%Switch desired locations mid-flight to test response    
if t > N/4 && t <= N/2
  quad.desired_position = [1000; -1000; 1500];  
% elseif t > N/2 && t <= 3*N/4
%   quad.desired_position = [-1000; -1000; 1500];  
% elseif t > 3*N/4
%  quad.desired_position = [1000; 1000; 1500];
end

%Ends flight path preemptively and QR is supposed to stop at point
if t > 6/dt
  des(1:3,t) = des(1:3,t-1); 
  quad.previous_desired_position = des(1:3,t-1);
end

%% Test momentary GPS communication loss (0.25s) 
% if (t > 5.0/dt && t < 5.25/dt)
%     measurement_buffer.pos = loss_temp_pos;   %Position is now the last known
%     measurement_buffer.vel = loss_temp_vel;   %Velocity is now the last known
%     des(1:3,t) = l_des;         %Trajectory path is now last known
%     
%                                 %(Setting to lost position value will cause
%                                 %a controller reaction which is expecting a
%                                 %position and velocity update, so leaving
%                                 %the desired value the same allows the
%                                 %controller to maintain a more stable
%                                 %response for the drop-out period)
% else
%     loss_temp_pos = measurement_buffer.pos;
%     loss_temp_vel = measurement_buffer.vel;
%     l_des = des(1:3,t);
% end

%% Control Inputs
% Adjust Full-State LQR
%Re-orient the optimal LQR gain to the QR measured yaw angle
K = [ K_lqr(1,1)*(cos(measurement_buffer.rot(3,1))-sin(measurement_buffer.rot(3,1))), K_lqr(1,2)*(cos(measurement_buffer.rot(3,1))+sin(measurement_buffer.rot(3,1)))     ,K_lqr(1,3:6),      K_lqr(1,7)*(cos(measurement_buffer.rot(3,1))-sin(measurement_buffer.rot(3,1))), K_lqr(1,8)*(cos(measurement_buffer.rot(3,1))+sin(measurement_buffer.rot(3,1))),        K_lqr(1,9:12);
      K_lqr(2,1)*(cos(measurement_buffer.rot(3,1))+sin(measurement_buffer.rot(3,1))), K_lqr(2,2)*(cos(measurement_buffer.rot(3,1))-sin(measurement_buffer.rot(3,1)))     ,K_lqr(2,3:6),      K_lqr(2,7)*(cos(measurement_buffer.rot(3,1))+sin(measurement_buffer.rot(3,1))), K_lqr(2,8)*(cos(measurement_buffer.rot(3,1))-sin(measurement_buffer.rot(3,1))),        K_lqr(2,9:12);
      K_lqr(3,1)*(cos(measurement_buffer.rot(3,1))+sin(measurement_buffer.rot(3,1))), K_lqr(3,2)*(cos(measurement_buffer.rot(3,1))-sin(measurement_buffer.rot(3,1)))     ,K_lqr(3,3:6),      K_lqr(3,7)*(cos(measurement_buffer.rot(3,1))+sin(measurement_buffer.rot(3,1))), K_lqr(3,8)*(cos(measurement_buffer.rot(3,1))-sin(measurement_buffer.rot(3,1))),        K_lqr(3,9:12);
      K_lqr(4,1)*(cos(measurement_buffer.rot(3,1))-sin(measurement_buffer.rot(3,1))), K_lqr(4,2)*(cos(measurement_buffer.rot(3,1))+sin(measurement_buffer.rot(3,1)))     ,K_lqr(4,3:6),      K_lqr(4,7)*(cos(measurement_buffer.rot(3,1))-sin(measurement_buffer.rot(3,1))), K_lqr(4,8)*(cos(measurement_buffer.rot(3,1))+sin(measurement_buffer.rot(3,1))),        K_lqr(4,9:12)];

% Calculate relevant position or rotational errors
%Position/Velocity error
e = [measurement_buffer.pos(:,1); zeros(3,1); measurement_buffer.vel(:,1); zeros(3,1)] - [des(1:3,t);zeros(3,1);des(7:9,t);zeros(3,1)];

%Rotational/Rate error
e_r = [measurement_buffer.rot(:,1);measurement_buffer.ang(:,1)] - [des(4:6,t);des(10:12,t)];

%Rotational error sum
err_sum = err_sum + qsim.dt*(measurement_buffer.rot(:,1) - des(4:6,t));


%% Full-State LQR (no saturation terms)
%u(:,t) = -K*([measurement_buffer.pos(:,1); measurement_buffer.rot(:,1); measurement_buffer.vel(:,1); measurement_buffer.ang(:,1)] - des(:,t));


%% Full-State LQR (with saturation terms)

%Full State Error
% e_f = [e(1:3,1); e_r(1:3,1); e(4:6,1); e_r(4:6,1)]
% for i = 1:4
%     row_sum = 0;
%     for j = 1:12
%         col_sum = -K(i,j)*(e_f(j,1));
%         %Saturate XY Position Controls
%         if (j == 1 || j == 2 || j == 7 || j == 8)
%             if col_sum > quad.p_cont_max
%                 col_sum = quad.p_cont_max;
%             elseif col_sum < -quad.p_cont_max
%                 col_sum = -quad.p_cont_max;
%             end
%         %Saturate RP Attitude Controls
%         elseif (j == 4 || j == 5 || j == 10 || j == 11)
%             if col_sum > quad.r_cont_max
%                 col_sum = quad.r_cont_max;
%             elseif col_sum < -quad.r_cont_max
%                 col_sum = -quad.r_cont_max;
%             end
%         end
%         row_sum = row_sum + col_sum;
%     end
%     u(i,t) = u(i,t) + row_sum;
% end


%% Position LQR (with saturation terms)
for i = 1:4
    row_sum = 0;
    for j = 1:12
        col_sum = -K(i,j)*(e(j,1));
        %Saturate XY Position Controls
        if (j == 1 || j == 2 || j == 7 || j == 8)
            if col_sum > quad.p_cont_max
                col_sum = quad.p_cont_max;
            elseif col_sum < -quad.p_cont_max
                col_sum = -quad.p_cont_max;
            end
        %Saturate RP Attitude Controls
        elseif (j == 4 || j == 5 || j == 10 || j == 11)
            if col_sum > quad.r_cont_max
                col_sum = quad.r_cont_max;
            elseif col_sum < -quad.r_cont_max
                col_sum = -quad.r_cont_max;
            end
        end
        row_sum = row_sum + col_sum;
    end
    u(i,t) = u(i,t) + row_sum;
end

%% Rotational LQR
%u(:,t) = QR_Rotational_LQR(quad_lqr, quad, e_r, u(:,t));


%% Rotational PID Control
%Calculate Attitude PID control values (note: errors are converted to deg)
PID(:,1) = QR_Rotational_PID(quad_pid, quad, e_r*180/pi(), err_sum*180/pi());


%% Motor Control Applicaiton
%Bound the control values to actual control range (1000us-2000us) and
%convert back to force to account for rounding errors.

%(Uniform) Motor Pulse Noise from Interrupts (+0-14us)
noise.motor_pulse_noise = round(10+15*randn(1,1));

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
   
   %ROT ONLY REMOVE OTHERWISE
   %mot(i,t) = mot(i,t) + 100; %shift up by a small value to allow for actuation

   %Apply PID control
   mot(i,t) = mot(i,t) + PID(i,1);
   
   %Since the max operating range of the command pulse is 1000-1900, the
   %linearized QR dynamics has the command pulse set to ~460. So we shift
   %down the max and min values to account for this.
   if mot(i,t) > quad.mmax
       mot(i,t) = quad.mmax;
   elseif mot(i,t) < quad.mmin
       mot(i,t) = quad.mmin;
   end
   
   %Simulate uS pulse legnth noise from interrupt subroutines
   %Currently modeled as simple variable offset
   mot(i,t) = mot(i,t) + noise.motor_pulse_noise;
   %mot(i,t) = mot(i,t) + motor_noise + sound;
   
   %Convert motor command back to a force (kg*mm/s^2
   u(i,t) = 1000*((2.3159e-6)*mot(i,t)^2 - (3.5209e-3)*mot(i,t) + 1.2079)/4;
end

%DESCRETE SAMPLING CONTROL APPLICATION
%Applies controls at the update rate of the physical control loop.
%Note that this is independent of the sampling rates of the sensors.
if qsim.t_loop >= quad.controller_update_rate
    measurement_buffer.control = u(:,t);
    measurement_buffer.motor = mot(:,t);
    qsim.t_loop = 0;
else
    u(:,t) = measurement_buffer.control;
    mot(:,t) = measurement_buffer.motor;
end


% NEGATE CONTROLS (FOR UNCONTROLLED SYSTEM TEST)
%u(:,t) = zeros(4,1);


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
%[q(:,t+1),acc(:,1)] = QR_VariableYaw_NL_Dyn((q(:,t)),u(:,t),quad,qsim);

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
title('Measured Position (mm)')

%Plot (true) rot dynamics
figure
plot((1:n-1)*dt, q(4:6,1:n-1)*180/pi())
xlabel('Time')
ylabel('Rotation Output')
legend('R','P','Yaw')
title('True Rotational Dynamics (deg)')
ylim([-20 20])

%Plot rot rate dynamics
figure
plot((1:n-1)*dt, q(10:12,1:n-1)*180/pi())
xlabel('Time')
ylabel('Rotational Rate Output')
legend('wR','wP','wYaw')
title('True Rotational Rate Dynamics (deg/s)')

%Plot accel rot dynamics
figure
plot((1:n-1)*dt, measurement_buffer.accel_rot(:,1:n-1)*180/pi())
xlabel('Time')
ylabel('Acceleration Rotational Output')
legend('aR','aP','aYaw')
title('Rotation based on Accel (deg/s)')
ylim([-20 20])

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
plot3(q(1,1:n-1),q(2,1:n-1),q(3,1:n-1),'-',q(1,1),q(2,1),q(3,1),'r*',q(1,n-1),q(2,n-1),q(3,n-1),'g*', des(1,1:n-1),des(2,1:n-1),des(3,1:n-1),'-r', des(1,n-1),des(2,n-1),des(3,n-1),'ro')
legend('QR Flight Path','Initial','Final','Desired Trajectory','End-point')
xlabel('X Position (mm)')
ylabel('Y Position (mm)')
zlabel('Z Position (mm)')
title('[True] QR Position (mm)')
xlim([-2500 2500])
ylim([-2500 2500])
zlim([0 2500])
grid on