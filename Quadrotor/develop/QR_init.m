%% RAINDrop Quadrotor Simulator: Initilization (runs at start of main)
%Thomas Miesen

%Clear variables and close all plots
clear all
close all

%Add structure formats once non-linear dynamics are added.
quad   = struct;
measurement_buffer   = struct;
qsim   = struct;
quad_lqr = struct;
quad_pid = struct;
noise  = struct;

%% Quad
%Quadrotor physical definitions
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

%Update rates
quad.controller_update_rate = 0.007; %Sample rate of microcontroller in s (~7ms)
quad.pos_sample_rate = 0.01; %sample rate of position/vel sensor
quad.rot_sample_rate = 0.007; %sample rate of rotational/rate sensor

%Trajectory definitions
quad.desired_position = [1000; 1000; 1500]; %Sets a desired position for the trajectory planning algorithm.
quad.previous_desired_position = [0;0;0];
quad.max_velocity = 500; %velocity on each axis: 500mm/s appears to be good

%Control limiters
quad.p_cont_max = 1.0; %1-100, max position control output (z excluded)
quad.r_cont_max = 2; %1-100, max attitude control output (yaw excluded)

%Max linearized motor commands allowed
quad.mmax = 2000;
quad.mmin = 1000;

%Estimator Gains
quad.k_common_gain = 0.05;  %0.1-0.75 (works all the way up to 1.0, however that would ignore the gyro entirely for r and p)
quad.k_lowpass_accel = 0.02; %0.02-0.075
quad.k_lowpass_ang = 0.2; %0.15-0.2



%% Qsim
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



%% Measurement
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



%% Noise
%Initialize state noise characteristics (vibration, disturbances, etc)
noise.pos = 0;  %Assume no noise directly affecting positon
noise.rot = 0;  %Assume no noise directly affecting angle
noise.vel = 0;
noise.ang_vel = 0.0005; %Small vibrational noise affecting attitude rates

%Initialize sensor noise characteristics (electical or measurement noise)
noise.pos_measurement = 0.25;      %GPS noise: +/-0.25mm
noise.rot_measurement = 0.1*pi()/180;  %No angle noise, noise is passed through estimation
noise.vel_measurement = 0;              %Velocity is not measured, noise is passed through estimation
noise.ang_vel_measurement = 0.01/sqrt(1/quad.rot_sample_rate);      %Angular rate noise (from datasheet): 0.01dps/sqrt(hz), checked at 142Hz
noise.accel_measurement = 0.1*qsim.g; %Noise in mm/s^2 (estimating to be ~ 0.1*g)
