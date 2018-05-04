function quad = init_quad( )
% RAINDrop Quadrotor Simulator: Quadrotor Init File
%
% Thomas Miesen -- RAIN Lab

quad = struct;

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


end

