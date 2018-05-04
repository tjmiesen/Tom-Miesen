function noise = init_noise( quad,qsim )
% RAINDrop Quadrotor Simulator: Noise Init File
%
% Initialize state noise characteristics (vibration, disturbances, etc)
%
% Thomas Miesen -- RAIN Lab

% Assume no noise directly affecting positon
noise.pos = 0;  
% Assume no noise directly affecting angle
noise.rot = 0;  
% Assume no noise directly affecting velocity
noise.vel = 0;
% Small vibrational noise affecting attitude rates
noise.ang_vel = 0.0005; 

% Initialize sensor noise characteristics (electical or measurement noise)
noise.pos_measurement = 0.25;      % GPS noise: +/-0.25mm
% No angle noise, noise is passed through estimation
noise.rot_measurement = 0.1*pi()/180;  
% Velocity is not measured, noise is passed through estimation
noise.vel_measurement = 0;   
% Angular rate noise (from datasheet): 0.01dps/sqrt(hz), checked at 142Hz
noise.ang_vel_measurement = 0.01/sqrt(1/quad.rot_sample_rate);    
% Noise in mm/s^2 (estimating to be ~ 0.1*g)
noise.accel_measurement = 0.1*qsim.g;

end

