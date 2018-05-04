function meas_buffer = init_meas_buffer( qsim )
% RAINDrop Quadrotor Simulator: Measurement Buffer Init File
%
% Thomas Miesen -- RAIN Lab

meas_buffer = struct;

%Initialize the measurement buffer
meas_buffer.control = 0;
meas_buffer.motor = 1000;
meas_buffer.pos = zeros(3,6);
meas_buffer.vel = [0;0;0];
meas_buffer.vel1 = [0;0;0];
meas_buffer.vel2 = zeros(3,6);
meas_buffer.rot = [0;0;0];
meas_buffer.ang = zeros(3,6);
meas_buffer.accel = [0;0;0];
meas_buffer.accel_rot = zeros(3,qsim.N);

end

