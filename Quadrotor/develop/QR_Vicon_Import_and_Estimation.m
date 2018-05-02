%% RAINDrop Quadrotor Simulator: VICON Data Import
%Thomas Miesen

%Assemble historical position buffer
measurement_buffer.pos(:,6) = measurement_buffer.pos(:,5);
measurement_buffer.pos(:,5) = measurement_buffer.pos(:,4);
measurement_buffer.pos(:,4) = measurement_buffer.pos(:,3);
measurement_buffer.pos(:,3) = measurement_buffer.pos(:,2);
measurement_buffer.pos(:,2) = measurement_buffer.pos(:,1);
measurement_buffer.pos(:,1) = round(measured(1:3,t)); %Rounding to emulate integer capping on QR

% VELOCITY ESTIMATION (AVERAGE OF FINITE DIFFERENCE APPROX)
%Dividing by 10 to get into range of working vel estimation
measurement_buffer.vel2(:,1) = 1/5*(5*measurement_buffer.pos(:,1)-measurement_buffer.pos(:,2)-measurement_buffer.pos(:,3)-measurement_buffer.pos(:,4)-measurement_buffer.pos(:,5)-measurement_buffer.pos(:,6))/(quad.pos_sample_rate*10);
measurement_buffer.vel = round(measurement_buffer.vel2(:,1)); %Emulate integer rounding error for vel

% YAW MEASUREMENT FROM VICON (simply replaces current yaw value onboard)
measurement_buffer.rot(3,1) = measured(4,t);