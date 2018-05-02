%% RAINDrop Quadrotor Simulator: Main Simulation
%Thomas Miesen

%Initialize parameters for quadrotor (see QR_init file)
QR_init();

%Initialize other simulation-specific variables
u = zeros(4, qsim.N);          %Motor thrust inputs
state = zeros(12, qsim.N);     %True state
measured = zeros(7, qsim.N);   %Measureable State
des = zeros(12, qsim.N);       %Desired state setpoint   
motor_cmd = zeros(4, qsim.N);  %Motor command values
cont_PID = zeros(4,1);         %Motor command for PID
accel_state = zeros(3,6);      %True acceleration state


%% Set up Linearized Matrixes
QR_Linearized_Matrices();
% A,B >> Full State
% A_r,B_r >> Rotational Only

 
%% LQR Controller Setup
%Linearization Conitions **(not initial conditions)**
state(:,1) = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]; %full state (only yaw will affect linearization)
u(:,1) = [quad.m*qsim.g/4; quad.m*qsim.g/4; quad.m*qsim.g/4; quad.m*qsim.g/4];

%Initialize LQR
QR_LQR_init();

%Setting up as a K gain, to allow for adjustments to xyz elements based on yaw angle. (it refers to the original K_lqr_full
K = quad_lqr.K_lqr_full;


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


%% Other Definitions
%Trajectory Parameters
omega = 0.075;
r = 0.5;

%% System Response
for t = 1:qsim.N-1
%% Sensor data import
%Collect state measurement (note rotation is measurable from vicon system)
%measured(:,t) = C*state(:,t);
measured(:,t) = C*state(:,t)+H*randn(12,1);

%Update pos and Rot values at set rates
if (qsim.t_loop_pos > quad.pos_sample_rate)
    % IMPORT VICON POSITON DATA AND ESTIMATE VELOCITY
    QR_Vicon_Import_and_Estimation();
end
if qsim.t_loop_rot > quad.rot_sample_rate
    % IMPORT VICON ANGULAR RATE DATA AND ACCEL DATA AND ESTIMATE ANGLE
    QR_MPU6050_Import_and_Estimation();
end


%% Trajectory Generation
%Cork-screw
%des(1:3,t) = [r*0.5*qsim.dt*t*cos(qsim.dt*t*omega);r*0.5*qsim.dt*t*sin(qsim.dt*t*omega);0.15*qsim.dt*t]*1000;

%Test setting a desired yaw
des(6,t) = 10*qsim.dt*t*pi()/180;

%Desired location, with simply linear trajectory planning
%This updates every time a new position value has been recieved.
if qsim.t_loop_pos > quad.pos_sample_rate
    %% USER TRAJECTORY GENERATION %%%%%%%%%
    %vvvvvvvvvvvvvv Place user defined algorithm here vvvvvvvvvvvv
    
    des(1:3,t) = QR_LinearTrajectory_Ccode_similar(measurement_buffer.pos, quad.desired_position, quad.previous_desired_position, quad.max_velocity, quad.pos_sample_rate);
    
    %^^^^^^^^^^^^^^ Place user defined algorithm here ^^^^^^^^^^^^
    
    quad.previous_desired_position = des(1:3,t);
else
    if (t>1)
        des(1:3,t) = quad.previous_desired_position;
    end
end

%Switch desired locations mid-flight to test response    
if t > qsim.N/4 && t <= qsim.N/2
    quad.desired_position = [1000; -1000; 1500];  
    elseif t > qsim.N/2 && t <= 3*qsim.N/4
      quad.desired_position = [-1000; -1000; 1500];  
    elseif t > 3*qsim.N/4
     quad.desired_position = [1000; 1000; 1500];
end

%% Test momentary GPS communication loss (0.25s) 
% if (t > 5.0/qsim.dt && t < 5.25/qsim.dt)
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

%%  Error Calculation
% Calculate relevant position or rotational errors
%Position/Velocity error
e_p = [measurement_buffer.pos(:,1); zeros(3,1); measurement_buffer.vel(:,1); zeros(3,1)] - [des(1:3,t);zeros(3,1);des(7:9,t);zeros(3,1)];
%Rotational/Rate error
e_r = [measurement_buffer.rot(:,1);measurement_buffer.ang(:,1)] - [des(4:6,t);des(10:12,t)];
%Full State Error
e_full = [e_p(1:3,1); e_r(1:3,1); e_p(4:6,1); e_r(4:6,1)];
%Rotational error sum
err_sum = err_sum + qsim.dt*(measurement_buffer.rot(:,1) - des(4:6,t));

%% Full State Controllers %%
%% Full-State LQR (no saturation terms)
%u(:,t) = -K*([measurement_buffer.pos(:,1); measurement_buffer.rot(:,1); measurement_buffer.vel(:,1); measurement_buffer.ang(:,1)] - des(:,t));

%% Full-State LQR (with saturation terms)
%u(:,t) = QR_Position_LQR(quad_lqr, measurement_buffer, quad, e_full, u(:,t));


%% Cascaded Controllers %%
%% Position LQR (with saturation terms)
u(:,t) = QR_Position_LQR(quad_lqr, measurement_buffer, quad, e_p, u(:,t));

%% Rotational LQR
%u(:,t) = QR_Rotational_LQR(quad_lqr, quad, e_r, u(:,t));

%% Rotational PID Control
%Calculate Attitude PID control values (note: errors are converted to deg)
cont_PID(:,1) = QR_Rotational_PID(quad_pid, quad, e_r*180/pi(), err_sum*180/pi());


%% USER SPECIFIC CONTROLLER
%vvvvvvvvvvvvvv Place user defined algorithm here vvvv

    %(Note: Comment out above controllers if needed.)

%^^^^^^^^^^^^^^ Place user defined algorithm here ^^^^
%%





%% Motor Control Applicaiton
%Bound the control values to actual control range (1000us-2000us) and
%convert back to force to account for rounding errors.

%(Uniform) Motor Pulse Noise from Interrupts (+0-14us)
noise.motor_pulse_noise = round(10+15*randn(1,1));

%Convert to motor commands, cap at reasonable values, then convert back.
QR_Motor_Thrust_Application();

%% DESCRETE SAMPLING CONTROL APPLICATION
%Applies controls at the update rate of the physical control loop.
%Note that this is independent of the sampling rates of the sensors.
if qsim.t_loop >= quad.controller_update_rate
    measurement_buffer.control = u(:,t);
    measurement_buffer.motor = motor_cmd(:,t);
    qsim.t_loop = 0;
else
    u(:,t) = measurement_buffer.control;
    motor_cmd(:,t) = measurement_buffer.motor;
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

%[state(:,t+1),acc(:,1)] = QR_VariableYaw_NL_Dyn((q(:,t)),u(:,t),quad,qsim);
[state(:,t+1),accel_state(:,1)] = QR_VariableYaw_NL_Dyn((state(:,t)+F*randn(12,1)),u(:,t),quad,qsim);

%% Ground Condition
%Simulates QR on ground (if is at 0 height, then it can't translate in xy)
if (state(3,t+1) <= 0)
    state(1:2,t+1) = state(1:2,t);
    state(3,t+1) = 0;
end
end

%% Plots
%Plot state dynamics
figure
[m,n] = size(state);
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