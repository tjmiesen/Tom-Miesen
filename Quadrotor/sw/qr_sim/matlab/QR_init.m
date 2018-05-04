%% RAINDrop Quadrotor Simulator: Initilization (runs at start of main)
% Thomas Miesen -- RAIN Lab

%Clear variables and close all plots
clear variables; close all

% Set paths
addpath(genpath(pwd))
addpath(genpath('../../qr_fsw/matlab/'))

%Add structure formats once non-linear dynamics are added.
quad_lqr = struct;
quad_pid = struct;

% Quad
quad = init_quad();

% Qsim
qsim = init_qsim();

% Measurement
measurement_buffer = init_meas_buffer(qsim);

% Noise
noise = init_noise(quad,qsim);

%Initialize other simulation-specific variables
u           = zeros(4, qsim.N);         % Motor thrust inputs
state       = zeros(12, qsim.N);        % True state
measured    = zeros(7, qsim.N);         % Measureable State
des         = zeros(12, qsim.N);        % Desired state setpoint   
motor_cmd   = zeros(4, qsim.N);         % Motor command values
cont_PID    = zeros(4,1);               % Motor command for PID
accel_state = zeros(3,6);               % True acceleration state
