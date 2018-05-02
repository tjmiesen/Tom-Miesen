%% RAINDrop Quadrotor Simulator: LQR Definitions and Init
%Thomas Miesen        

%Descrete time LQR function for Continuous Plant Dyanamics
%% Position LQR
%Q definitions
quad_lqr.Q = eye(12)*0.00001;           %Sets general
quad_lqr.Q(3,3) = 1;                    %z position
quad_lqr.Q(4:5,4:5) = eye(2)*100;       %roll,pitch
quad_lqr.Q(6,6) = 1000;                 %yaw
quad_lqr.Q(7:8,7:8) = eye(2)*0.000001;  %xy velocity
quad_lqr.Q(9,9) = 0.000001;             %z velocity
quad_lqr.Q(10:11,10:11) = eye(2)*250;   %roll,pitch rates
quad_lqr.Q(12,12) = 50;                 %yaw rate

%R definitions
quad_lqr.R = eye(4)*1;   

%Determine optimal full-state gain
quad_lqr.K_lqr_full = lqrd(A(state(:,1),u(:,1)),B(state(:,1)),quad_lqr.Q,quad_lqr.R,quad.pos_sample_rate);

%% Rotational LQR
%Q definitions
quad_lqr.Q_r = eye(6);                  %Sets general
quad_lqr.Q_r(1:2,1:2) = 100*eye(2);     %roll,pitch
quad_lqr.Q_r(3,3) = 1000;               %yaw
quad_lqr.Q_r(4:5,4:5) = 250*eye(2);     %roll, pitch rates
quad_lqr.Q_r(6,6) = 50;                 %yaw rates

%R definitions
quad_lqr.R_r = 1*eye(4);

%Determine optimal rotational gain
quad_lqr.K_lqr_rot = lqrd(A_r,B_r,quad_lqr.Q_r,quad_lqr.R_r,quad.pos_sample_rate);