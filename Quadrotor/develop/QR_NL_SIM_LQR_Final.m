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
quad.sample_rate = 0.02; %Sample rate of controller in s

%Simulation Parameters
qsim.g = 9810;  %mm/s^2
qsim.tmax = 50;
qsim.dt = 0.01; %simulation dt in s (NOTE: 10ms is ONLY for to speed up the SETUP process)
qsim.N = qsim.tmax/qsim.dt;
qsim.t_loop = 0;

dt = qsim.dt;
tmax = qsim.tmax;
N = qsim.N;

%Variable Init
temp_control = quad.sample_rate;
%temp_control = 0;
u =     zeros(4, qsim.N); %Postion control inputs
q =     zeros(12, qsim.N); %state
y =     zeros(9, qsim.N); %Output
des =   zeros(12, qsim.N);
mot =   zeros(4, qsim.N); %Motor Command Values

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
             quad.d/(quad.Ix) -quad.d/(quad.Ix) quad.d/(quad.Ix) -quad.d/(quad.Ix); %Vr  10
            -quad.d/(quad.Iy) -quad.d/(quad.Iy) quad.d/(quad.Iy)  quad.d/(quad.Iy); %Vp  11
            -quad.a/quad.Iz quad.a/quad.Iz quad.a/quad.Iz -quad.a/quad.Iz];         %Vya 12

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
noise.pos = 1.0;
noise.rot = 0.01*pi()/180;
noise.vel = 0;
noise.ang_vel = 0.01;

pos_noise = 1.0;
rot_noise = 0.01*pi()/180;
vel_noise = 0;
ang_vel_noise = 0.01;

F = [eye(3)*noise.pos, zeros(3,9);
     zeros(3,3), eye(3)*noise.rot, zeros(3,6);
     zeros(3,6), eye(3)*noise.vel, zeros(3,3);
     zeros(3,9), eye(3)*noise.ang_vel];

%Determine Discrete Time System
% sys = ss(A(q(:,1),u(:,1)),B(q(:,1)),C,0);
% sysd = c2d(sys,quad.sample_rate);
% Ad = sysd.a
% Bd = sysd.b;

%Measurement Noise Matrix Definitions
noise.pos_measurement = 0.005;
noise.rot_measurement = 0.01*pi()/180;
noise.vel_measurement = 0;
noise.ang_vel_measurement = 0.05;

H = [eye(3)*noise.pos_measurement, zeros(3,9);
     zeros(3,3), eye(3)*noise.rot_measurement, zeros(3,6);
     zeros(3,9), eye(3)*noise.ang_vel_measurement];
 
%% Optimal LQR Controller
%Based off the cost function int[0-inf](||d(t)||^2 + ||u(t)||^2)dt
%k_lqr is found such that u* = k_lqr*x(t)

%Initialize P = Qf, which is 1.
QR_LQR.Q = C'*C*0.05;
QR_LQR.Q(4:6,4:6) = eye(3)*5;
QR_LQR.P = QR_LQR.Q;
QR_LQR.R = eye(4)*1;

%Import LQR values (note: discrete time ricatti is in the loop)
Q = QR_LQR.Q;
P = QR_LQR.P;
R = QR_LQR.R;

%Descrete time LQR function for Continuous Plant Dyanamics
K_lqr = lqrd(A(q(:,1),u(:,1)),B(q(:,1)),Q,R,quad.sample_rate);
K = K_lqr;
Pd = P;

%Pre-determine LQR gains for various Yaw Angles. Simulation suggests that
%these gains are stable for a +/- 12 deg. So have partitioned them up into
%quadrants.
% [K22p P e] = lqrd(A([q(1:5,1);22.5*pi()/180;q(7:12,1)],u(:,1)),B(q(:,1)),Q,R,quad.sample_rate);
% [K45p P e] = lqrd(A([q(1:5,1);45*pi()/180;q(7:12,1)],u(:,1)),B(q(:,1)),Q,R,quad.sample_rate);
% [K67p P e] = lqrd(A([q(1:5,1);67.5*pi()/180;q(7:12,1)],u(:,1)),B(q(:,1)),Q,R,quad.sample_rate);
% [K90p P e] = lqrd(A([q(1:5,1);90*pi()/180;q(7:12,1)],u(:,1)),B(q(:,1)),Q,R,quad.sample_rate);
% 
% [K22n P e] = lqrd(A([q(1:5,1);-22.5*pi()/180;q(7:12,1)],u(:,1)),B(q(:,1)),Q,R,quad.sample_rate);
% [K45n P e] = lqrd(A([q(1:5,1);-45*pi()/180;q(7:12,1)],u(:,1)),B(q(:,1)),Q,R,quad.sample_rate);
% [K67n P e] = lqrd(A([q(1:5,1);-67.5*pi()/180;q(7:12,1)],u(:,1)),B(q(:,1)),Q,R,quad.sample_rate);
% [K90n P e] = lqrd(A([q(1:5,1);-90*pi()/180;q(7:12,1)],u(:,1)),B(q(:,1)),Q,R,quad.sample_rate);


%Trajectory Parameters
omega = 0.075;
r = 0.5;

%Max linearized motor commands allowed
mmax = 2000;
mmin = 1000;

%% System Response
for t = 1:qsim.N-1
%% Trajectory Generation
%Circular Path
%des(1:3,t) = [r*cos(dt*t*omega);r*sin(dt*t*omega);1.0]*1000;

%Cork-screw
des(1:3,t) = [r*0.05*dt*t*cos(dt*t*omega);r*0.05*dt*t*sin(dt*t*omega);0.015*dt*t]*1000;

%Test setting a desired yaw
des(6,t) = 2*dt*t*pi()/180;

%Straight Path
%des(1:3,t) = [0.025*dt*t; 0.05*dt*t; 0.025*dt*t]*1000;
 
% %Triangle
% des(1:3,t) = [0.0025*dt*t; 0.005*dt*t; 0.025*dt*t]*1000;    
% if t > N/4 && t <= N/2
%   des(1:3,t) = des(1:3,round(N/4)) + [0.075*dt*(t-N/4); 0; 0]*1000;  
% elseif t > N/2 && t <= 3*N/4
%   des(1:3,t) = des(1:3,round(N/2)) + [0; -0.075*dt*(t-N/2); 0]*1000;  
% elseif t > 3*N/4
%   des(1:3,t) = des(1:3,round(3*N/4)) + [-0.075*dt*(t-3*N/4); 0.075*dt*(t-3*N/4); 0]*1000;
% end

% %Ends flight path preemptively and QR is supposed to stop at point, allows inspection 
%  if t > 3*N/4
%    des(1:3,t) = des(1:3,t-1);  
%    %Test setting desired yaw back to 0
%    des(6,t) = des(6,t-1)*0.75;
% 
%  end

%% Control Inputs
% %APPLY LQR CONTROL
%Re-orient the optimal LQR gain to the QR current yaw angle
K = [ K_lqr(1,1)*(cos(q(6,t))-sin(q(6,t))), K_lqr(1,2)*(cos(q(6,t))+sin(q(6,t)))     ,K_lqr(1,3:6),      K_lqr(1,7)*(cos(q(6,t))-sin(q(6,t))), K_lqr(1,8)*(cos(q(6,t))+sin(q(6,t))),        K_lqr(1,9:12);
      K_lqr(2,1)*(cos(q(6,t))+sin(q(6,t))), K_lqr(2,2)*(cos(q(6,t))-sin(q(6,t)))     ,K_lqr(2,3:6),      K_lqr(2,7)*(cos(q(6,t))+sin(q(6,t))), K_lqr(2,8)*(cos(q(6,t))-sin(q(6,t))),        K_lqr(2,9:12);
      K_lqr(3,1)*(cos(q(6,t))+sin(q(6,t))), K_lqr(3,2)*(cos(q(6,t))-sin(q(6,t)))     ,K_lqr(3,3:6),      K_lqr(3,7)*(cos(q(6,t))+sin(q(6,t))), K_lqr(3,8)*(cos(q(6,t))-sin(q(6,t))),        K_lqr(3,9:12);
      K_lqr(4,1)*(cos(q(6,t))-sin(q(6,t))), K_lqr(4,2)*(cos(q(6,t))+sin(q(6,t)))     ,K_lqr(4,3:6),      K_lqr(4,7)*(cos(q(6,t))-sin(q(6,t))), K_lqr(4,8)*(cos(q(6,t))+sin(q(6,t))),        K_lqr(4,9:12)];
u(:,t) = -K*(q(:,t)-des(:,t));


%Yaw-Specific LQR Control (Increments of 22.5 deg)
% if(q(6,t) < 11.25*pi()/180 && q(6,t)> -11.25*pi()/180)
%     u(:,t) = -K*(q(:,t)-des(:,t));
% elseif(q(6,t)< 33.75*pi()/180 && q(6,t)>= 11.25*pi()/180)
%     u(:,t) = -K22p*(q(:,t)-des(:,t));
% elseif(q(6,t)< 56.25*pi()/180 && q(6,t)>= 33.75*pi()/180)
%     u(:,t) = -K45p*(q(:,t)-des(:,t));
% elseif(q(6,t)< 78.75*pi()/180 && q(6,t)>= 56.25*pi()/180)
%     u(:,t) = -K67p*(q(:,t)-des(:,t));
% elseif(q(6,t)< 101.25*pi()/180 && q(6,t)>= 78.75*pi()/180)
%     u(:,t) = -K90p*(q(:,t)-des(:,t));
%     
% elseif(q(6,t)> -33.75*pi()/180 && q(6,t)<= -11.25*pi()/180)
%     u(:,t) = -K22n*(q(:,t)-des(:,t));
% elseif(q(6,t)> -56.25*pi()/180 && q(6,t)<= -33.75*pi()/180)
%     u(:,t) = -K45n*(q(:,t)-des(:,t));
% elseif(q(6,t)> -78.75*pi()/180 && q(6,t)<= -56.25*pi()/180)
%     u(:,t) = -K67n*(q(:,t)-des(:,t));
% elseif(q(6,t)> -101.25*pi()/180 && q(6,t)<= -78.75*pi()/180)
%     u(:,t) = -K90n*(q(:,t)-des(:,t));
% end

%Bound the control values to actual control range (1000us-1900us)
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

   %Convert motor command back to a force (kg*mm/s^2
   u(i,t) = 1000*((2.3159e-6)*mot(i,t)^2 - (3.5209e-3)*mot(i,t) + 1.2079)/4;
end

%DESCRETE SAMPLING CONTROL APPLICATION
%Test descrete sampling rate of data (eg: position data provided at rate of
%~10-15ms, so control value does not get updated until new data is recieved)
if qsim.t_loop >= quad.sample_rate
    temp_control = u(:,t);
    qsim.t_loop = 0;
else
    u(:,t) = temp_control;
end
qsim.t_loop = qsim.t_loop + qsim.dt;

%Non-linear Dynamics (Noise and Noise-less Available)
%Note these are note discrete as the real system is continuous by nature.
q(:,t+1) = QR_VariableYaw_NL_Dyn((q(:,t)),u(:,t),quad,qsim);
%q(:,t+1) = QR_VariableYaw_NL_Dyn((q(:,t)+F*randn(12,1)),u(:,t),quad,qsim);

y(:,t) = C*q(:,t);
%y(:,t) = C*q(:,t)+H*randn(12,1);
end


%Plot state dynamics
figure
[m,n] = size(q);
plot((1:n-1)*dt, y(1:3,1:n-1))
xlabel('Time')
ylabel('Position Output')
legend('X','Y','Z')
title('Position/Rotational Dynamics (mm)')

%Plot rot dynamics
figure
plot((1:n-1)*dt, y(4:6,1:n-1)*180/pi())
xlabel('Time')
ylabel('Rotation Output')
legend('R','P','Yaw')
title('Rotational Dynamics (deg)')
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