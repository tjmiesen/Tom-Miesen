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
quad.Ix = quad.m*(quad.h^2 + quad.w^2)/12;
quad.Iy = quad.m*(quad.h^2 + quad.w^2)/12;
quad.Iz = quad.m*(quad.w^2 + quad.w^2)/12;
quad.sample_rate = 0.02; %Sample rate of controller in s

%Simulation Parameters
qsim.g = 9810;  %mm/s^2
qsim.tmax = 10;
qsim.dt = 0.001;
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
e =     zeros(12, qsim.N+1);
des =   zeros(12, qsim.N);
mot =   zeros(4, qsim.N); %Motor Command Values

%Initial Position and rotation
q(:,1) = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]; %LQR Position Controller
u(:,1) = [quad.m*qsim.g/4; quad.m*qsim.g/4; quad.m*qsim.g/4; quad.m*qsim.g/4];

%% Full Quad-rotor System (Linearized about the origin)
%Non-linear system dynamics for dx(t)/dt = Ax(t) + Bu
I = eye(6);
G = @(q,u) [0 0 0 0 (sum(u(:,1),1)/quad.m)*cos(q(5,1)) 0;
            0 0 0 (sum(u(:,1),1)/quad.m)*cos(q(4,1)) 0 0;
            0 0 0 -(sum(u(:,1),1)/quad.m)*cos(q(5,1))*sin(q(4,1)) -(sum(u(:,1),1)/quad.m)*cos(q(4,1))*sin(q(5,1)) 0;
            0 0 0 -(quad.d/quad.Iy)*((u(1,1)-u(2,1)+u(3,1)-u(4,1)))*cos(q(5,1))*sin(q(4,1)) -(quad.d/quad.Iy)*((u(1,1)-u(2,1)+u(3,1)-u(4,1)))*cos(q(4,1))*sin(q(5,1)) 0;
            0 0 0 -(quad.d/quad.Ix)*((-u(1,1)-u(2,1)+u(3,1)+u(4,1)))*cos(q(5,1))*sin(q(4,1)) -(quad.d/quad.Ix)*((-u(1,1)-u(2,1)+u(3,1)+u(4,1)))*cos(q(4,1))*sin(q(5,1)) 0;
            0 0 0 0 0 0];
A = @(q,u) [I*0 I; G(q,u) I*0];


%%% Double-check to see if m is needed for rotational rate changes
%%% (shouldn't be since the m is accounted for in the mass-moment of
%%% inertia)
B = @(q)    [0 0 0 0; %x                 1
             0 0 0 0; %y                 2
             0 0 0 0; %z                 3
             0 0 0 0; %r                 4
             0 0 0 0; %p                 5
             0 0 0 0; %ya                6
             sin(q(5,1))/(quad.m) sin(q(5,1))/(quad.m) sin(q(5,1))/(quad.m) sin(q(5,1))/(quad.m); %Vx                7
             sin(q(4,1))/(quad.m) sin(q(4,1))/(quad.m) sin(q(4,1))/(quad.m) sin(q(4,1))/(quad.m); %Vy                8
             cos(q(5,1))*cos(q(4,1))/(quad.m) cos(q(5,1))*cos(q(4,1))/(quad.m) cos(q(5,1))*cos(q(4,1))/(quad.m) cos(q(5,1))*cos(q(4,1))/(quad.m);       %Vz  9
             quad.d/(quad.Ix)*cos(q(5,1))*cos(q(4,1)) -quad.d/(quad.Ix)*cos(q(5,1))*cos(q(4,1)) quad.d/(quad.Ix)*cos(q(5,1))*cos(q(4,1)) -quad.d/(quad.Ix)*cos(q(5,1))*cos(q(4,1)); %Vr  10
            -quad.d/(quad.Iy)*cos(q(5,1))*cos(q(4,1)) -quad.d/(quad.Iy)*cos(q(5,1))*cos(q(4,1)) quad.d/(quad.Iy)*cos(q(5,1))*cos(q(4,1))  quad.d/(quad.Iy)*cos(q(5,1))*cos(q(4,1)); %Vp  11
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
noise.pos = 0.005;
noise.rot = 0.01*pi()/180;
noise.vel = 0;
noise.ang_vel = 0.005;

pos_noise = 0.005;
rot_noise = 0.01*pi()/180;
vel_noise = 0;
ang_vel_noise = 0.005;

F = [eye(3)*noise.pos, zeros(3,9);
     zeros(3,3), eye(3)*noise.rot, zeros(3,6);
     zeros(3,6), eye(3)*noise.vel, zeros(3,3);
     zeros(3,9), eye(3)*noise.ang_vel];

%Determine Discrete Time System
sys = ss(A(q(:,1),u(:,1)),B(q(:,1)),C,0);
sysd = c2d(sys,quad.sample_rate);
Ad = sysd.a;
Bd = sysd.b;

%Measurement Noise Matrix Definitions
noise.pos_measurement = 0.05;
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
QR_LQR.P = QR_LQR.Q;
QR_LQR.R = eye(4)*1;

%Import LQR values (note: discrete time ricatti is in the loop)
Q = QR_LQR.Q;
P = QR_LQR.P;
R = QR_LQR.R;
% for i = 0:50
%     P = Ad'*P*Ad-(Ad'*P*Bd)*inv(R+Bd'*P*Bd)*(Bd'*P*Ad) + Q;
% end
%K = inv(R+Bd'*P*Bd)*Bd'*P*Ad;

%Descrete time LQR function for Continuous Plant Dyanamics
[K P e] = lqrd(A(q(:,1),u(:,1)),B(q(:,1)),Q,R,quad.sample_rate);
%eig(A(q(:,1),u(:,1))-B(q(:,1))*K)

%% Rotational PID controller Parameters
%Current Parameters on QR
kp = 0.05;
ki = 0.025;
kd = 0.0375;

K_pid = [kp -kd ki];
ir = 0;
ip = 0;
iyaw = 0;
imax = 10;
rmax = 20;

%% Position PID Controller Parameters
kpp = 0.0001; %Good: 0.0001
kip = 0.0;  %Good: 0.0
kdp = 0.00015;  %Good: 0.00015
K_pid_p = [kpp+kdp/dt -kdp/dt kip];
ix = 0;
iy = 0;
pmax = 500;

%Position z Controller Parameters
kpz = 0.8;
kiz = 0.001;
kdz = 1.0;
K_pid_z = [kpz+kdz/dt -kdz/dt kiz];
iz = 0;
zmax = 50;
hover = 480;

%Trajectory Parameters
omega = 0.075;
r = 0.5;

%Max linearized motor commands allowed
%mmax = 1900;
%mmin = 1000;
mmax = 2000;
mmin = 1000;

%% System Response
for t = 1:qsim.N-1
%% Trajectory Generation
%Circular Path
%des(1:3,t) = [r*cos(dt*t*omega);r*sin(dt*t*omega);1.0]*1000;

%Cork-screw
%des(1:3,t) = [r*0.05*dt*t*cos(dt*t*omega);r*0.05*dt*t*sin(dt*t*omega);0.015*dt*t]*1000;

%Straight Path
%des(1:3,t) = [0.025*dt*t; 0.05*dt*t; 0.025*dt*t]*1000;

%Triangle
des(1:3,t) = [0.0025*dt*t; 0.005*dt*t; 0.025*dt*t]*1000;    
if t > N/4 && t <= N/2
  des(1:3,t) = des(1:3,round(N/4)) + [0.075*dt*(t-N/4); 0; 0]*1000;  
elseif t > N/2 && t <= 3*N/4
  des(1:3,t) = des(1:3,round(N/2)) + [0; -0.075*dt*(t-N/2); 0]*1000;  
elseif t > 3*N/4
  des(1:3,t) = des(1:3,round(3*N/4)) + [-0.075*dt*(t-3*N/4); 0.075*dt*(t-3*N/4); 0]*1000;
end

%Ends flight path preemptively and QR is supposed to stop at point, allows inspection 
if t > 3*N/4
  des(1:3,t) = des(1:3,t-1);  
end

%% Control Inputs
%Determine error of state from destination list (t+1 simply to store
%previous error, so e(t+1) is the current error)
e(:,t+1) = des(1:12,t) - q(:,t);

%PID control of positon
ix = ix+kip*e(1,t+1)*dt;
iy = iy+kip*e(2,t+1)*dt;
iz = iz+e(3,t+1)*dt;

%Position Error Matrix [Current Error; Previous Error; Error sum] 
E_p = [e(1:3,t+1)'; e(1:3,t)';[ix,iy,iz]];

%Position Control
C_p = (K_pid_p*E_p);            %XY control
C_p(1,3) = (K_pid_z*E_p(:,3));  %Z control

%Saturate XYZ control terms
if C_p(1,1) > pmax
   C_p(1,1) = pmax; 
elseif C_p(1,1) < -pmax
   C_p(1,1) = -pmax; 
end
if C_p(1,2) > pmax
   C_p(1,2) = pmax; 
elseif C_p(1,2) < -pmax
   C_p(1,2) = -pmax; 
end
if C_p(1,3) > zmax
   C_p(1,3) = zmax; 
elseif C_p(1,3) < -zmax
   C_p(1,3) = -zmax; 
end

%Position PID Control Applied as reference input to rot PID
%Update Cascaded Rotational Error
e(4:6,t+1) = [C_p(1,2);C_p(1,1);0] - q(4:6,t);

ir = ir+e(4,t+1)*dt;
ip = ip+e(5,t+1)*dt;
iyaw = iyaw+e(6,t+1)*dt;

if ir > imax
    ir = imax;
elseif ir < -imax
    ir = -imax;
end
if ip > imax
    ip = imax;
elseif ip < -imax
    ip = -imax;
end
if iyaw > imax
    iyaw = imax;
elseif iyaw < -imax
    iyaw = -imax;
end

%Assemble error matrix [Error; Measured Rotation Rates; Error Sum]
E_r = [e(4:6,t+1)'; q(10:12,t)'; [ir,ip,iyaw]];

%Determine attitude control
C_r = (K_pid*E_r);

%Saturate rotational control terms
for i = 1:3
    if C_r(1,i) > rmax
       C_r(1,i) = rmax; 
    elseif C_r(1,i) < -rmax
       C_r(1,i) = -rmax; 
    end
end

%Apply motor commands for PID (if LQR is enable, these will be overwritten)
mot(1,t) =  C_r(1,1) - C_r(1,2) - C_r(1,3) + 1000 + C_p(1,3);
mot(2,t) = -C_r(1,1) - C_r(1,2) + C_r(1,3) + 1000 + C_p(1,3);
mot(3,t) =  C_r(1,1) + C_r(1,2) + C_r(1,3) + 1000 + C_p(1,3);
mot(4,t) = -C_r(1,1) + C_r(1,2) - C_r(1,3) + 1000 + C_p(1,3);


%DISCRETE TIME RICCATI RECURSION (updates at sample rate)
% if qsim.t_loop >= quad.sample_rate
%     [K P e] = lqrd(A(q(:,1),u(:,1)),B(q(:,1)),Q,R,quad.sample_rate);
% end

% %APPLY LQR CONTROL
u(:,t) = -K*(q(:,t)-des(:,t));

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
   mot(i,t) = round(mot(i,t));
   
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
q(:,t+1) = QR_NL_Dyn((q(:,t)),u(:,t),quad,qsim);
%q(:,t+1) = QR_NL_Dyn((q(:,t)+F*randn(12,1)),u(:,t),quad,qsim);

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