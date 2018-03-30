%% QR Project for AA548
% Reference for QR dynamics: http://dx.doi.org/10.1109/ROBOT.2010.5509452
clear all
close all

%Add structure formats once non-linear dynamics are added.
%quad = struct;
%quad.m = 0.1;

%Physical Definitions
g = 9810;  %mm/s^2
m = 0.11;  %kg
d = 55;    %mm 
a = 75;    %mm

%Estimating approx mass distribution to be a 50mm x 50mm x 5mm box.
h_quad = 5;
w_quad = 50; 
Ix = m*(h_quad^2 + w_quad^2)/12;
Iy = m*(h_quad^2 + w_quad^2)/12;
Iz = m*(w_quad^2 + w_quad^2)/12;

%Time Definitions
tmax = 100;
dt = 0.001;
N = tmax/dt;

%Variable Init
u = zeros(4,N); %Postion control inputs
w = zeros(4,N); %Cascaded control inputs
des_r = zeros(3,N); %Cascaded control inputs
q = zeros(12,N); %state
q2 = zeros(6,N); %state
y = zeros(9,N); %Output
e = zeros(12,N+1);
des = zeros(12,N);
mot = zeros(4,N);

%Initial Position and rotation
q(:,1) = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]; %LQR Position Controller
des_r(:,1) = [0; 0; 0];

%% Full Quad-rotor System (Linearized about the origin)
%Linearized system (fixed) for dx(t)/dt = Ax(t) + Bu
I = eye(6);
G = [0 0 0 0 g 0;
     0 0 0 g 0 0;
     0 0 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0 0 0 0];
 
A = [I*0 I; G I*0];

B = [0 0 0 0; %x                 1
     0 0 0 0; %y                 2
     0 0 0 0; %z                 3
     0 0 0 0; %r                 4
     0 0 0 0; %p                 5
     0 0 0 0; %ya                6
     0 0 0 0; %Vx                7
     0 0 0 0; %Vy                8
     1/(m) 1/(m) 1/(m) 1/(m);       %Vz  9
     d/(Ix) -d/(Ix) d/(Ix) -d/(Ix); %Vr  10
    -d/(Iy) -d/(Iy) d/(Iy)  d/(Iy); %Vp  11
    -a/Iz a/Iz a/Iz -a/Iz];         %Vya 12

A
B

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
pos_noise = 0.005;
rot_noise = 0.01*pi()/180;
vel_noise = 0;
ang_vel_noise = 0.005;

F = [eye(3)*pos_noise, zeros(3,9);
     zeros(3,3), eye(3)*rot_noise, zeros(3,6);
     zeros(3,6), eye(3)*vel_noise, zeros(3,3);
     zeros(3,9), eye(3)*ang_vel_noise];

%Measurement Noise Matrix Definitions
pos_measurement_noise = 0.05;
rot_measurement_noise = 0.01*pi()/180;
vel_measurement_noise = 0;
ang_vel_measurement_noise = 0.05;

H = [eye(3)*pos_measurement_noise, zeros(3,9);
     zeros(3,3), eye(3)*rot_measurement_noise, zeros(3,6);
     zeros(3,9), eye(3)*ang_vel_measurement_noise];

%Eigenvalues of Open-loop
eig(A)

%% Check controllablility of the system with a simple PBH test.
%n_A = size(A)
%cont_A = rank([B, A*B, A*A*B, A*A*A*B, A*A*A*A*B, A*A*A*A*A*B, A*A*A*A*A*A*B, A*A*A*A*A*A*A*B, A*A*A*A*A*A*A*A*B, A*A*A*A*A*A*A*A*A*B, A*A*A*A*A*A*A*A*A*A*B, A*A*A*A*A*A*A*A*A*A*A*B])
% if (n_A == cont_A)
%     sprintf('System is controllable!')
% end

%% Optimal LQR Controller
%Based off the cost function int[0-inf](||d(t)||^2 + ||u(t)||^2)dt
%k_lqr is found such that u* = k_lqr*x(t)

%Initialize P = Qf, which is 1.
%Q = eye(12);
%Q = [eye(3)*0.1, zeros(3,9); zeros(3,3), eye(3)*0.1, zeros(3,6); zeros(3,6), eye(3)*10, zeros(3,3); zeros(3,9), eye(3)*0.1]
Q = C'*C;
P = Q;
R = eye(4)*0.1;

h = 0.001;
for i=1:100
    %Linearized
    P = P + h*(Q+P*A+A'*P-P*B*inv(R)*B'*P);  %Pt = Pt+h - dt*dPdt
end

%Derived that optimal state feedback gain for cost function
%Linearized
K = -inv(R)*B'*P;

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
mmax = 1900-hover;
mmin = 1000-hover;

%% System Response
for t = 1:N-1
%% Trajectory Generation
%Circular Path
%des(1:3,t) = [r*cos(dt*t*omega);r*sin(dt*t*omega);1.0]*1000;

%Cork-screw
des(1:3,t) = [r*0.05*dt*t*cos(dt*t*omega);r*0.05*dt*t*sin(dt*t*omega);0.015*dt*t]*1000;

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

%Apply motor commands 
m1 =  C_r(1,1) - C_r(1,2) - C_r(1,3) + 1000 + C_p(1,3);
m2 = -C_r(1,1) - C_r(1,2) + C_r(1,3) + 1000 + C_p(1,3);
m3 =  C_r(1,1) + C_r(1,2) + C_r(1,3) + 1000 + C_p(1,3);
m4 = -C_r(1,1) + C_r(1,2) - C_r(1,3) + 1000 + C_p(1,3);

%LQR CONTROL
u(:,t) = K*(q(:,t)-des(:,t));

%Convert thrust to motor speed command (LQR ONLY)
m1 = ((3.5209e-3) + sqrt((3.5209e-3)^2 + 4*(2.3159e-6)*(-1.2079+u(1,t)*4/1000)))/(2*(2.3159e-6));
m2 = ((3.5209e-3) + sqrt((3.5209e-3)^2 + 4*(2.3159e-6)*(-1.2079+u(2,t)*4/1000)))/(2*(2.3159e-6));
m3 = ((3.5209e-3) + sqrt((3.5209e-3)^2 + 4*(2.3159e-6)*(-1.2079+u(3,t)*4/1000)))/(2*(2.3159e-6));
m4 = ((3.5209e-3) + sqrt((3.5209e-3)^2 + 4*(2.3159e-6)*(-1.2079+u(4,t)*4/1000)))/(2*(2.3159e-6));

%Rounding the motor command values to the nearest integer value
m1 = round(m1);
m2 = round(m2);
m3 = round(m3);
m4 = round(m4);
mot(:,t) = [m1;m2;m3;m4];

%Convert motor speeds to applied thrusts
%(note: this is done for LQR as well to account for rounding error in the
%motor speed value)

%Bound the control values to actual control range (1000us-1900us)
for i = 1:4
   %Since the max operating range of the command pulse is 1000-1900, the
   %linearized QR dynamics has the command pulse set to ~460. So we shift
   %down the max and min values to account for this.
   if mot(i,t) > mmax
       mot(i,t) = mmax;
   elseif mot(i,t) < mmin
       mot(i,t) = mmin;
   end

   %Convert motor command back to a force
   u(i,t) = 1000*((2.3159e-6)*mot(i,t)^2 - (3.5209e-3)*mot(i,t) + 1.2079)/4;
end

%Forward Euler approx of position/rotation, q(t+1) = q(t) + dt*q'(t)
%Linearized
q(:,t+1) = q(:,t) + dt*(A*q(:,t)+B*u(:,t));

%Write non-linear dynamics based off of state values using RK4/5

%q(:,t+1) = q(:,t) + dt*(A*q(:,t)+B*u(:,t) + F*randn(12,1));
%y(:,t) = C*q(:,t);
y(:,t) = C*q(:,t)+H*rand(12,1);
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