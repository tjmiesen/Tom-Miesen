%% QR Project for AA548
% Reference for QR dynamics: http://dx.doi.org/10.1109/ROBOT.2010.5509452
clear all
close all

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
qsim.tmax = 10;
qsim.dt = 0.01; %simulation dt in s (NOTE: 10ms is ONLY for to speed up the SETUP process)
qsim.N = qsim.tmax/qsim.dt;
qsim.t_loop = 0;

syms yaw

q = zeros(12,1);
u = zeros(4,1);

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

 
QR_LQR.Q = C'*C*0.05;
QR_LQR.Q(4:6,4:6) = eye(3)*5;
QR_LQR.R = eye(4)*1;
K_lqr = lqrd(A(q(:,1),u(:,1)),B(q(:,1)),QR_LQR.Q,QR_LQR.R,quad.sample_rate);

q(6,1) = 45*pi()/180;

K_true = lqrd(A(q(:,1),u(:,1)),B(q(:,1)),QR_LQR.Q,QR_LQR.R,quad.sample_rate)
%Test applying the yaw relation to the optimal K.

for j = 1:100
K_test = [K_lqr(1,1)*(cos(q(6,1))-sin(q(6,1))),K_lqr(1,2)*(cos(q(6,1))+sin(q(6,1)))     ,K_lqr(1,3:6),      K_lqr(1,7)*(cos(q(6,1))-sin(q(6,1))),K_lqr(1,8)*(cos(q(6,1))+sin(q(6,1))),        K_lqr(1,9:12);
          K_lqr(2,1)*(cos(q(6,1))+sin(q(6,1))),K_lqr(2,2)*(cos(q(6,1))-sin(q(6,1)))     ,K_lqr(2,3:6),      K_lqr(2,7)*(cos(q(6,1))+sin(q(6,1))),K_lqr(2,8)*(cos(q(6,1))-sin(q(6,1))),        K_lqr(2,9:12);
          K_lqr(3,1)*(cos(q(6,1))+sin(q(6,1))),K_lqr(3,2)*(cos(q(6,1))-sin(q(6,1)))     ,K_lqr(3,3:6),      K_lqr(3,7)*(cos(q(6,1))+sin(q(6,1))),K_lqr(3,8)*(cos(q(6,1))-sin(q(6,1))),        K_lqr(3,9:12);
          K_lqr(4,1)*(cos(q(6,1))-sin(q(6,1))),K_lqr(4,2)*(cos(q(6,1))+sin(q(6,1)))     ,K_lqr(4,3:6),      K_lqr(4,7)*(cos(q(6,1))-sin(q(6,1))),K_lqr(4,8)*(cos(q(6,1))+sin(q(6,1))),        K_lqr(4,9:12)]
end
%check to see if matrixes match
E = round(10^9*(K_true - K_test))
%E = (K_true - K_test)

%Conversion relationship between each of the states and the yaw
% T = [-1*(cos(q(6,1))-sin(q(6,1))), 1*(cos(q(6,1))+sin(q(6,1)));
%      -1*(cos(q(6,1))+sin(q(6,1))), -1*(cos(q(6,1))-sin(q(6,1)));
%      1*(cos(q(6,1))+sin(q(6,1))), 1*(cos(q(6,1))-sin(q(6,1)));
%      1*(cos(q(6,1))-sin(q(6,1))), -1*(cos(q(6,1))+sin(q(6,1)))]

