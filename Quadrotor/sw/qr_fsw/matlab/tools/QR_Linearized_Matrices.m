%% RAINDrop Quadrotor Simulator: Linearized Dynamical Equations
%Thomas Miesen


%% Linearized Rotational State and Control Matrix
%Rotational State Matrix
A_r = [eye(3)*0 eye(3); eye(3)*0 eye(3)*0];

%Rotational Control Matrix
B_r =  [0 0 0 0; %r                 1
        0 0 0 0; %p                 2
        0 0 0 0; %ya                3
        quad.d/(quad.Ix) -quad.d/(quad.Ix) quad.d/(quad.Ix) -quad.d/(quad.Ix); %Vr  4
       -quad.d/(quad.Iy) -quad.d/(quad.Iy) quad.d/(quad.Iy)  quad.d/(quad.Iy); %Vp  5
       -quad.a/quad.Iz    quad.a/quad.Iz   quad.a/quad.Iz   -quad.a/quad.Iz];  %Vya 6

%% Linearized State and Control Matrices with the Variable Yaw Elements Left for LQR Recalculation
%Full State Matrix
G = @(q,u) [0 0 0 (sum(u(:,1),1)/quad.m)*sin(q(6,1)) (sum(u(:,1),1)/quad.m)*cos(q(6,1)) 0;
            0 0 0 (sum(u(:,1),1)/quad.m)*cos(q(6,1)) -(sum(u(:,1),1)/quad.m)*sin(q(6,1)) 0;
            0 0 0 0 0 0;
            0 0 0 0 0 0;
            0 0 0 0 0 0;
            0 0 0 0 0 0];
A = @(q,u) [eye(6)*0 eye(6); G(q,u) eye(6)*0];

%Full Control Matrix
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
            -quad.a/quad.Iz    quad.a/quad.Iz   quad.a/quad.Iz   -quad.a/quad.Iz];         %Vya 12


%% Output and Noise Definitions

%Output Matrix (Can only directly measure X, Y, Z, Yaw, dr/dt, dp/dt, and dyaw/dt from state)
%(note acceleration is output from the non-linear dynamics)
C = [1 0 0 0 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 0 0 0 0 1];

%State Noise Matrix
F = [eye(3)*noise.pos, zeros(3,9);
     zeros(3,3), eye(3)*noise.rot, zeros(3,6);
     zeros(3,6), eye(3)*noise.vel, zeros(3,3);
     zeros(3,9), eye(3)*noise.ang_vel];

%Measurement Noise Matrix
H = [eye(3)*noise.pos_measurement, zeros(3,9);
     zeros(1,5), noise.rot_measurement, zeros(1,6);
     zeros(3,9), eye(3)*noise.ang_vel_measurement];
