function [q] = QR_NL_Dyn(q,u,quad,qsim)

%Pull in simulation parameters
g = qsim.g;
dt = qsim.dt;

%Non-linear dynamics can be described by the following vectors:
%% Translational
accel = @(q,u) [sum(u(:,1),1)/quad.m*sin(q(5,1));
                sum(u(:,1),1)/quad.m*sin(q(4,1));
                sum(u(:,1),1)/quad.m*cos(q(4,1))*cos(q(5,1))-g];
velocity = q(7:9,1) + dt*accel(q(:,1),u(:,1));
position = q(1:3,1) + dt*velocity;

%% Rotational
ang_accel =    @(q,u)  [quad.d/quad.Iy*(u(1,1)-u(2,1)+u(3,1)-u(4,1))*cos(q(4,1))*cos(q(5,1));
                        quad.d/quad.Ix*(-u(1,1)-u(2,1)+u(3,1)+u(4,1))*cos(q(4,1))*cos(q(5,1));
                        quad.a/quad.Iz*(-u(1,1)+u(2,1)+u(3,1)-u(4,1))*cos(q(4,1))*cos(q(5,1))];
ang_velocity = q(10:12,1) + dt*ang_accel(q(:,1),u(:,1));
rotation = q(4:6,1) + dt*ang_velocity;

%% Output
%% Euler Approx
% q(:,1) =   [position;
%             rotation;
%             velocity;
%             ang_velocity];

%% RK4
% [dv/dt; ddv/ddt] = [dv/dt; forces acting on body]
q(:,1) = QR_RK4_Solver(@(q,u) [q(7:12);accel(q,u);ang_accel(q,u)],dt,q,u);

