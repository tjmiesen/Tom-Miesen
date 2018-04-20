function [q, acc] = QR_VariableYaw_NL_Dyn(q,u,quad,qsim)

%Pull in simulation parameters
g = qsim.g;
dt = qsim.dt;

%Non-linear dynamics can be described by the following vectors:
%% Translational
% accel = @(q,u) [sum(u(:,1),1)/quad.m*(sin(q(5,1))*cos(q(6,1))+sin(q(4,1))*sin(q(6,1)));
%                 sum(u(:,1),1)/quad.m*(sin(q(4,1))*cos(q(6,1))-sin(q(5,1))*sin(q(6,1)));
%                 sum(u(:,1),1)/quad.m*cos(q(4,1))*cos(q(5,1))-g];

%Damped system
accel = @(q,u) [sum(u(:,1),1)/quad.m*(sin(q(5,1))*cos(q(6,1))+sin(q(4,1))*sin(q(6,1))) - (quad.k1/quad.m)*q(7,1);
                sum(u(:,1),1)/quad.m*(sin(q(4,1))*cos(q(6,1))-sin(q(5,1))*sin(q(6,1))) - (quad.k2/quad.m)*q(8,1);
                sum(u(:,1),1)/quad.m*cos(q(4,1))*cos(q(5,1))- g - (quad.k3/quad.m)*q(9,1)];

            
            velocity = q(7:9,1) + dt*accel(q(:,1),u(:,1));
%position = q(1:3,1) + dt*velocity;
position = q(1:3,1) + dt*velocity + 0.5*dt*dt*accel(q(:,1),u(:,1));

%% Rotational
% ang_accel =    @(q,u)  [quad.d/quad.Iy*(-u(1,1)+u(2,1)-u(3,1)+u(4,1))*cos(q(4,1))*cos(q(5,1));
%                         quad.d/quad.Ix*(-u(1,1)-u(2,1)+u(3,1)+u(4,1))*cos(q(4,1))*cos(q(5,1));
%                         quad.C_yaw*quad.a/quad.Iz*(u(1,1)-u(2,1)-u(3,1)+u(4,1))*cos(q(4,1))*cos(q(5,1))];

%Damped system
ang_accel =    @(q,u)  [quad.d/quad.Iy*(-u(1,1)+u(2,1)-u(3,1)+u(4,1))*cos(q(4,1))*cos(q(5,1)) - (quad.d*quad.k4/quad.Iy)*q(10,1);
                        quad.d/quad.Ix*(-u(1,1)-u(2,1)+u(3,1)+u(4,1))*cos(q(4,1))*cos(q(5,1)) - (quad.d*quad.k5/quad.Ix)*q(11,1);
                        quad.C_yaw*quad.a/quad.Iz*(u(1,1)-u(2,1)-u(3,1)+u(4,1))*cos(q(4,1))*cos(q(5,1)) - (quad.a*quad.k5/quad.Iz)*q(12,1)];

                    
ang_velocity = q(10:12,1) + dt*ang_accel(q(:,1),u(:,1));
%rotation = q(4:6,1) + dt*ang_velocity;
rotation = q(4:6,1) + dt*ang_velocity + 0.5*dt*dt*ang_accel(q(:,1),u(:,1));

%% Output
%% Euler Approx
% q(:,1) =   [position;
%             rotation;
%             velocity;
%             ang_velocity];

%% RK4
% [dv/dt; ddv/ddt] = [dv/dt; forces acting on body]
q(:,1) = QR_RK4_Solver(@(q,u) [q(7:12);accel(q(:,1),u(:,1));ang_accel(q(:,1),u(:,1))],dt,q(:,1),u(:,1));
acc(:,1) = accel(q(:,1),u(:,1));
