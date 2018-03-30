function [ xdot ] = Q_ode( t,x,u,ut,method )
% T. Reynolds, RAIN Lab. Updated: 8.8.17

% Quaternion ODE formula used for propagating attitude motion. Equations
% are:
%   \dot{q} = 0.5 * q \otimes w
%   \dot{w} = J^{-1} ( u - w x Jw )

q   = x(1:4);
q   = q./norm(q);
w   = x(5:7);

if nargin == 3
    uu  = u;
else
    uu  = zeros(3,1);
    for i = 1:3
        uu(i)   = interp1(ut,u(i,:),t,method);
    end
end

w_quat  = [w; 0];

qdot    = 0.5*Q_mult(q,w_quat);
wdot    = J\(uu - skew(w)*J*w);

xdot = [qdot; wdot];

end

