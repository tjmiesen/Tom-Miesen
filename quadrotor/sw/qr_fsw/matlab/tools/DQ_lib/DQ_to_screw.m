function [ theta,d,l,m ] = DQ_to_screw( dq )

% T. Reynolds, RAIN Lab
% Updated: 9.5.17
% Converts a dual quaternion to the screw parameters that define rotation
% and translation.
% Output: - theta: rotation angle about axis l [rad]
%         - d: translation distance along l [length]
%         - l: unit vector describing motion direction
%         - m: moment vector of l

if length(dq) ~= 8
    error('Input is not a dual quaternion')
end

dq = reshape(dq,8,1);

q   = dq(1:4);
qt  = dq(5:8);

qv  = q(1:3);
q0  = q(4);

theta   = 2*acos(q0);

if norm(qv) > 1e-10
    l     = qv./norm(qv);
else
    l     = zeros(3,1);
end

l   = reshape(l,3,1);

t   = 2*Q_mult(Q_conj(q),qt);
t   = t(1:3);

d   = dot(t,l);

skewt   = skew(t);
skewl   = skew(l);

if theta == 0 || theta == pi
    m   = NaN(3,1);
    disp('theta is either 0 or pi, screw axis not defined')
else
    m   = 0.5*(skewt*l + skewl*skewt*l*cot(theta/2));
end

m   = reshape(m,3,1);

end

