function [ dq ] = screw_to_DQ( theta,d,l,m )

% T. Reynolds, RAIN Lab
% Updated: 9.5.17
% Converts four screw parameters to a dual quaternion
% Input: - theta: rotation angle about axis l [rad]
%         - d: translation distance along l [length]
%         - l: unit vector describing motion direction
%         - m: moment vector of l

ct2     = cos(theta/2);
st2     = sin(theta/2);

w_r     = ct2;
v_r     = l.*st2;
w_d     = -0.5*d*st2;
v_d     = st2*m + 0.5*d*ct2*l;

q1  = [v_r; w_r];
q1  = reshape(q1,4,1);
q2  = [v_d; w_d];
q2  = reshape(q2,4,1);

dq  = [q1; q2];

end

