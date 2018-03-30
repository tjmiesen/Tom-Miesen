function [ dq_expt ] = DQ_power( dq,t )

% T. Reynolds, RAIN Lab
% Updated: 9.5.17

% Computes dq^t, where dq is a unit dual quaternion and t > 0.

if length(dq) ~= 8
    error('Input is not a dual quaternion')
end

[theta,d,l,m] = DQ_to_screw(dq);

ang     = 0.5*t*theta;

sang    = sin(ang);
cang    = cos(ang);
td2     = 0.5*t*d;

q   = [sang.*l; cang];

qt_vec  = td2*cang.*l + sang*m;
qt  = [qt_vec; -td2*sang];

dq_expt     = [q; qt];


end

