function [ dqt ] = DQ_SLERP( dq1,dq2,tspan )

% T. Reynolds, RAIN Lab
% Updated: 9.5.17

% Uses Dual-Quaternion Screw Linear intERPolation to interpolate between
% two dual quaternions over the time span tspan.
% Reference: B. Kenwright, Dual-Quaternions: From Classical Mechanics to
% Computer Graphics and Beyond, xbdev.net, October 2012.


if length(dq1) ~= 8 || length(dq2) ~= 8
    error('One of the inputs is not a dual quaternion')
end

% Reshape to column vectors
dq1     = reshape(dq1,8,1);
dq2     = reshape(dq2,8,1);
len     = length(tspan);

% Check that dq1 is a unit dual quaternion
q1  = dq1(1:4);

if norm(q1) == 1
    % nothing
else
    dq1(1:4) = dq1(1:4)./norm(dq1(1:4));
end
    
% Make sure time goes from 0 to 1
if tspan(1) ~= 0
    tspan   = tspan - tpsan(1);
end

if tspan(end) ~= 1
    tspan   = tspan/tspan(end);
end

dqt     = zeros(8,len);

for i = 1:len
   t    = tspan(i);
   temp     = DQ_power(DQ_mult(DQ_conj(dq1),dq2),t);
   dqt(:,i)  = DQ_mult(dq1,temp);
end



end

