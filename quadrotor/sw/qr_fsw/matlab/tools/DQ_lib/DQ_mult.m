function [ pxq ] = DQ_mult( p,q )

% Performs dual quaternion multiplication using the standard quaternion
% rules. Calls the function Q_mult

if length(p) == 6
    p = [p(1:3); 0; p(4:6); 0];
end

if length(q) == 6
    q = [ q(1:3); 0; q(4:6); 0];
end

if length(p) ~= 8 || length(q) ~= 8  
    error('One of the vectors is not a dual quaternion')
end

p   = reshape(p,8,1);
q   = reshape(q,8,1);

p1  = p(1:4);
p2  = p(5:8);
q1  = q(1:4);
q2  = q(5:8);

pxq1    = Q_mult(p1,q1);
pxq2    = Q_mult(p1,q2) + Q_mult(p2,q1);

pxq1    = reshape(pxq1,4,1);
pxq2    = reshape(pxq2,4,1);

pxq     = vertcat(pxq1,pxq2);

end

