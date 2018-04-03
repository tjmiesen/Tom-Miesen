function [ dq ] = DQ_rand( )

% computes a random dual quaternion

q   = Q_rand();
r   = randn(3,1);
r   = [r; 0];

qxr     = 0.5*Q_mult(q,r);
qxr     = qxr - dot(q,qxr).*q;

dq  = [q; qxr];


end

