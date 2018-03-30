function [ dq_cross_star ] = DQ_cross_star( dq )
% ----------------------------------------------------------------------- %
% Computes the right handed dual quaternion cross product matrix.
%
% T. Reynolds -- RAIN Lab 8.1.17
% ----------------------------------------------------------------------- %

if length(dq) ~= 8 
    error('Input vector is not a DQ')
end

dq   = reshape(dq,8,1);

q1  = dq(1:4);
q2  = dq(5:8);

q1_cross_star    = Q_cross_star( q1 );
q2_cross_star    = Q_cross_star( q2 );

dq_cross_star    = [ q1_cross_star    zeros(4);
                     q2_cross_star    q1_cross_star ]; 
end

