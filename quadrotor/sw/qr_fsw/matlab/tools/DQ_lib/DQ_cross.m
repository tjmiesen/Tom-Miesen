function [ dq_cross ] = DQ_cross( dq )

if length(dq) ~= 8 
    error('Input vector is not a DQ')
end

dq   = reshape(dq,8,1);

q1  = dq(1:4);
q2  = dq(5:8);

q1_cross    = Q_cross( q1 );
q2_cross    = Q_cross( q2 );

dq_cross    = [ q1_cross    zeros(4);
                q2_cross    q1_cross ]; 
end

