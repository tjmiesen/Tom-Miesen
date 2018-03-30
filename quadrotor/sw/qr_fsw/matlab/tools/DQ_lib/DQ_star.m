function [ dq_star ] = DQ_star( dq )

% Conjugates a dual quaternion

if length(dq) ~= 8 
    error('Input is not a dual quaternion')
end

dq = reshape(dq,8,1);

dq1     = dq(1:4);
dq2     = dq(5:8);

dq1_star    = Q_star(dq1);
dq2_star    = Q_star(dq2);

dq_star     = [dq1_star; dq2_star];


end

