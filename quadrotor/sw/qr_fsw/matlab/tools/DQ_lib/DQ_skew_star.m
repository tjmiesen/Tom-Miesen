function [ dq_skew_star ] = DQ_skew_star( dq )

if length(dq) ~= 8
    error('Input is not a dual quaternion')
end

dq  = reshape(dq,8,1);

q1  = dq(1:4);
q2  = dq(5:8);

q1_skew_star     = Q_skew_star(q1);
q2_skew_star     = Q_skew_star(q2);

dq_skew_star     = [ q1_skew_star     zeros(4);   
                     q2_skew_star     q1_skew_star ];


end


