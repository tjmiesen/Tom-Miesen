function [ dq_skew ] = DQ_skew( dq )

if length(dq) ~= 8
    if length(dq) == 6
        dq  = [dq(1:3) 0 dq(4:6) 0]';
    else
        error('Input is not a dual quaternion')
    end
end

dq  = reshape(dq,8,1);

q1  = dq(1:4);
q2  = dq(5:8);

q1_skew     = Q_skew(q1);
q2_skew     = Q_skew(q2);

dq_skew     = [ q1_skew     zeros(4);   
                q2_skew     q1_skew ];


end

