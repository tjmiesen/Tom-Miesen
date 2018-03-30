function [ norm_dq ] = DQ_norm( dq )

% Computes the dual quaternion norm according to
%   || q ||_{DQ} = q_conj \otimes q 

if length(dq) ~= 8
    error('Input is not a dual quaternion')
end

dq  = reshape(dq,8,1);

dq_conj     = DQ_conj( dq );

norm_dq     = DQ_mult(dq_conj,dq);

end

